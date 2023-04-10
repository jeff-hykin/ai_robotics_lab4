import sys
import os
from os.path import isabs, isfile, isdir, join, dirname, basename, exists, splitext
from os import remove, getcwd, makedirs, listdir, rename, rmdir
from shutil import move
import glob
import numpy as np
import blissful_basics as bb
import random
import math
import insightface
from insightface.app import FaceAnalysis
import time

logging = False

# 
# basics
# 
if True:
    def now():
        return time.time()
    
    def clip_value(value, minimum, maximum):
        if value < minimum:
            value = minimum
        if value > maximum:
            value = maximum
        return value
    
    prev_time = time.time()
    def time_since_prev():
        global prev_time
        now = time.time()
        output = now - prev_time
        prev_time = now
        return output

# 
# joints
# 
class JointPositions(list):
    """
        Note:
            Everything is in degrees
    """
    torso_joint_max = 40 
    torso_joint_min = -40 
    
    @property
    def torso_joint(self): return self[0]
    @torso_joint.setter
    def torso_joint(self, value):
        value = clip_value(value, minimum=self.torso_joint_min, maximum=self.torso_joint_max)
        if logging and value != self[0]:
            print(f"   torso_joint: {self[0]:.0f}° => {value:.0f}°")
        self[0] = value
    
    neck_swivel_max = 40 
    neck_swivel_min = -40 
    @property
    def neck_swivel(self): return self[1]
    @neck_swivel.setter
    def neck_swivel(self, value):
        value = clip_value(value, minimum=self.neck_swivel_min, maximum=self.neck_swivel_max)
        if logging and value != self[1]:
            print(f"   neck_swivel: {self[1]:.0f}° => {value:.0f}°")
        self[1] = value
    
    head_tilt_max = 40 
    head_tilt_min = -40
    @property
    def head_tilt(self): return self[2]
    @head_tilt.setter
    def head_tilt(self, value):
        value = clip_value(value, minimum=self.head_tilt_min, maximum=self.head_tilt_max)
        if logging and value != self[2]:
            print(f"   head_tilt: {self[2]:.0f}° => {value:.0f}°")
        self[2] = value
    
    # more negative = face the cieling
    head_nod_max = 40 
    head_nod_min = -40
    @property
    def head_nod(self): return self[3]
    @head_nod.setter
    def head_nod(self, value):
        value = clip_value(value, minimum=self.head_nod_min, maximum=self.head_nod_max)
        if logging and value != self[3]:
            print(f"   head_nod: {self[3]:.0f}° => {value:.0f}°")
        self[3] = value
    
    def __repr__(self):
        return f"[ torso_joint={self.torso_joint:.0f}°, neck_swivel={self.neck_swivel:.0f}°, head_tilt={self.head_tilt:.0f}°, head_nod={self.head_nod:.0f}°,  ]"
        

# 
# event manager
# 
class Event(set):
    def trigger(self, *args, **kwargs):
        for each in self:
            each(*args, **kwargs)
    
    @staticmethod
    def when(event):
        def decorator(function_being_wrapped):
            event.add(function_being_wrapped)
            return function_being_wrapped
        return decorator
    
    @staticmethod
    def once(event):
        def decorator(function_being_wrapped):
            def self_removing_function(*args, **kwargs):
                function_being_wrapped(*args, **kwargs)
                try:
                    event.remove(self_removing_function)
                except KeyError:
                    pass
            
            event.add(self_removing_function)
            return function_being_wrapped
        return decorator

# 
# cli helpers
# 
if True:
    def convert_args(raw_args):
        """
        Summary:
            parses sys.argv into keyword arguments, flags (boolean), and positional arguments
            
        Details:
            Returns a list and a dictionary with positional and keyword arguments.

            -This function assumes flags must start with a "-" and and cannot be a 
                number (but can include them).
            
            -Flags should either be followed by the value they want to be associated 
                with (i.e. -p 5) or will be assigned a value of True in the dictionary.

            -The dictionary will map flags to the name given minus ONE "-" sign in
                front. If you use TWO minus signs in the flag name (i.e. --verbose), 
                the dictionary key will be the name with ONE minus sign in front 
                (i.e. {"-verbose":True}).
        
        Arguments:
            raw_args (list): List of positional/keyword arguments. As obtained from
                            sys.argv.

        Returns:
            list: List of positional arguments (i.e. arguments without flags),
                    in order provided.
            dict: Dictionary mapping flag (key is flag minus the first "-") and
                    their values.

        """
        from collections import defaultdict
        def str_is_float(string):
            try:
                float(string)
                return True
            except ValueError:
                return False
        
        def str_is_int(string):
            return string.lstrip('+-').isdigit()
        
        
        args = []
        kwargs = defaultdict(lambda *args: None)
        count = 0
        
        raw_args = list(raw_args)
        while len(raw_args) > 0:
            next_raw_argument = raw_args.pop(0)
            # If the current argument starts with "-", then it's a key
            if next_raw_argument.startswith('--'):
                if len(raw_args) == 0:
                    raise Exception(f'''
                        
                        This argument: {next_raw_argument}
                        expects a value after it (-key value), however it was the last argument
                        Maybe you meant: -{next_raw_argument}
                        (which is just a flag, e.g. no value)
                        
                    ''')
                
                next_arg_is_possibly_negative_number = False
                try:
                    float(raw_args[0])
                    next_arg_is_possibly_negative_number = True
                except Exception as error: pass
                
                if raw_args[0].startswith('-') and not next_arg_is_possibly_negative_number:
                    raise Exception(f'''
                        
                        This argument: {next_raw_argument}
                        expects a value after it (-key value)
                        However it was follow by another key/flag (--key {raw_args[0]})
                        Maybe this would be valid: -{next_raw_argument} {raw_args[0]}
                        (which is just a flag, e.g. no value)
                        
                    ''')
                # consume the next element as the value
                kwargs[next_raw_argument] = raw_args.pop(0)
            # its a flag
            elif next_raw_argument.startswith("-") and not str_is_float(next_raw_argument):
                kwargs[next_raw_argument] = True
            # Else, it's a positional argument without flags
            else:
                args.append(next_raw_argument)
        
        # 
        # convert number arguments to be numbers
        # 
        for each_index, each_value in enumerate(list(args)):
            if str_is_int(each_value):
                args[each_index] = int(each_value)
            if str_is_float(each_value):
                args[each_index] = float(each_value)
        for each_key, each_value in kwargs.items():
            if isinstance(each_value, str):
                if str_is_int(each_value):
                    kwargs[each_key] = int(each_value)
                if str_is_float(each_value):
                    kwargs[each_key] = float(each_value)
        
        # 
        # make the -name vs -name irrelevent 
        # 
        for each_key, each_value in list(kwargs.items()):
            if isinstance(each_key, str):
                if each_key.startswith("--"):
                    kwargs[each_key[1:]] = each_value
                    kwargs[each_key[2:]] = each_value
                elif each_key.startswith("-"):
                    kwargs[each_key[1:]] = each_value
                    kwargs["-"+each_key] = each_value
                else:
                    kwargs["--"+each_key] = each_value
                    kwargs["-"+each_key] = each_value
                
        return args, kwargs

# 
# reactions
# 
if True:
    should_redo_random_0 = bb.countdown(seconds=0.2)
    slow_random_value_0 = random.random()
    def get_slow_random_0():
        global slow_random_value_0
        if should_redo_random_0():
            slow_random_value_0 = random.random()
        return slow_random_value_0
    
    should_redo_random_1 = bb.countdown(seconds=0.5)
    slow_random_value_1 = random.random()
    def get_slow_random_1():
        global slow_random_value_1
        if should_redo_random_1():
            slow_random_value_1 = random.random()
        return slow_random_value_1

    should_redo_random_2 = bb.countdown(seconds=1)
    slow_random_value_2 = random.random()
    def get_slow_random_2():
        global slow_random_value_2
        if should_redo_random_2():
            slow_random_value_2 = random.random()
        return slow_random_value_2
    
    max_joint_angle = 40 # degrees
    class EmotionalPositionGenerators:
        
        @staticmethod
        def surprised(amount=1):
            # saftey check
            amount = clip_value(amount, minimum=0, maximum=1)
            positions = JointPositions([0,0,0,0])
            positions.torso_joint = -20 * amount
            positions.neck_swivel = -19 * amount
            positions.head_tilt   = -12 * amount
            positions.head_nod    =  -8 * amount # head goes back 
            
            # pick a side randomly
            if get_slow_random_1() > 0.5:
                positions.neck_swivel = -1 * positions.neck_swivel
                positions.head_tilt   = -1 * positions.head_tilt
            
            return positions
        
        @staticmethod
        def fearful(amount=1):
            # saftey check
            amount = clip_value(amount, minimum=0, maximum=1)
            positions = JointPositions([0,0,0,0])
            positions.torso_joint = -25 * amount # leans back more
            positions.neck_swivel =  -8 * amount
            positions.head_tilt   =  -7 * amount # small tilt
            positions.head_nod    =  33 * amount # tucks "chin" close to body
            
            # pick a side randomly
            if random.random() > 0.5:
                positions.neck_swivel = -1 * positions.neck_swivel
                positions.head_tilt   = -1 * positions.head_tilt
            
            if amount > 0:
                vertical_variation = 12
                while abs(vertical_variation + positions.head_nod) > max_joint_angle:
                    positions.head_nod /= 2 
                positions.head_nod += random.random()*vertical_variation - 1
            
            return positions
        
        @staticmethod
        def curious(amount=1):
            # saftey check
            amount = clip_value(amount, minimum=0, maximum=1)
            positions = JointPositions([0,0,0,0])
            positions.torso_joint =  14 * amount # leans forward
            positions.neck_swivel =  14 * amount
            positions.head_tilt   = -15 * amount # increased tilt
            positions.head_nod    = -29 * amount # looks towards object
            
            # pick a side randomly
            if get_slow_random_1() > 0.5:
                positions.neck_swivel = -1 * positions.neck_swivel
            if get_slow_random_2() > 0.5:
                positions.head_tilt   = -1 * positions.head_tilt  
            
            return positions
        
        @staticmethod
        def look_at_me(amount=1):
            # saftey check
            amount = clip_value(amount, minimum=0, maximum=1)
            positions = JointPositions([0,0,0,0])
            positions.torso_joint =  14 * amount # leans forward
            positions.neck_swivel =  14 * amount
            positions.head_tilt   = -15 * amount # increased tilt
            positions.head_nod    = -29 * amount # looks towards object
            
            # pick a side randomly
            if get_slow_random_0() > 0.5:
                positions.neck_swivel = -1 * positions.neck_swivel
            # rock back and fourth
            if get_slow_random_1() > 0.5:
                positions.torso_joint   = -1 * positions.torso_joint  
            if get_slow_random_2() > 0.5:
                positions.head_tilt   = -1 * positions.head_tilt  
            
            return positions
# 
# Geometry
# 
if True:
    class Position(list):
        @property
        def x(self): return self[0]
        
        @x.setter
        def x(self, value): self[0] = value
        
        @property
        def y(self): return self[1]
        
        @y.setter
        def y(self, value): self[1] = value
        
        @property
        def z(self): return self[2]
        
        @z.setter
        def z(self, value): self[2] = value
        
        def __repr__(self):
            if len(self) >= 3:
                return f'(x={self.x},y={self.y},z={self.z})'
            elif len(self) == 2:
                return f'(x={self.x},y={self.y})'
            elif len(self) == 1:
                return f'(x={self.x})'
            else:
                return '[]'

    class BoundingBox(list):
        """
        x_top_left, y_top_left, width, height format
        """
        
        @classmethod
        def from_points(cls, *points, top_left=None, bottom_right=None,):
            max_x = -float('Inf')
            max_y = -float('Inf')
            min_x = float('Inf')
            min_y = float('Inf')
            for each in [*points, top_left, bottom_right]:
                if type(each) != type(None):
                    if max_x < each[0]:
                        max_x = each[0]
                    if max_y < each[1]:
                        max_y = each[1]
                    if min_x > each[0]:
                        min_x = each[0]
                    if min_y > each[1]:
                        min_y = each[1]
            top_left = Position(min_x, min_y)
            bottom_right = Position(max_x, max_y)
            width  = abs(top_left.x - bottom_right.x)
            height = abs(top_left.y - bottom_right.y)
            return BoundingBox([ top_left.x, top_left.y, width, height ])
        
        @classmethod
        def from_array(cls, max_x, max_y, min_x, min_y):
            width  = abs(max_x - min_x)
            height = abs(max_y - min_y)
            return BoundingBox([ min_x, min_y, width, height ])
        
        @property
        def x_top_left(self): return self[0]
        
        @x_top_left.setter
        def x_top_left(self, value): self[0] = value
        
        @property
        def y_top_left(self): return self[1]
        
        @y_top_left.setter
        def y_top_left(self, value): self[1] = value
        
        @property
        def x_bottom_right(self): return self.x_top_left + self.width
        
        @property
        def y_bottom_right(self): return self.y_top_left + self.height
        
        @property
        def width(self): return self[2]
        
        @width.setter
        def width(self, value): self[2] = value
        
        @property
        def height(self): return self[3]
        
        @height.setter
        def height(self, value): self[3] = value
        
        @property
        def center(self):
            return Position([
                self.x_top_left + (self.width / 2),
                self.y_top_left + (self.height / 2),
            ])
        
        @property
        def area(self):
            return self.width * self.height
        
        def contains(self, point):
            point = Position(point)
            return (
                self.x_top_left     < point.x and
                self.x_bottom_right > point.x and
                self.y_top_left     < point.y and
                self.y_bottom_right > point.y
            )
            
        def __repr__(self):
            return f'[x_top_left={f"{self.x_top_left:.2f}".rjust(5)},y_top_left={f"{self.y_top_left:.2f}".rjust(5)},width={f"{self.width:.2f}".rjust(5)},height={f"{self.height:.2f}".rjust(5)}]'

    class Geometry:
        @classmethod
        def bounds_to_points(self, max_x, max_y, min_x, min_y):
            return (min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, min_y)
        
        @classmethod
        def bounding_box(self, array_of_points):
            """
            @array_of_points
                the input needs to be an array of tuples (x,y)
                the array can be any number of points
            returns:
                max_x, max_y, min_x, min_y
            """
            max_x = -float('Inf')
            max_y = -float('Inf')
            min_x = float('Inf')
            min_y = float('Inf')
            for each in array_of_points:
                if max_x < each[0]:
                    max_x = each[0]
                if max_y < each[1]:
                    max_y = each[1]
                if min_x > each[0]:
                    min_x = each[0]
                if min_y > each[1]:
                    min_y = each[1]
            return max_x, max_y, min_x, min_y
        
        @classmethod
        def poly_area(self, points):
            """
            @points: a list of points (x,y tuples) that form a polygon
            
            returns: the area of the polygon
            """
            xs = []
            ys = []
            for each in points:
                x,y = each
                xs.append(x)
                ys.append(y)
            return 0.5*np.abs(np.dot(xs,np.roll(ys,1))-np.dot(ys,np.roll(xs,1)))


        @classmethod
        def distance_between(self, point1, point2):
            x1,y1 = point1
            x2,y2 = point2
            dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
            return dist  
        
        def line_intersection(line1, line2):
            """
                Example:
                    line_1 = [
                        [x1, y1,]
                        [x2, y2,]
                    ]
                    line_2 = [
                        [x1, y1,]
                        [x2, y2,]
                    ]
                    x, y = line_intersection(line_1, line_2)
            """
            
            # https://stackoverflow.com/a/20677983/4367134
            xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
            ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

            def det(a, b):
                return a[0] * b[1] - a[1] * b[0]

            divisor = det(xdiff, ydiff)
            if divisor == 0:
                return None

            d = (det(*line1), det(*line2))
            x = det(d, xdiff) / divisor
            y = det(d, ydiff) / divisor
            return x, y
        
        @classmethod
        def rotate(self, point, angle, about):
            """
            @about a tuple (x,y) as a point, this is the point that will be used as the axis of rotation
            @point a tuple (x,y) as a point that will get rotated counter-clockwise about the origin
            @angle an angle in radians, the amount of counter-clockwise roation

            The angle should be given in radians.
            """
            ox, oy = about
            px, py = point

            qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
            qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
            return qx, qy
        
        @staticmethod
        def get_angle_from_origin(origin, target):
            x1, y1 = origin
            x2, y2 = target
            if x1 == x2 and y1 == y2:
                return 0
            
            x = x2 - x1
            y = y2 - y1
            theta = math.atan2(y, x)
            return Geometry.pi_to_negative_pi(theta)

        tau = math.pi*2
        def pi_to_negative_pi(theta):
            # make sure its within positive/negative theta first
            if theta > Geometry.tau:
                theta = theta % Geometry.tau 
            elif theta < -Geometry.tau:
                theta = -theta # modulus works diff for negative in python
                theta = theta % Geometry.tau 
                theta = -theta
            
            # wrap > 180
            if theta > math.pi:
                overshoot_amount = theta % math.pi
                theta = -(math.pi - overshoot_amount)
            # wrap < -180
            elif theta < -math.pi:
                theta = -theta
                overshoot_amount = theta % math.pi
                theta = -(math.pi - overshoot_amount)
                theta = -theta
            
            return theta

        
        @classmethod
        def vector_pointing(self, from_, to):
            return tuple(map(int.__sub__, to, point))
        
        
        


# 
# Face detection stuff
# 
if True:
    import cv2
    def bgr_to_rgb(image):  
        # converting BGR to RGB
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # Download models from: https://github.com/deepinsight/insightface/issues/1896#issuecomment-1023867304
    insightface_app = None
    def init_insightface_app_if_needed():
        global insightface_app
        if insightface_app == None:
            insightface_app = FaceAnalysis(name="buffalo_s", providers=['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider'])
            insightface_app.prepare(ctx_id=1, det_size=(640,640))
    
    def get_faces(image):
        init_insightface_app_if_needed()
        return [ Face(each_face, image) for each_face in insightface_app.get(image) ]
    
    class Face:
        def __init__(self, insight_face, image):
            init_insightface_app_if_needed()
            self.image = image
            self.insight = insight_face
            # arrive in DEGREES
            pitch, yaw, roll = insight_face.pose
            self.nod    = pitch  # negative is down
            self.swivel = yaw    # left of the image is negative
            self.tilt   = roll   # if the person's face was a clock, then its negative when counter-clockwise
            self.width  = abs(self.insight.bbox[2] - self.insight.bbox[0])
            self.height = abs(self.insight.bbox[3] - self.insight.bbox[1])
        
        def __repr__(self):
            relative_x, relative_y = self.relative_position
            return f"Face(age={self.insight.age},nod={self.nod:.2f},swivel={self.swivel:.2f},tilt={self.tilt:.2f},height={self.height:.0f},width={self.width:.0f},relative_x={relative_x*100:.0f},relative_y={relative_y*100:.0f},)"
        
        @property
        def bounding_box(self):
            leftmost_x = self.insight.bbox[0]
            topmost_y = self.insight.bbox[1]
            return BoundingBox([ leftmost_x, topmost_y, self.width, self.height ])
        
        @property
        def relative_position(self):
            """
                Example:
                    relative_x, relative_y = face.relative_position
                    # the position is returned as a proportion, from -1 to 1
                    # an x value of -1 means all the way to the right side of the picture
                    # an y value of -1 means all the way to the top
            """
            face_x, face_y = self.bounding_box.center
            height, width, *channels = self.image.shape
            x_center = width/2
            y_center = height/2
            relative_x = (face_x - x_center)/x_center
            relative_y = (face_y - y_center)/y_center
            return relative_x, relative_y

# 
# dancing tools
# 
if True:
    # - tools:
    #     - movment set
    #         - lean forward
    #         - lean back
    #         - look far left
    #         - look far right
    #         - look far up
    #         - look far down
    #         - face animations
    #     - timing between key points
    #     - interpolation
    #     - list between robots

    # - candiate songs
    #     - try slow songs
    #     - use rviz

    reset        = dict(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
    lean_forward = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    lean_back    = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    look_left    = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    look_right   = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    look_up      = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    look_down    = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    tilt_left    = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)
    tilt_right   = dict(torso_joint=None, neck_swivel=None, head_tilt=None, head_nod=None)

    routine = [
        # time is in seconds
        { "time":     0, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
        { "time":    10, "positions": [             lean_forward,                     None,                     None,                     None, ], },
        { "time":    11, "positions": [                     None,             lean_forward,                     None,                     None, ], },
        { "time":    12, "positions": [                     None,                     None,             lean_forward,                     None, ], },
        { "time":    13, "positions": [                     None,                     None,                     None,             lean_forward, ], },
        { "time":    16, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
        { "time":    18, "positions": [ [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], [ tilt_left, lean_back ], ], },
        { "time":  1000, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
    ]

    def shift_towards(*, new_value, old_value, proportion):
        if proportion == 1:
            return new_value
        if proportion == 0:
            return old_value
        
        difference = new_value - old_value
        amount = difference * proportion
        return old+amount
    
    def convert_routine_to_function(routine):
        if len(routine) == 0:
            raise Exception(f'''The convert_routine_to_function() function got a routine with a length of 0, there needs to be at least one frame in the routine''')
        # standarize the routine values (LazyDict, and flatten the keypoint lists so they're all dict or None)
        for index, each_timestep in enumerate(routine):
            routine[index] = LazyDict(each_timestep)
            new_positions_list = []
            for each_bot_keypoint in routine[index].positions:
                if isinstance(each_bot_keypoint, (dict, type(None))):
                    keypoint = each_bot_keypoint
                if not isinstance(each_bot_keypoint, list):
                    raise Exception(f'''got an unknown value as a keypoint (expected list, dict, or None): {each_bot_keypoint}\nroutine index:{index}''')
                else: # must be a list
                    keypoint = LazyDict()
                    for each_keypoint in each_bot_keypoint:
                        # override any non-None keys
                        for each_key, each_value in each_keypoint.items():
                            if each_value != None:
                                keypoint[each_key] = each_value
                
                # convert to LazyDict
                if not isinstance(each_bot_keypoint, type(None)):
                    keypoint = LazyDict(keypoint)
                    
                new_positions_list.append(keypoint)
            routine[index].positions = new_positions_list
        
        # validate start/end
        for start_or_end_positions in [routine[0].positions, routine[-1].positions]
            for each_bot in start_or_end_positions:
                # make sure start/end of routine specifies values for all joint positions
                assert each_bot.torso_joint != None
                assert each_bot.neck_swivel != None
                assert each_bot.head_tilt != None
                assert each_bot.head_nod != None
        
        # create a joint-centric representation of the data
        number_of_bots = len(routine[0].positions)
        bot_time_mappings = []
        for bot_index in range(number_of_bots):
            time_mappings = LazyDict(
                torso_joint=LazyDict(),
                neck_swivel=LazyDict(),
                head_tilt=LazyDict(),
                head_nod=LazyDict(),
            )
            for each_frame in routine:
                time_value = each_frame.time
                position_updates = each_frame.positions[bot_index]
                if position_updates != None:
                    for each_key, each_value in position_updates.items():
                        # only add an entry if the value is not None
                        if each_value != None:
                            time_mappings[each_key] = each_value
            bot_time_mappings.append(time_mappings)
        
        def joint_at_time(bot_index, joint_name, time):
            key_times = tuple(bot_time_mappings[bot_index][joint_name].items())
            prev_time_and_value = list(key_times[0])
            next_time_and_value = list(key_times[0])
            # by the end of the loop prev_time and next_time will correctly be < and > the time argument
            for next_time, next_value in key_times:
                next_time_and_value = [ next_time, next_value ]
                # found the true "next" point
                if time <= next_time:
                    break
                prev_time_and_value = next_time_and_value
            
            # just need to interpolate the values
            prev_time, prev_value = prev_time_and_value
            next_time, next_value = next_time_and_value
            duration = next_time - prev_time
            if duration == 0:
                return prev_value # prev_value should == next_value at this point
            assert duration > 0
            relative_time_progress = time - prev_time
            progress_between_points = relative_time_progress/duration
            interpolated_value = shift_towards(new_value=next_value, old_value=prev_value, proportion=progress_between_points)
            return interpolated_value
        
        
        from copy import deepcopy
        def time_to_positions(time):
            # out-of-bound times
            if time <= routine[0].time:
                return routine[0].positions
            if time >= routine[-1].time:
                return routine[-1].positions
            
            # in-bound times
            positions_per_bot = []
            for bot_index, each_bot_mapping in enumerate(bot_time_mappings):
                positions = LazyDict()
                for joint_name in bot_time_mappings.keys():
                    positions[joint_name] = joint_at_time(bot_index=bot_index, joint_name=joint_name, time=time)
                positions_per_bot.append(positions)
            
            return positions_per_bot
        
        return time_to_positions
    
    # create a converter function
    time_to_positions = convert_routine_to_function(routine)
    
    time_to_positions(10) # returns the location of all the joints for all the bots at time 10
    
    