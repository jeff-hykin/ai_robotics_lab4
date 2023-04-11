from blissful_basics import LazyDict, print

reset        = dict(torso_joint=0   , neck_swivel=0   , head_tilt=0   , head_nod=0   ) 
lean_forward = dict(torso_joint=1   , neck_swivel=2   , head_tilt=3   , head_nod=4   )
lean_back    = dict(torso_joint=5   , neck_swivel=6   , head_tilt=7   , head_nod=8   )
look_left    = dict(torso_joint=9   , neck_swivel=10   , head_tilt=11   , head_nod=12   )
look_right   = dict(torso_joint=13   , neck_swivel=14   , head_tilt=15   , head_nod=16   )
look_up      = dict(torso_joint=17   , neck_swivel=18   , head_tilt=19   , head_nod=20   )
look_down    = dict(torso_joint=21   , neck_swivel=22   , head_tilt=23   , head_nod=24   )
tilt_left    = dict(torso_joint=25   , neck_swivel=26   , head_tilt=27   , head_nod=28   )
tilt_right   = dict(torso_joint=29   , neck_swivel=30   , head_tilt=31   , head_nod=32   )

routine = [
    # time is in seconds
    { "time":     0, "positions": [                    reset,                    reset,                    reset,                    reset, ], },
    { "time":    10, "positions": [             lean_forward,                     None,                     None,                     None, ], },
    { "time":    11, "positions": [                     None,             lean_forward,                     None,                     None, ], },
    { "time":    12, "positions": [                     None,                     None,             lean_forward,                     None, ], },
    { "time":    13, "positions": [                     None,                     None,                     None,             lean_forward, ], },
    { "time":    16, "positions": [             lean_forward,                    reset,                    reset,                    reset, ], },
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
    return old_value+amount

def convert_routine_to_function(routine):
    if len(routine) == 0:
        raise Exception(f'''The convert_routine_to_function() function got a routine with a length of 0, there needs to be at least one frame in the routine''')
    
    # standarize the routine values (LazyDict, and flatten the keypoint lists so they're all dict or None)
    for index, each_timestep in enumerate(routine):
        routine[index] = LazyDict(each_timestep)
        new_positions_list = []
        for each_bot_keypoint in routine[index].positions:
            if isinstance(each_bot_keypoint, (dict, LazyDict, type(None))):
                keypoint = each_bot_keypoint
            elif not isinstance(each_bot_keypoint, list):
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
    for start_or_end_positions in [routine[0].positions, routine[-1].positions]:
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
                for each_joint_name, each_value in position_updates.items():
                    # only add an entry if the value is not None
                    if each_value != None:
                        time_mappings[each_joint_name][time_value] = each_value
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
            for joint_name in each_bot_mapping.keys():
                positions[joint_name] = joint_at_time(bot_index=bot_index, joint_name=joint_name, time=time)
            positions_per_bot.append(positions)
        
        return positions_per_bot
    
    return time_to_positions

# create a converter function
time_to_positions = convert_routine_to_function(routine)

time_to_positions(11.21321) # returns the location of all the joints for all the bots at time 10