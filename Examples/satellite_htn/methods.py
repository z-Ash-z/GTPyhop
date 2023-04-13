"""
Method definitions for satellite_htn
- Aneesh Chodisetty <aneeshch@umd.edu>, March 29th, 2023
"""

import gtpyhop

################################################################################
# The helper functions.

def mode_in_satellite(state, mode, satellite):
    """
    Method to tell if a mode is supported in the satellite.

    Args:
        state: The state of the system.
        mode: The mode in which we need the data.
        satellite: The satellite in which we are checking the data.

    Returns:
        The instrument if available.
    """
    instruments = list(filter(lambda x : state.on_board[x] == satellite, state.on_board))    
    
    for instrument in instruments:
        if mode in state.supports[instrument]:
            return instrument
    return None

def min_fuel(state, new_direction, mode):
    """
    Function to find the satellite closed to the direction that we need.

    Args:
        state: The current direction.

    Returns:
        The satellite that can point to the direction we need along with the instrument we need for that mode. 
    """
    satellites = list()
    slew_times = list()

    for satellite in state.pointing.keys():
        instrument = mode_in_satellite(state, mode, satellite)
        if instrument and state.power_avail[satellite] == True and state.data_capacity[satellite] >= state.data[(new_direction, mode)]:
            if state.pointing[satellite] == new_direction:
                return (satellite, instrument)
            if state.fuel[satellite] >= state.slew_time[(new_direction, state.pointing[satellite])]:
                satellites.append((satellite, instrument))
                slew_times.append(state.slew_time[(state.pointing[satellite], new_direction)])

    if slew_times:
        return satellites[slew_times.index(min(slew_times))]
    
    return (None, None)

def check_pointing(state, goal):
    """
    Checks the goal and lists the satellites that need change in direction in the current state.

    Args:
        state: The state of the system.
        goal: The goal state for the system.

    Returns:
        The list of satellites that need change in pointing.
    """
    goal_satellites = list()

    satellite_list = goal.pointing.keys()
    
    for satellite in satellite_list:
        if state.pointing[satellite] != goal.pointing[satellite]:
            goal_satellites.append(satellite)
    return goal_satellites

def check_image(state, goal):
    """
    Checks the goal and lists the images that need to be captured.

    Args:
        state: The state of the system.
        goal: The goal state for the system.

    Returns:
        The list of the (direction, mode) that need to be captured.
    """
    keys_list = goal.have_image.keys()
    goal_list = list()
    for direction, mode in keys_list:
        if (direction, mode) not in state.have_image:
            goal_list.append((direction, mode))
        elif (direction, mode) in state.have_image and not state.have_image[(direction, mode)]:
            goal_list.append((direction, mode))
    return goal_list

################################################################################
# The method functions changePointing and storeImage.

def m_achieveGoal(state, mgoal):
    
    # for satellite in check_pointing(state, mgoal):
    #     return [('changePointing', satellite, mgoal.pointing[satellite], state.pointing[satellite]), ('achieve_goal', mgoal)]
    
    for new_direction, mode in check_image(state, mgoal):
        satellite, instrument = min_fuel(state, new_direction, mode)
        if satellite and instrument:
            return [('calibrateInstrument', satellite, instrument), ('storeImage', satellite, instrument, mode, new_direction), ('achieve_goal', mgoal)]
        else:
            return False
    
    return []

gtpyhop.declare_task_methods('achieve_goal', m_achieveGoal)


################################################################################
# The method functions changePointing and storeImage.

def m_changePointing(state, satellite, new_direction, prev_direction):
    """
    The method that will change the pointing of the satellite if viable.

    Args:
        state: The current state of the problem.
        satellite: The satellite that has to be changed.
        new_direction: The new direction the satellite has to move to.
        prev_direction: The current direction of the satellite.
    """
    if state.pointing[satellite] == new_direction:
        return []
    elif not (state.fuel[satellite] >= state.slew_time[(new_direction, prev_direction)]) or not (state.pointing[satellite] == prev_direction):
        return False
    else:
        return [('turnTo', satellite, new_direction, prev_direction)]
    
gtpyhop.declare_task_methods('changePointing', m_changePointing)

def m_switchOFFInstrument(state, instrument, satellite):
    pass

def m_calibrateInstrument(state, satellite, instrument):
    """
    The method to calibrate the instrument that we need.

    Args:
        state: The state of the system.
        satellite: The satellite that is housing the instrument.
        instrument: The instrument which has to be calibrated.

    Returns:
        The plan the needs to be excecuted to calibrate the instrument.
    """
    plan = list()

    # Switching On the instrument.
    plan.append(('switchOn', instrument, satellite))

    # Calibrate the instrument - change this conditions so that it includes min slew time.
    if state.pointing[satellite] not in state.calibration_target[instrument]:
        plan.append(('changePointing', satellite, state.calibration_target[instrument][0], state.pointing[satellite]))
        plan.append(('calibrate', satellite, instrument, state.calibration_target[instrument][0]))
    else:
        plan.append(('calibrate', satellite, instrument, state.pointing[satellite]))

    return plan

gtpyhop.declare_task_methods('calibrateInstrument', m_calibrateInstrument)

def m_storeImage(state, satellite, instrument, mode, new_direction):
    
    

    plan = list()

    print(f'\n------------\nChecking the directions: {state.pointing[satellite]} and {new_direction}')
    # Change the pointing of the satellite if needed.
    if state.pointing[satellite] is not new_direction:
        plan.append(('changePointing', satellite, new_direction, state.pointing[satellite]))

    # Take the image.
    plan.append(('take_image', satellite, new_direction, instrument, mode))

    # switch OFF the instrument - change this later.
    # plan.append(('switchOff', instrument, satellite))

    return plan

gtpyhop.declare_task_methods('storeImage', m_storeImage)