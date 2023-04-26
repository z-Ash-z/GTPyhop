"""
Method definitions for satellite_htn
- Aneesh Chodisetty <aneeshch@umd.edu>, March 29th, 2023
"""

import gtpyhop

################################################################################
# Some global variables
first_run = True
nodes_expanded = 0

################################################################################
# The helper functions.

def setFirstRun():
    global first_run, nodes_expanded
    first_run = True
    nodes_expanded = 0


def getNodeCount():
    return nodes_expanded


def getPointingTasks(state, mgoal):
    """
    The method to get all the tasks that are related to pointing in the goal.

    Args:
        state: The state of the system.
        mgoal: The goal state of the system.

    Returns:
        Returns the list of the pointing tasks that are not completed yet.
    """
    task_list = list()

    if 'pointing' in mgoal.__dict__.keys():
        for key in mgoal.pointing.keys():
            if mgoal.pointing[key] != state.pointing[key]:
                task_list.append(key)
        return task_list
    else:
        return None
    

def getHaveImageTasks(state, mgoal):
    """
    The method to get all the tasks that are related to have image in the goal state.

    Args:
        state: The state of the system.
        mgoal: The goal state of the system.

    Returns:
        Returns the list of the have_image tasks that are not completed yet.
    """
    task_list = list()

    if 'have_image' in mgoal.__dict__.keys():
        for key in mgoal.have_image.keys():
            if key in state.have_image.keys():
                if mgoal.have_image[key] != state.have_image[key]:
                    task_list.append(key)
            else:
                task_list.append(key)
        return task_list
    else:
        return None


def getBestCalibTarget(state, instrument, satellite, calib_target):
    """
    Gets the best calibration target for the instrument in the satellite.

    Args:
        state: The state of the system.
        instrument: The instrument that needs to be calibrated.
        satellite: The satellite which houses the instrument.
        calib_target: The calibration target that will be updated.

    Returns:
        The calibration target with the lowest slew_time.
    """
    best_location_cost = float('inf')
    for target in state.calibration_target[instrument]:
        if target == state.pointing[satellite]:
            return target
        if state.slew_time[(state.pointing[satellite], target)] < best_location_cost:
            best_location_cost = state.slew_time[(state.pointing[satellite], target)]
            calib_target = target
    return calib_target


def select_best_satellite(state, goal, new_direction, mode, pointing_tasks):
    """
    The best satellite needed for the pointing task.

    Args:
        state: The state of the system.
        goal: The goal state.
        new_direction: The direction which the satellite needs to face.
        mode: The mode in which we need an image.
        pointing_tasks: The number of pointing tasks, if any.

    Returns:
        A dictionary containing the instrument, satellite and calibration_target that needs to be part of the plan
    """
    slew_times = {}
    slew_times[float('inf')] = {'instrument' : None,
                                'satellite' : None,
                                'calibration_target' : None}
    
    # Getting all the supported instruments
    supported_instruments = [instrument for instrument, modes in state.supports.items() if mode in modes]

    for instrument in supported_instruments:
        cost = 0
        calib_target = None
        need_calibration = False

        satellite = state.on_board[instrument]
        
        # If instrument is already calibrated, no need for additional slew_time.
        if 'calibrated' in state.__dict__.keys():
            if instrument in state.calibrated.keys():
                if not state.calibrated[instrument]:
                    need_calibration = True
            else:
                need_calibration = True
        else:
            need_calibration = True

        if need_calibration:
            calib_target = getBestCalibTarget(state, instrument, satellite, calib_target)
            if calib_target != state.pointing[satellite]:
                cost += state.slew_time[(state.pointing[satellite], calib_target)]

        
        if calib_target:
            cost += state.slew_time[(new_direction, calib_target)]
        else:
            cost += state.slew_time[(state.pointing[satellite], new_direction)]

        # If there are pointing tasks check if we have enough fuel to complete them.
        if pointing_tasks:
            if satellite in pointing_tasks and goal.pointing[satellite] != new_direction:
                cost += state.slew_time[(goal.pointing[satellite], new_direction)]

        # Reject satellite if we do not have enough fuel for all the tasks.
        if state.fuel[satellite] > cost:
            slew_times[cost] = {'instrument' : instrument,
                                'satellite' : satellite,
                                'calibration_target' : calib_target}
    
    return slew_times[min(sorted(slew_times))]


def getImageStatus(state, mgoal, task, pointing_tasks):
    """
    Method to compute the status of the image task.

    Args:
        state: The state of the system.
        mgoal: The goal state.
        task: The current task.
        pointing_tasks: The list of pointing tasks.

    Returns:
        A dictionary containing the status, instrument, satellite and calibration_target that needs to be part of the plan
    """
    new_direction, mode = task

    status = {}
    # Checking if task is already done
    if 'have_image' in state.__dict__.keys():
        if task in state.have_image.keys():
            if state.have_image[task]:
                status['status'] = 'Done'
                return status

    # Selecting the best satellite
    status = select_best_satellite(state, mgoal, new_direction, mode, pointing_tasks)

    # Failure case if no instruments can take the image.    
    if status['instrument'] == None:
        status['status'] = 'Fail'

    # Instrument already calibrated hence just TakeImage.
    elif status['calibration_target'] == None:
        status['status'] = 'TakeImage'

    else:
        power_avail = False
        
        if 'power_avail' in state.__dict__.keys():
            if status['satellite'] in state.power_avail.keys():
                if state.power_avail[status['satellite']]:
                    power_avail = True
            else:
                power_avail = True
        else:
            power_avail = True
        
        # Power is available for calibration so Calibrate and TakeImage.
        if power_avail:
            status['status'] = 'Calib-and-TakeImage'

        # No power is available hence switchOFF the instrument first.
        else:
            instrument = None
            for i, s in state.on_board.items():
                if s != status['satellite']:
                    continue
                else:
                    if i in state.calibrated.keys():
                        if state.calibrated[i]:
                            instrument = i
            status['status'] = 'Switch-Off'
            status['instrument'] = instrument

    return status


def getPointingStatus(state, mgoal, task):
    """
    Method to compute the status of pointing tasks.

    Args:
        state: The state of the system.
        mgoal: The goal state.
        task: The pointing task.

    Returns:
        Status of the task.
    """
    status = 'Change-Pointing'
    
    if state.pointing[task] == mgoal.pointing[task]:
        status = 'Done'
    
    return status


################################################################################
# The Methods to create the plan.

def m_achieveGoal(state, mgoal):
    """
    The main planner that creates the plan based on the task that needs to be performed.

    Args:
        state: The state of the system.
        mgoal: The goal state the system has to reach.

    Returns:
        Returns the plan if possible.
    """

    # Initializing attributes that might not be given in the init state.
    global first_run
    if first_run:
        first_run = False
        state.power_on ={}
        state.calibrated = {}
        state.have_image = {}

    # Get the tasks.
    pointing_tasks = getPointingTasks(state, mgoal)
    image_tasks = getHaveImageTasks(state, mgoal)

    # Planning the image tasks.
    if image_tasks:
        for task in image_tasks:
            status = getImageStatus(state, mgoal, task, pointing_tasks)
            new_direction, mode = task

            if status['status'] == 'Fail':
                return False
            
            elif status['status'] == 'TakeImage':
                # StoreImage -> achieve
                return [('storeImage', status['satellite'], status['instrument'], mode, new_direction), ('achieve_goal', mgoal)]
            
            elif status['status'] == 'Calib-and-TakeImage':
                # CalibrateInstrument -> StoreImage -> achieve
                return [('calibrateInstrument', status['instrument'], status['satellite']), ('storeImage', status['satellite'], status['instrument'], mode, new_direction), ('achieve_goal', mgoal)]

            elif status['status'] == 'Switch-Off':
                # Instrument-Off -> achieve
                return [('instrumentOff', status['instrument'], status['satellite']), ('achieve_goal', mgoal)]
            
            else:
                continue

    # Planning the pointing tasks.
    if pointing_tasks:
        for task in pointing_tasks:
            status = getPointingStatus(state, mgoal, task)
            
            if status == 'Change-Pointing':
                # ChangePointing -> achieve
                return [('changePointing', task, mgoal.pointing[task], state.pointing[task]), ('achieve_goal', mgoal)]
            else:
                continue

    return []

gtpyhop.declare_task_methods('achieve_goal', m_achieveGoal)


################################################################################
# The Methods for the tasks.

def m_changePointing(state, satellite, new_direction, prev_direction):
    """
    The method that will change the pointing of the satellite if viable.

    Args:
        state: The current state of the problem.
        satellite: The satellite that has to be changed.
        new_direction: The new direction the satellite has to move to.
        prev_direction: The current direction of the satellite.
    """
    global nodes_expanded
    nodes_expanded += 1

    if state.pointing[satellite] == new_direction:
        return []
    elif not (state.fuel[satellite] >= state.slew_time[(new_direction, prev_direction)]) or not (state.pointing[satellite] == prev_direction):
        return False
    else:
        nodes_expanded += 1
        return [('turnTo', satellite, new_direction, prev_direction)]
    
gtpyhop.declare_task_methods('changePointing', m_changePointing)


def m_calibrateInstrument(state, instrument, satellite):
    """
    The method to calibrate the instrument that we need.

    Args:
        state: The state of the system.
        instrument: The instrument which has to be calibrated.
        satellite: The satellite that is housing the instrument.

    Returns:
        The plan the needs to be excecuted to calibrate the instrument.
    """
    global nodes_expanded
    nodes_expanded += 1

    plan = list()

    # Safety check before Switching On the instrument.
    if state.on_board[instrument] != satellite or not state.power_avail[satellite]:
        return False
    
    plan.append(('switchOn', instrument, satellite))
    nodes_expanded += 1

    # Calibrating the instrument. 
    if state.pointing[satellite] not in state.calibration_target[instrument]:
        plan.append(('changePointing', satellite, state.calibration_target[instrument][0], state.pointing[satellite]))
        plan.append(('calibrate', satellite, instrument, state.calibration_target[instrument][0]))
        nodes_expanded += 2
    else:
        plan.append(('calibrate', satellite, instrument, state.pointing[satellite]))
        nodes_expanded += 1

    return plan

gtpyhop.declare_task_methods('calibrateInstrument', m_calibrateInstrument)


def m_storeImage(state, satellite, instrument, mode, new_direction):
    """
    The method to store the Image of the requested location using the given mode.

    Args:
        state: The state of the system.
        satellite: The satellite used for taking the image.
        instrument: The instrument that will take the image.
        mode: The mode of the instrument chosen.
        new_direction: The direction in which we need the image.

    Returns:
        The plan that needs to be executed for storing the image.
    """
    global nodes_expanded
    nodes_expanded += 1

    plan = list()

    # Change the pointing of the satellite if needed.
    if state.pointing[satellite] is not new_direction:
        plan.append(('changePointing', satellite, new_direction, state.pointing[satellite]))
        nodes_expanded += 1

    # Sanity check.
    if state.on_board[instrument] != satellite or (mode not in state.supports[instrument]) or (state.power_on[instrument] != True) or (state.data_capacity[satellite] < state.data[(new_direction, mode)]):
        return False
    
    # Take the image.
    plan.append(('take_image', satellite, new_direction, instrument, mode))
    nodes_expanded += 1

    return plan

gtpyhop.declare_task_methods('storeImage', m_storeImage)


def m_instrumentOff(state, instrument, satellite):
    """
    The method to switch off an instrument.

    Args:
        state: The state of the system.
        instrument: The instrument that will be turned OFF.
        satellite: The satellite that houses the instrument.

    Returns:
        The plan that needs to be executed for switching off the instrument.
    """
    global nodes_expanded
    nodes_expanded += 1

    if not state.power_on[instrument] and state.power_avail[satellite]:
        return []
    elif state.on_board[instrument] != satellite or not state.power_on[instrument]:
        return False
    else:
        nodes_expanded += 1
        return [('switchOff', instrument, satellite)]
    
gtpyhop.declare_task_methods('instrumentOff', m_instrumentOff)