"""
Method definitions for satellite_htn
- Aneesh Chodisetty <aneeshch@umd.edu>, March 29th, 2023
"""

import gtpyhop

################################################################################
# The helper functions.

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
        if 'pointing' in state.__dict__.keys():
            for key in mgoal.pointing.keys():
                if mgoal.pointing[key] != state.pointing[key]:
                    task_list.append(key)
            return task_list
        else:
            return mgoal.pointing.keys()
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
        if 'have_image' in state.__dict__.keys():
            for key in mgoal.have_image.keys():
                if mgoal.have_image[key] != state.have_image[key]:
                    task_list.append(key)
        else:
            return mgoal.have_image.keys()
    else:
        return None


def getBestCalibTarget(state, instrument, satellite, calib_target):
    best_location_cost = float('inf')
    for target in state.calibration_target[instrument]:
        if state.slew_time[(state.pointing[satellite], target)] < best_location_cost:
            best_location_cost = state.slew_time[(state.pointing[satellite], target)]
            calib_target = target
    return calib_target


def select_best_satellite(state, goal, new_direction, mode, pointing_tasks):

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
            cost += state.slew_time[(state.pointing[satellite], calib_target)]

        
        if calib_target:
            cost += state.slew_time[(new_direction, calib_target)]
        else:
            cost += state.slew_time[(state.pointing[satellite], new_direction)]

        if satellite in pointing_tasks:
            cost += state.slew_time[(goal.pointing[satellite], new_direction)]

        if state.fuel[satellite] > cost:
            slew_times[cost] = {'instrument' : instrument,
                                'satellite' : satellite,
                                'calibration_target' : calib_target}
    
    return slew_times[min(sorted(slew_times))]


def getImageStatus(state, mgoal, task, pointing_tasks):
    
    new_direction, mode = task

    status = select_best_satellite(state, mgoal, new_direction, mode, pointing_tasks)

    if 'have_image' in state.__dict__.keys():
        if task in state.have_image.keys():
            if state.have_image[task]:
                status['status'] = 'Done'
                return status
    
    if status['instrument'] == None:
        status['status'] = 'Fail'

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
        
        if power_avail:
            status['status'] = 'Calib-and-TakeImage'
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

    pointing_tasks = getPointingTasks(state, mgoal)
    image_tasks = getHaveImageTasks(state, mgoal)

    if image_tasks:
        for task in image_tasks:
            status = getImageStatus(state, mgoal, task, pointing_tasks)

            if status['status'] == 'Fail':
                return False
            elif status['status'] == 'TakeImage':
                pass # Replace with storeImage -> achieve
            elif status['status'] == 'Calib-and-TakeImage':
                pass # Replace with calibrate -> storeImage -> achieve
            elif status['status'] == 'Switch-Off':
                pass # Replace with Switch-Off -> achieve
            else:
                continue

    if pointing_tasks:
        for task in pointing_tasks:
            # status =
            pass

    return []

gtpyhop.declare_task_methods('achieve_goal', m_achieveGoal)


################################################################################
# The Methods for the tasks.