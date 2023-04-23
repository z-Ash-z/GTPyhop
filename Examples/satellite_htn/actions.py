"""
Acttion defenitions for the satellite domain.
-- Aneesh Chodisetty <aneeshch@umd.edu>, March 29th, 2023
"""

import gtpyhop

"""
The satellite domain actions use the following state variables:
- 
"""

def turnTo(s, sat, d_new, d_prev):
    """
    The task of turning to the mentioned direction.

    Args:
        s: The current state of the system.
        sat: The satellite whose direction needs to changed.
        d_new: The new direction of the satellite.
        d_prev: The previous/current domain.

    Returns:
        The updated state.
    """
    if (s.pointing[sat] == d_prev) and (d_new != d_prev) and (s.fuel[sat] >= s.slew_time[(d_new, d_prev)]):
        s.pointing[sat] = d_new
        s.fuel[sat] -= s.slew_time[(d_new, d_prev)]
        s.fuel_used += s.slew_time[(d_new, d_prev)]
        return s


def switchOn(s, i, sat):
    """
    The task of switching ON the instrument in the satellite.

    Args:
        s: The state of the system.
        i: The instrument that should be turned ON.
        sat: The satellite in which the instrument is present.

    Returns:
        The updated state.
    """
    if (s.on_board[i] == sat) and (s.power_avail[sat] == True):
        s.power_on[i] = True
        s.calibrated[i] = False
        s.power_avail[sat] = False
        return s


def switchOff(s, i, sat):
    """
    The task of switching OFF the instrument in the satellite.

    Args:
        s: The state of the system.
        i: The instrument that should be turned OFF.
        sat: The satellite in whihc the instrument is present.

    Returns:
        The updated state.
    """
    if (s.on_board[i] == sat) and (s.power_on[i] == True):
        s.power_on[i] = False
        s.power_avail[sat] = True
        return s
    
def calibrate(s, sat, i, d):
    """
    The task to calibrate the instrument in the satellite.

    Args:
        s: The state of the system.
        sat: The satellite in which the instrument has to be calibrated.
        i: The instrument to be calibrated.
        d: The direction to which the instrument should be calibrated

    Returns:
        The updated state.
    """
    if (s.on_board[i] == sat) and (d in s.calibration_target[i]) and (s.pointing[sat] == d) and (s.power_on[i] == True):
        s.calibrated[i] = True
        return s

def take_image(s, sat, d, i, m):
    """
    The task to take the image of the destination with the mentioned mode via the instrument in the satellite.

    Args:
        s: The state of the system.
        sat: The satellite that houses the instrument.
        d: The direction in which we need to take the image.
        i: The instrument via which the image is captured.
        m: The mode in which we need to capture the image.

    Returns:
        The updated state of the image.
    """
    if (s.calibrated[i] == True) and (s.on_board[i] == sat) and (m in s.supports[i]) and (s.power_on[i] == True) and (s.pointing[sat] == d) and (s.data_capacity[sat] >= s.data[(d, m)]):
        s.data_capacity[sat] -= s.data[(d, m)]
        s.have_image[(d, m)] = True
        s.data_stored += s.data[(d, m)]
        return s

# Telling Pyhop the actions for this domain
gtpyhop.declare_actions(turnTo, switchOn, switchOff, calibrate, take_image)