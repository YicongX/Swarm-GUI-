'''
    This module includes all necessary classes and functions for providing motion commands to stage_ros
'''

import numpy as np
import time

from helper_functions import *

def teleport_2_commands(C_c, travel_dist_desired, turn_angle_desired, motion_noise):
    '''
        This function adds noise to the motion commands and teleports robots to their respective positions in the Stage simulator
        INPUTS:
            C_c - swarm's current true configuration
            travel_dist_desired - desired travel distance for all robots (m), n_r by 1 matrix
            turn_angle_desired - desired turn angle for all robots (rad), n_r by 1 matrix
            motion_noise - noise in motion execution
        OUTPUTS:
            C_achieved - the true achieved configuration of the swarm
    '''
    n_r = C_c.shape[0]

    # add noise
    travel_dist_actual = travel_dist_desired + np.random.normal(0,travel_dist_desired*motion_noise/3)
    turn_angle_actual = turn_angle_desired + np.random.normal(0, np.absolute(turn_angle_desired)*motion_noise/3)

    # calculate true achieved configuration
    actual_orientations = C_c[:,2].reshape((n_r,1)) + turn_angle_actual
    dx, dy = pol2cart(travel_dist_actual,actual_orientations)
    dx = dx + C_c[:,0].reshape((n_r,1))
    dy = dy + C_c[:,1].reshape((n_r,1))

    C_achieved = np.hstack((dx, dy, actual_orientations))

    return C_achieved
