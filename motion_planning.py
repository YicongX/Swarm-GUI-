'''
    This module includes all necessary classes and functions for swarm motion planning
'''

import numpy as np
from scipy.optimize import minimize

from helper_functions import *

def calc_motion_commands(C_c, C_d):
    '''
        This function calculates the motion commands that take the swarm from its current to desired configurations. Motion commands are for a robot that reaches its destination through a 2 step FSM - turning to the desired direction, and travelling along a straight path
        INPUTS:
            C_c - swarm's current configuration
            C_d - swarm's desired configuration
        OUTPUTS:
            turn_angle_desired - desired turn angle of all robots (rad), n_r by 1 matrix
            travel_dist_desired - desired travel distance along the straight path for all robots (m), n_r by 1 matrix
    '''
    move_desired = C_d - C_c
    n_r = move_desired.shape[0]
    travel_dist_desired, heading_desired = cart2pol(move_desired[:,0].reshape((n_r,1)), move_desired[:,1].reshape((n_r,1)))
    turn_angle_desired = min_turn(C_c[:,2].reshape((n_r,1)),heading_desired)

    return travel_dist_desired, turn_angle_desired


class Motion_Planner:
    def __init__(self, d_wiggle = 0.01, d_step_min = 0, d_step_max = 0.15, theta_step_min = -np.pi, theta_step_max = np.pi):
        '''
            This is the motion planner class
                INPUTS:
                d_wiggle - wiggle distance (m)
                d_step_min, d_step_max - step size range (m)
                theta_step_min, theta_step_max - step direction range (rad)
        '''
        self.d_wiggle = d_wiggle
        self.d_step_min = d_step_min
        self.d_step_max = d_step_max
        self.theta_step_min = theta_step_min
        self.theta_step_max = theta_step_max

        self.theta_last = np.nan

    def create_random_C_d(self, C_c):
        '''
            This function randomly selects a desired configuration that translates in a random direction with respect to the swarm's current configuration
            INPUTS:
                C_c - swarm's current configuration
            OUTPUTS:
                C_d - swarm's desired configuration
        '''
        n_r = C_c.shape[0]
        C_d = np.zeros((n_r,3))

        # wiggle swarm topology
        for i in range(n_r):
            d_temp = np.random.uniform(0,self.d_wiggle)
            theta_temp = np.random.uniform(-np.pi,np.pi)
            x_wiggle, y_wiggle = pol2cart(d_temp, theta_temp)
            C_d[i,0:2] = C_c[i,0:2] + np.array([x_wiggle, y_wiggle])

        # take step in random directions
        d_step = np.random.uniform(self.d_step_min, self.d_step_max)
        theta_step = np.random.uniform(self.theta_step_min, self.theta_step_max)
        x_step, y_step = pol2cart(d_step, theta_step)
        C_d[:,0:2] = C_d[:,0:2] + np.array([x_step, y_step])

        return C_d

    def set_random_C_d(self, C_c, map):
        '''
            This function randomly selects a desired configuration that translates and wiggles, while remaining within the obstacle free and boundary region of the map
            INPUTS:
                C_c - swarm's current configuration
                map - map class object, see mapping.py for details
            OUTPUTS:
                C_d - swarm's desired configuration
        '''
        found = False
        counter = 0
        # remember old parameters just in case:
        d_wiggle_old, d_step_min_old, d_step_max_old, theta_step_min_old, theta_step_max_old = self.d_wiggle, self.d_step_min, self.d_step_max, self.theta_step_min, self.theta_step_max

        while found is False:
            # set random destination
            #print('setting C_d')
            C_d_temp = self.create_random_C_d(C_c)
            counter +=1
            # check if destination is within obstacle free map
            if map.type == None:
                found = True
            else:
                found = check_within_map(C_d_temp[:,0], C_d_temp[:,1], map)

            # try new approach if it can't find feasible destination
            if counter >= 10:
                mean_swarm_pose = np.mean(C_c[:,:2], axis = 0)
                mean_map_pose = np.mean(map.map, axis = 0)
                angle = np.arctan2(mean_map_pose[1]-mean_swarm_pose[1], mean_map_pose[0]-mean_swarm_pose[0])
                self.theta_step_min = angle
                self.theta_step_max = angle
                self.d_wiggle = 0
                self.d_step_min = 0.01*counter
                self.d_step_max = 0.01*counter

        C_d = C_d_temp

        # reset old parameters
        self.d_wiggle, self.d_step_min, self.d_step_max, self.theta_step_min, self.theta_step_max = d_wiggle_old, d_step_min_old, d_step_max_old, theta_step_min_old, theta_step_max_old

        return C_d

    def set_bouncy_C_d(self, C_c, map):
        '''
            This function sets a desired configuration that bounces off the boundaries of the swarm's configuration
            INPUTS:
                C_c - swarm's current configuration
                map - map class object, see mapping.py for details
            OUTPUTS:
                C_d - swarm's desired configuration
        '''
        found = False
        counter = 0

        # make sure we are going in a given direction only:
        self.theta_step_min = self.theta_step_max
        
        # remember old parameters:
        d_wiggle_old, d_step_min_old, d_step_max_old, theta_step_min_old, theta_step_max_old = self.d_wiggle, self.d_step_min, self.d_step_max, self.theta_step_min, self.theta_step_max

        # look for feasible desired configuration
        while found == False:
            counter += 1
            # set random desired configuration in previous direction:
            C_d_temp = self.create_random_C_d(C_c)

            # check if destination is within obstacle free map
            if map.type == None:
                found = True
            else:
                found = check_within_map(C_d_temp[:,0], C_d_temp[:,1], map)

            if found == False:
                # change direction of swarm:
                self.theta_step_min += np.pi/2 + np.random.uniform(-10*np.pi/180, 10*np.pi/180)
                self.theta_step_max = self.theta_step_min

            if counter >= 10:
                mean_swarm_pose = np.mean(C_c[:,:2], axis = 0)
                mean_map_pose = np.mean(map.map, axis = 0)
                angle = np.arctan2(mean_map_pose[1]-mean_swarm_pose[1], mean_map_pose[0]-mean_swarm_pose[0])
                self.theta_step_min = angle
                self.theta_step_max = angle
                self.d_wiggle = 0
                self.d_step_min = 0.01*counter
                self.d_step_max = 0.01*counter

        # reset old parameters
        self.d_wiggle, self.d_step_min, self.d_step_max = d_wiggle_old, d_step_min_old, d_step_max_old

        C_d = C_d_temp 

        return C_d