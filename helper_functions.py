'''
    This module includes some useful helper functions
'''

import numpy as np
import itertools

def cart2pol(x,y):
    '''
        This function converts from cartesian coordinations to polar coordinations
        INPUT: cartesian coordinates
            x - n by m matrix of all x coordinates
            y - n by m matrix of all y coordinates
        OUTPUT: polar coordinates
            d - n by m matrix of all distance values
            theta - n by m matrix of all angle values, ranging from (-pi, pi)
    '''
    d = np.sqrt(np.square(x) + np.square(y))
    theta = np.arctan2(y,x)

    return d, theta

def pol2cart(d,theta):
    '''
        This function converts from polar coordinations to cartesian coordinations
        INPUT: polar coordinates
            d - n by m matrix of all distance values
            theta - n by m matrix of all angle values, ranging from (-pi, pi)
        OUTPUT: cartesian coordinates
            x - n by m matrix of all x coordinates
            y - n by m matrix of all y coordinates
    '''

    x = np.multiply(d, np.cos(theta))
    y = np.multiply(d, np.sin(theta))

    return x,y

def rotate(position_L, theta_LG):
    '''
        This is the rotation function in 2D space
        INPUTS:
            position_L - _ by 2 np array of positions [x, y], defined with respect to frame L
            theta_LG - rotation angle, defining the angle from frame G to frame L
        OUTPUTS:
            position_G - rotated positions with respect to frame G, _ by 2 np array
    '''
    c, s = np.cos(theta_LG), np.sin(theta_LG)
    R_LG = np.array(((c, -s), (s, c)))

    position_G = np.dot(R_LG,position_L.T)

    return position_G.T

def xy2rc(x, y, resolution):
    '''
        This function converts the xy coordinates of a point in space to row column of a numpy array
        it is assumed that the origin of the xy coordinates correspond with the [0,0] of the numpy array
        INPUTS:
            x,y - coordinates of the point in space (m)
            resolution - resolution of numpy array (m/row or m/column)
        OUTPUTS:
            r, c - the related row and column of the input position
    '''
    c = np.floor(x/resolution).astype(int)
    r = np.floor(y/resolution).astype(int)

    return r,c

def check_within_map(x, y, map):
    '''
        This function checks whether the positions, x and y, are within the obstacle free map region
        INPUTS:
            x,y - position of point, _ by 1 np array
            map - map object, see mapping.py for more details
        OUPUTS:
            bool - True if within region, false if not
    '''
    result = False

    # checking for map boundaries
    min_x = map.xmin;
    min_y = map.ymin
    max_x = map.xmax;
    max_y = map.ymax;

    # check if within map boundary
    if (np.max(x) < max_x) and (np.max(y) < max_y) and (np.min(x) > min_x) and (np.min(y) > min_y):
        if map.type == 'occupancy':
            # checking for obstacles within boundary
            r,c = xy2rc(x,y,map_res)
            ind = (r,c)
            obstacles_values = map[(ind)]
            if (np.sum(obstacles_values) == 0):
                result = True
        elif map.type == 'landmark':
            result = True;

    return result

def two_pi_to_pi(angles):
    '''
        This function converts the angles ranging from 0 -> 2PI to ranging from -PI -> PI
        INPUTS:
            angles - array of angles (rad)
        OUTPUTS:
            angles_mod - modified angles (rad)
    '''
    angles_mod = np.mod(angles+np.pi,2*np.pi) - np.pi

    return angles_mod

def pi_to_two_pi(angles):
    '''
        This function converts the angles ranging from -PI -> PI to ranging from 0 -> 2PI
        INPUTS:
            angles - array of angles (rad), ranging from -pi to pi
        OUTPUTS:
            angles_mod - modified angles (rad), randing from 0 to 2pi
    '''
    angles_mod = np.mod(angles,2*np.pi)

    return angles_mod

def min_turn(current_orientation, desired_orientation):
    '''
        This function calculates the minimum turn angle
        INPUTS:
            current_orientation - current orientation of the robots, _ by 1 matrix
            desired_orientation - desired orientation of the robots, _ by 1 matrix
        OUTPUTS:
            min_turn_angle - minimum turn angle for the robots, _ by 1 matrix
    '''

    turn_angle = desired_orientation - current_orientation
    min_turn_angle = np.mod(turn_angle+np.pi, 2*np.pi) - np.pi

    return min_turn_angle
