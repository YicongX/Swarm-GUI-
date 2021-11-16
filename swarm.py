'''
    This module includes all necessary classes and functions for creating and controlling a swarm
'''

import numpy as np

from helper_functions import *


class Swarm:
    def __init__(self,n_r):
        self.n_r = n_r

        self.C_t_init = [] # swarm's initial true configuration
        self.C_t_init_hat = [] # swarm's initial estimated true configuration

        self.C_t = [] # list of all swarm true configurations (not including initial)
        self.C_t_hat = [] # list of all swarm estimated true configurations (not including initial)

        self.C_d = [] # list of all swarm desired configurations

        self.map_hat = [] # list of all estiamted maps

def create_random_configuration(n_r, min_dist):
    '''
        This function randomly initializes the configuration of the swarm
        INPUTS:
            n_r - number of robots in swarm
            min_dist - minimum distance between robots for initialization
        OUTPUTS:
            C_t_init - swarm's initial true configuration
    '''
    C_t_init = np.zeros((1,3))
    for i in range(1,n_r):
        #
        d = np.random.uniform(min_dist, 2*min_dist)
        theta = np.random.uniform(-np.pi, np.pi)
        dx, dy = pol2cart(d,theta)

        n_created = C_t_init.shape[0]
        rand_robot_id = np.random.randint(0,n_created)
        pose_temp = C_t_init[rand_robot_id,:] + np.array([dx, dy, theta])

        C_t_init = np.vstack((C_t_init, pose_temp))

    return C_t_init

def init_random_configuration(n_r, min_dist, map):
    '''
        This function randomly initializes the configuration of the swarm within the boundary of the given map, while also considering obstacles
        INPUTS:
            n_r - number of robots in swarm
            min_dist - minimum distance between robots for initialization
            map - map object, see mapping.py for more details
        OUTPUTS:
            C_t_init - swarm's initial true configuration
    '''

    # randomly initialize:
    C_t_init = create_random_configuration(n_r, min_dist)

    # move swarm within free map region
    if map.type != None:
        min_x = map.xmin;
        min_y = map.ymin
        max_x = map.xmax;
        max_y = map.ymax;

        found = False
        while found is False:
            # randomly translate swarm
            x_temp = np.random.uniform(min_x, max_x)
            y_temp = np.random.uniform(min_y, max_y)
            C_t_init_temp = C_t_init[:,0:2] + np.array([x_temp, y_temp])

            # check if within map bounds
            found = check_within_map(C_t_init_temp[:,0], C_t_init_temp[:,1], map)

        C_t_init[:,0:2] = C_t_init_temp

    return C_t_init



# newly added functions
# This function is used to generate scatter formation, similar idea as init_random_configuration
# boudary = the boudary of swarm formation
def init_random_position(n_r, min_dist, map, boundary=None):
    '''
    boundary: [min_x, min_y, max_x, max_y]
    '''
    if boundary is None:
        boundary = [0, 0, 0.2, 0.2]
    
    C_t_init = np.zeros((1,3))
    for i in range(1, n_r):
        #
        d = np.random.uniform(min_dist, 2*min_dist)
        theta = np.random.uniform(-np.pi, np.pi)
        dx, dy = pol2cart(d,theta)

        n_created = C_t_init.shape[0]
        rand_robot_id = np.random.randint(0,n_created)
        pose_temp = C_t_init[rand_robot_id,:] + np.array([dx, dy, theta])

        C_t_init = np.vstack((C_t_init, pose_temp))
    
    # move swarm within free map region
    if map.type != None:
        min_x, min_y, max_x, max_y = boundary

        found = False
        while found is False:
            # randomly translate swarm
            x_temp = np.random.uniform(min_x, max_x)
            y_temp = np.random.uniform(min_y, max_y)
            C_t_init_temp = C_t_init[:,0:2] + np.array([x_temp, y_temp])

            # check if within map bounds
            found = check_within_map(C_t_init_temp[:,0], C_t_init_temp[:,1], map)

        C_t_init[:,0:2] = C_t_init_temp

    return C_t_init



# This function is used to rotate point B around a fixed point A 
def rotate_point(o, p, alpha):
    """
    rotate p with respect to o by alpha deg
    """
    d = np.sqrt(np.sum((o-p)**2))
    p_r = o + np.array([d*np.cos(alpha), d*np.sin(alpha)])
    return p_r

# This function is used to transform robots into a 0.1 radius circle
# Initial center is (0.1,0.1), centers will be each destination after mission starts
def generate_circle_formations(n_r, radius=0.1, centre=None):
    f_pos = np.zeros((n_r,3))
    if centre is None: centre =[0.1, 0.1]
    angle_interval = np.pi*2 / n_r
    
    for i in range(n_r):
        theta = angle_interval*i
        dx = centre[0] + radius*np.cos(theta)
        dy = centre[1] + radius*np.sin(theta)
        f_pos[i, :] = np.array([dx, dy, theta])
        
    return f_pos

# This function is used to generate polygon formation includes triangle and square.
# The idea is to equally space robots in a straight line and rotate robots around the first referance bot to form desired shape
def generate_polygon_formations(n_r, edges=3, length=0.2):
    side_ = n_r // edges if n_r % edges == 0 else n_r//edges + 1
    interval = length / side_
    pos = []
    # put in a line
    for i in range(n_r):
        pos.append([i*interval, 0])
    pos = np.array(pos)
    # rotate
    for i in range(1,edges):
        for p in range(n_r-i*side_):
            pos[i*side_+p] = rotate_point(pos[i*side_], pos[i*side_+p], i*np.pi/(edges/2))
    
    f_pos = np.concatenate([pos, np.zeros([n_r, 1])], axis=1)
    
    return f_pos           


