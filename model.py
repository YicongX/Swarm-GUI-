import numpy as np

from mapping import create_landmark_map_random, Map
from swarm import (init_random_configuration, init_random_position,
                   generate_circle_formations, generate_polygon_formations)

from motion_planning import calc_motion_commands
from motion_inputs import teleport_2_commands
from scipy.spatial.distance import pdist, squareform

class Model:
    def __init__(self):
        # create swarm
        self.n_r = 10
        self.min_init_distance = 0.05
        self.dis_error_thr = 0.01
        self.robot_p_history = []
        self.boundary = [0, 1, 0, 1]

    def init_map(self):
        # create random map
        map_seed = 1
        np.random.seed(map_seed)
        n_l = 200
        xmin, xmax, ymin, ymax = self.boundary
        self.map_t = create_landmark_map_random(xmin, xmax, ymin, ymax, n_l,
                                                np.zeros((n_l, 1)))

    def init_initial_positions(self):
        # move swarm to random configuration in the environment:
        init_config_seed = 2
        np.random.seed(init_config_seed)
        xmin, xmax, ymin, ymax = self.boundary
        self.C_t = init_random_position(self.n_r, self.min_init_distance,
                                        self.map_t)
        self.robot_p_history.append(self.C_t)

        # estimate swarm's initial configuration and map:
        self.C_t_hat = self.C_t.copy()
        self.map_hat = Map('landmark', xmin, xmax, ymin, ymax, self.map_t.map,
                           self.map_t.delta_map, False)

    def init_formations(self, desired_formation='random', center=None):
        if desired_formation == 'random':
            self.C_d = init_random_position(self.n_r, self.min_init_distance,
                                            self.map_t)
        elif desired_formation == 'circle':
            self.C_d = generate_circle_formations(self.n_r)
        elif desired_formation == 'square':
            self.C_d = generate_polygon_formations(self.n_r, edges=4)
        elif desired_formation == 'triangle':
            self.C_d = generate_polygon_formations(self.n_r, edges=3)
        if center is not None:
            self.C_d[:,0:2] += (center - 0.1)
            
    def init_swarm(self, desired_formation='random'):
        self.init_map()
        self.init_initial_positions()
        self.init_formations(desired_formation)
        print('## swarm initialized~')

    def calculate_dis_error(self):
        return np.sum(np.sqrt((self.C_t_hat[:, 0:2] - self.C_d[:, 0:2])**2))

    def calculate_ave_position(self):
        pos = np.mean(self.C_t_hat[:, 0:2], axis=0)
        return pos

    def calculate_speed(self):
        if len(self.robot_p_history) >= 2:
            d = self.robot_p_history[-1][:, 0:2] - self.robot_p_history[-2][:, 0:2]
            ave_speed = np.mean(np.abs(d), axis=0)
            return np.sqrt(ave_speed[0]**2 + ave_speed[1]**2)
        else:
            return 0.0

    def calculate_longest_robot_dis(self):
        points = self.C_t_hat[:, 0:2]
        return np.max(squareform(pdist(points)))
    
    def run_mission(self, motion_noise=0.0):
        # calculate motion commands for swarm
        travel_dist_desired, turn_angle_desired = calc_motion_commands(
            self.C_t_hat, self.C_d)
        # move swarm to motion commands:
        self.C_t = teleport_2_commands(self.C_t, travel_dist_desired * 0.1,
                                       turn_angle_desired, motion_noise)

        # localize:
        self.C_t_hat = self.C_t.copy()

        # save
        self.robot_p_history.append(self.C_t)
