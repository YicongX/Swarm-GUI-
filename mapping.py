'''
    This module includes all necessary classes and functions for creating and updating maps
'''

import numpy as np

class Map:
    def __init__(self, type=None, xmin=0, xmax=0, ymin=0, ymax=0, value=[], delta_map=[], history = False):
        '''
        INPUTS:
            ymin, ymax, xmin, xmax - minimum and maximum positions of rectangular map region (in meters)
            value - map data, occupancy grid OR landmark positions
            delta_map - uncertainty in the position of landmarks (for landmark maps only), _by 2 array, for the uncertainty in x and y positions
        '''
        if type == 'landmark':
            self.type = 'landmark'
            self.xmin = xmin;
            self.xmax = xmax;
            self.ymin = ymin;
            self.ymax = ymax;
            self.map = value; # landmark positions (2 column array)
            self.delta_map = delta_map; # uncertainty in position of each landmark (1 column array)
            if history:
                self.map_history = value # history of map data
                self.map_history_delta = delta_map # uncertainty in map measurements
            else:
                self.mamp_history = []
        elif type == None:
            self.type = type
            self.xmin = xmin;
            self.ymin = ymin;
            self.xmax = xmax;
            self.ymax = ymax
            self.map = value;
            self.delta_map = delta_map;
            
    def calc_map_hat(self, C_l_hat_new, uncertainty_C_l_hat_new):
        '''
            This function updates the map values and history
            INPUTS: 
                C_l_hat_new - new estimates of map
                uncertainty_C_l_hat_new - uncertainty in the new map estimate
        '''
        n_l = self.map.shape[0]
        
        if len(self.map_history)>0:
            # update map history and uncertainty values:
            self.map_history = np.vstack((self.map_history, C_l_hat_new))
            self.map_history_delta = np.vstack((self.map_history_delta, uncertainty_C_l_hat_new))
            n_h = self.map_history_delta.shape[0]//n_l
            
            # calculate weights for weighted addition of map info:
            weights = np.divide(1,self.map_history_delta)
            sum_weights = np.full((n_l,1), np.nan)
            for i in range(n_l):
                sum_weights[i,:] = np.nansum(weights[i::n_l,:], axis = 0)

            sum_weights = np.tile(sum_weights,(n_h,1))
            weights_normalized = np.divide(weights, sum_weights).reshape((-1,1))
            
            # multiply weights by map
            map_temp = np.multiply(self.map_history, weights_normalized)
            delta_temp = np.multiply(self.map_history_delta, weights_normalized)
            
            # update map values
            for i in range(n_l):
                self.map[i,:] = np.nansum(map_temp[i::n_l,:], axis = 0)
                self.delta_map[i,:] = np.nansum(delta_temp[i::n_l,:], axis = 0)

            id = np.where(self.delta_map == 0)
            self.map[id[0],:] = np.nan
            self.delta_map[id] = np.nan
            #print('done update')
        else:
            weights_old = np.divide(1, self.delta_map)
            weights_new = np.divide(1, uncertainty_C_l_hat_new)
            weights_sum = np.nansum(np.hstack(weights_old, weights_new), axis = 1)
            
            C_l_old_weighted = np.multiply(np.divide(weights_old, weights_sum), self.map)
            C_l_new_weighted = np.multiply(np.divide(weights_new, weights_sum), C_l_hat_new)
            
            uncertainty_old_weighted = np.multiply(np.divide(weights_old, weights_sum), self.delta_map)
            uncertainty_new_weighted = np.multiply(np.divide(weights_new, weights_sum), uncertainty_C_l_hat_new)
            
            self.map = np.nansum(np.dstack(C_l_old_weighted, C_l_new_weighted), axis =2 )
            self.delta_map = np.nansum(np.dstack(uncertainty_old_weighted, uncertainty_new_weighted), axis =2 )
            
            
        

def create_landmark_map_random(xmin, xmax, ymin, ymax, n_l, uncertainty):
    '''
        This function creates a random landmark based map
        INPUTS:
            ymin, ymax, xmin, xmax - minimum and maximum positions of rectangular map region (in meters)
            n_l - number of landmarks to be initialized
            uncertainty - uncertainty in the position of landmarks
        OUTPUTS:
            map - Map class object with necessary info
    '''

    mapx = np.random.uniform(low=xmin, high=xmax, size=(n_l,1));
    mapy = np.random.uniform(low=ymin, high=ymax, size=(n_l,1));
    map_values = np.concatenate((mapx,mapy),axis =1);

    map = Map('landmark', xmin, xmax, ymin, ymax, map_values, uncertainty, False)

    return map



    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    