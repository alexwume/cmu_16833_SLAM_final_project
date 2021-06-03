#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid

from visualization_msgs.msg import Marker, MarkerArray
import matplotlib
from matplotlib import pyplot
from pyquaternion import Quaternion
from gazebo_msgs.msg import ModelStates 


class SD_algo(object):
    def __init__(self, height, width, resolution):
        self.height = height
        self.width = width
        self.static_map = -1  * np.ones((height, width))
        self.dynamic_map = -1 * np.ones((height, width))
        self.counter = 0
        self.resolution = resolution
        print("initialized static/ dynamic map")
        # print("init static map")
        # print(self.static_map)

    def update_sd_map(self, laser_measure):
        '''
        \param: laser_measure is a H x W array

        '''
        eps = 0.001
        #static map masks
        free_mask = (self.static_map >= 0) & (self.static_map < 80) # H x W 
        nonzero_mask = self.static_map > 0
        unk_mask = self.static_map == -1 # H x W
        occ_mask = self.static_map >= 80 # H x W

        #dynamic map masks
        free_mask_d = (self.dynamic_map >= 0) & (self.dynamic_map < 80) # H x W 
        nonzero_mask_d = self.dynamic_map > 0
        unk_mask_d = self.dynamic_map == -1 # H x W
        occ_mask_d = self.dynamic_map >= 80 # H x W


        print("laser map")
        print(laser_measure[0, : ])

        print("static map")
        print(self.static_map[0,:])

        print("dynamic map")
        print(self.dynamic_map[0,:])

        #laser masks
        laser_free_mask = laser_measure == 0  # H x W
        laser_occ_mask = laser_measure == 1 # H x W
        laser_unk_mask = laser_measure == -1 # H x W


 
        #dynamic map

        # deal with the numerical issue when unknown cell is intialized to -1
        reformed_dynamic_map = self.dynamic_map * nonzero_mask_d + eps *np.ones_like(self.dynamic_map) * ~nonzero_mask_d

        tmp = np.log(reformed_dynamic_map / (101 - reformed_dynamic_map))
        print("tmp")
        print(tmp[0,3:6])

        #free | free
        tmp1 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(0.9 / 0.1) + tmp)
        map_fo = tmp4 /  (1 + tmp4)      
        mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(0.3 / 0.7))
        map_uo = tmp5 /  (1 + tmp5)
        mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(0.3 / 0.7) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        mask_oo = occ_mask & laser_occ_mask

        mask_not = ~(mask_ff | mask_uf | mask_of | mask_fo | mask_uo | mask_oo)
        # print(self.dynamic_map)

        self.dynamic_map = self.dynamic_map * mask_not + 100 * (map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo) 

        unk_mask_d = self.dynamic_map == -1
        self.dynamic_map = np.clip(self.dynamic_map, 10, 100) * ~unk_mask_d + self.dynamic_map * unk_mask_d         
        #static map
        
        # deal with the numerical issue when unknown cell is intialized to -1
        reformed_static_map = self.static_map * nonzero_mask + eps *np.ones_like(self.static_map) * ~nonzero_mask

        tmp = np.log(reformed_static_map / (101 - reformed_static_map))

        #free | free
        tmp1 = np.exp(np.log(0.1 / 0.8) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        # mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(0.1 / 0.8) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        # mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(0.1 / 0.9) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        # mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(0.1 / 0.9) + tmp)
        map_fo = tmp4 /  (1 + tmp4)
        # mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(0.9 / 0.1))
        map_uo = tmp5 /  (1 + tmp5)
        # mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(0.9 / 0.1) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        # mask_oo = occ_mask & laser_occ_mask


        self.static_map = self.static_map * mask_not + 100 * (map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo) 
        
        unk_mask = self.static_map == -1
        self.static_map = np.clip(self.static_map, 10, 100) * ~unk_mask + self.static_map * unk_mask 

        print("dynamic map")
        print(self.dynamic_map[0,:])
        print("===========================================")


if __name__ == "__main__":
    sd_test = SD_algo(10, 10, 0.05)
    laser_map = -1 * np.ones((10, 10))
    laser_map[0,:] = [0,0,0,1,1,1,-1,-1,-1,0] 
    
    for i in range(10):
        print(i)
        sd_test.update_sd_map(laser_map)