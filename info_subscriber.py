#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan PointCloud2, PointField
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from matplotlib import pyplot
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib as mpl
from pyquaternion import Quaternion
from gazebo_msgs.msg import ModelStates 

from __future__ import division
'''
author: Alex Wu
email: chichiaw@andrew.cmu.edu
purpose: cmu-16883 final project
'''

class map_reconstructor(object):
    def __init__(self, robot_pose, laser_data, curr_map):
        '''
        \param: robot_pose: robot poses from gazebo simulation. [xyz(translation), xyzw(orientation quaternion)]
        \param: laser_reading: 360 laser readings
        '''
        self.robot_pose = robot_pose #[x,y,z, qx,qy,qz,qw] (7,)
        self.laser_data = laser_data #360 laser readings (360, )
        self.interval = 10
        self.map = curr_map
        self.laser_map = -1 * np.ones((384, 384))


    def visualize_laser(self):
        '''
        visualize the laser reading (360 x 3)
        '''
        # print(self.robot_pose)
        # print(len(self.laser_data))

        # require pyquaternion

        # read euler data for quaternion
        robot_qw = self.robot_pose[6]
        robot_qaxis = self.robot_pose[3:6]
        robot_q = np.vstack((robot_qw, robot_qaxis))

        # create quaternion
        curr_quater = Quaternion(robot_q)
        # retrieve theta from quaternion
        theta = curr_quater.angle

        # calculate laser readning
        robot_x = self.robot_pose[0]
        robot_y = self.robot_pose[1]
        robot_z = self.robot_pose[2]    
        laser_x = robot_x + laser_data * np.cos(theta)
        laser_y = robot_y + laser_data * np.sin(theta)
        laser_z = robot_z * np.ones(360)
        laser_reading = np.vstack((laser_x, laser_y, laser_z)) # (3,360)
        laser_reading = laser_reading.T #(360,3)
        

        # return laser_reading
    def laser_map(self):
        '''
        constuct a laser result map, where detected points will be 1, unknown space is -1 and free space is 0
        '''

        

    # def map_reform(self):
    #     '''
    #     reconstruct a map including original map and laser readings
    #     '''
    #     # pass
    #     pixel_value = 80 # pixel value for overlay

    #     # convert laser reading to axis(int)
    #     laser_reading = visualize_laser(self)
    #     laser_axis = np.rint(laser_reading)
    #     laser_axis = laser_axis.astype(int)
    #     new_map = np.zeros_like(self.map)
    #     # get axes
    #     overlay_x = laser_axis[0,:]
    #     overlay_y = laser_axis[1,:]
    #     # use numpy advanced indexing
    #     new_map[overlay_x, overlay_y] = pixel_value
    #     self.map += new_map

    # def map_saver(self):
    #     '''
    #     save the map at a set interval
    #     '''
    #     curr_map = self.map
    #     outfile = '../data/map_time_' + str(self.interval)
    #     np.save(outfile, curr_map)



class SD_algo(object):
    def __init__(self, height, width):
        self.static_map = -1  * np.ones((height, width))
        self.dynamic_map = -1 * np.ones((height, width))
        self.dict_static = {} 
        self.dict_dynamic = {}
        self.prob_dict()
        
    def update_sd_map(self, laser_measure):
        '''
        \param: laser_measure is a H x W array

        '''
        #static map
        free_mask = (self.static_map >= 0) & (self.static_map < 90) # H x W 
        unk_mask = self.static_map == -1 # H x W
        occ_mask = self.static_map > 90 # H x W
        
        laser_free_mask = laser_measure == 0  # H x W
        laser_occ_mask = laser_measure == 1 # H x W
        laser_unk_mask = laser_measure == -1 # H x W

        tmp = np.log(self.static_map / (101 - self.static_map))

        #free | free\
        tmp1 = np.exp(np.log(0.3 / 0.7) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(0.1 / 0.9) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(0.1 / 0.9) + tmp)
        map_fo = tmp4 /  (1 + tmp4)      
        mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(0.7 / 0.3) + tmp)
        map_uo = tmp5 /  (1 + tmp5)
        mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(0.9 / 0.1) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        mask_oo = occ_mask & laser_occ_mask

        mask_not = ~(mask_ff | mask_uf | mask_of | mask_fo | mask_uo | mask_oo)



        #dynamic map

        #free | free
        tmp1 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        # mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        # mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(0.2 / 0.8) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        # mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(0.9 / 0.1) + tmp)
        map_fo = tmp4 /  (1 + tmp4)      
        # mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(0.3 / 0.7) + tmp)
        map_uo = tmp5 /  (1 + tmp5)
        # mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(0.6 / 0.5) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        # mask_oo = occ_mask & laser_occ_mask

        # mask_not = ~(mask_ff | mask_uf | mask_of | mask_fo | mask_uo | mask_oo)

        self.dynamic_map = self.static_map * mask_not + map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo 
        
        self.static_map = self.static_map * mask_not + map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo 
        
        return



'''
info_sub class is used to subscribe to the ROS topic published by the Gazebo simulation/ turtlebot3.
Here, we will retrieve the map information(values, height, weight, origin), laser readings, robot positions
'''
class info_sub(object):
    def __init__(self):
        self.map_height = 384
        self.map_width = 384
        self.map_origin = [0,0,0,0,0,0,0]
        self.robot_pose = [0,0,0,0,0,0,0]
        self.detection = []
        # var defined in self.functions

        
    def laser_reading(self, msg):
        '''
        Read in laser reading results from topic /scan
        '''
        max_reading = msg.range_max
        self.detection = msg.ranges

    def map_info(self, info):
        '''
        Retrieve map info from topic /map_megadata
        '''
        self.map_height = info.height
        self.map_width = info.width
        self.map_origin[:] = [info.origin.position.x,info.origin.position.y, info.origin.position.z, \
                              info.origin.orientation.x, info.origin.orientation.y, info.origin.orientation.z, info.origin.orientation.w ]
        # self.map = np.zeros((self.map_width, self.map_height))
        # print("height: ", self.map_height)
        # print("width: ", self.map_width)
        # print("origin: ",self.map_origin)

    def map_output(self, map):
        pyplot.close()
        # print(np.array(map.data).shape)
        self.map = np.array(map.data)
        print(max(self.map))
        lv = map_reconstructor(self.robot_pose, self.detection, self.map) 
        lv.visualize_laser()
        lv.map_reform()
        lv.map_saver()
        # self.map_visualization()

    def map_visualization(self):
        # tell imshow about color map so that only set colors are used
        self.map = self.map.reshape((self.map_width, self.map_height))
        cmap = mpl.colors.ListedColormap(['grey', 'white','black'])
        bounds=[-6,0,90,200]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        img = pyplot.imshow(self.map,cmap = cmap,norm=norm)
        pyplot.show(block=False)

    def robot_states(self, state):
        # turtlebot3 is at the forth element
        turtlebot3_ind = 3
        pose = state.pose[turtlebot3_ind].position
        orien = state.pose[turtlebot3_ind].orientation
        self.robot_pose[:3] = [pose.x, pose.y, pose.z]
        self.robot_pose[3:] = [orien.x, orien.y, orien.z, orien.w]
        # print(self.robot_pose)

    def listener(self):
        rospy.init_node('info_listener', anonymous=True)
        rate = rospy.Rate(10)
        #ROS subscribers subscribe to different topics to retrieve data from the simulation
        rospy.Subscriber("/scan", LaserScan, self.laser_reading)
        rospy.Subscriber("/map_metadata", MapMetaData, self.map_info)
        rospy.Subscriber("/map", OccupancyGrid, self.map_output, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.robot_states)
            

if __name__ == '__main__':
    sub = info_sub()
    sub.listener()
    rospy.spin()