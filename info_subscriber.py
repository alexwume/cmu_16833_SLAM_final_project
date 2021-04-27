#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from matplotlib import pyplot
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib as mpl
from pyquaternion import Quaternion
from gazebo_msgs.msg import ModelStates 


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
        self.map = curr_map
        self.map_resolution = 0.05
        self.map_origin = np.array([-10, -10, 0])
        self.laser_map = -1 * np.ones((384, 384))


    def create_laser_map(self):
        '''
        constuct a laser result map, where detected points will be 1, unknown space is -1 and free space is 0
        '''
        if len(self.laser_data) == 0:
            return

        # create quaternion
        curr_quater = Quaternion(self.robot_pose[6], self.robot_pose[3], self.robot_pose[4], self.robot_pose[5])
        # retrieve theta from quaternion
        theta = curr_quater.angle * 180 / np.pi # convert it into degree to do increment later

        # print("theta", theta * 180 / np.pi )

        ang = np.arange(360) + theta# 0 ~ 359 degree
        ang = ang * np.pi / 180 # in radian
        # print(len(self.laser_data * np.cos(ang)))
       

        for i in range(360):
            cur_dist = 0
            laser_dist = self.laser_data[i]
            while cur_dist < laser_dist:
                lx = self.robot_pose[0] +  cur_dist * np.cos((theta + i) * np.pi / 180) - self.map_origin[0] # 360 x 1
                ly = self.robot_pose[1] +  cur_dist * np.sin((theta + i) * np.pi / 180) - self.map_origin[1]# 360 x 1

                i_x = (lx / self.map_resolution).astype(int)
                i_y = (ly / self.map_resolution).astype(int)
                self.laser_map[i_x, i_y] = 0
                cur_dist = cur_dist + self.map_resolution

        
        laser_x = self.robot_pose[0] +  self.laser_data * np.cos(ang) # 360 x 1
        laser_y = self.robot_pose[1] +  self.laser_data * np.sin(ang) # 360 x 1

        # visualize the laser results, this is only for testing purposes
        self.laser_visualization_rviz(laser_x, laser_y)

        laser_x -= self.map_origin[0]
        laser_y -= self.map_origin[1]

        ind_x = (laser_x / self.map_resolution).astype(int)
        ind_y = (laser_y / self.map_resolution).astype(int)

        # set the detected points to 1
        self.laser_map[ind_x, ind_y] = 1
        # visualize the laser_map for testing purposes
        # self.laser_map_visual()


    def laser_map_visual(self):
        # tell imshow about color map so that only set colors are used
        pyplot.close()
        cmap = mpl.colors.ListedColormap(['grey', 'white','black'])
        bounds=[-2,0,1,10]
        norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
        robot_xind = int((self.robot_pose[0] - self.map_origin[0]) / self.map_resolution)
        robot_yind = int((self.robot_pose[1] - self.map_origin[1]) / self.map_resolution)
        self.laser_map[robot_xind, robot_yind] = 1
        
        img = pyplot.imshow(self.laser_map,cmap = cmap,norm=norm)
        pyplot.show(block=False)
        

    def laser_visualization_rviz(self, x_data, y_data):
        
        topic = 'visualization_marker_array'
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=360)

        markerArray = MarkerArray()

        for count in range(360):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x_data[count]
            marker.pose.position.y = y_data[count]
            marker.pose.position.z = 1
            markerArray.markers.append(marker)
            count += 1
            publisher.publish(markerArray)


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
        self.detection = np.array(msg.ranges)
        mask = self.detection < max_reading
        self.detection = self.detection * mask
        self.detection = np.nan_to_num(self.detection) + ~mask * max_reading

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
        self.map = np.array(map.data).reshape((self.map_height, self.map_width))
        lv = map_reconstructor(self.robot_pose, self.detection, self.map) 
        lv.create_laser_map()

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