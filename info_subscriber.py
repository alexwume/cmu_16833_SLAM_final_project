#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid

from visualization_msgs.msg import Marker, MarkerArray
import matplotlib
from matplotlib import pyplot
from gazebo_msgs.msg import ModelStates 
from scipy.spatial.transform import Rotation as R

'''
author: Alex Wu
email: chichiaw@andrew.cmu.edu
purpose: cmu-16883 final project
'''

class map_reconstructor(object):
    def __init__(self, robot_pose, laser_data):
        '''
        \param: robot_pose: robot poses from gazebo simulation. [xyz(translation), xyzw(orientation quaternion)]
        \param: laser_reading: 360 laser readings
        '''
        self.robot_pose = robot_pose #[x,y,z, qx,qy,qz,qw] (7,)
        
        self.laser_data = laser_data #360 laser readings (360, )
        self.laser_max = 3.5
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
        r = R.from_quat(self.robot_pose[3:])


        # retrieve theta from quaternion
        theta = r.as_euler('zyx', degrees = True)[0]
        # print("degree: ", theta)


        for i in range(360):
            cur_dist = 0
            laser_dist = self.laser_data[i]
            while cur_dist < laser_dist:
                lx = self.robot_pose[0] +  cur_dist * np.cos((theta + i) * np.pi / 180) - self.map_origin[0] # 360 x 1
                ly = self.robot_pose[1] +  cur_dist * np.sin((theta + i) * np.pi / 180) - self.map_origin[1]# 360 x 1
                # lx = self.robot_pose[0] +  cur_dist * np.cos((theta + i) * np.pi / 180) # 360 x 1
                # ly = self.robot_pose[1] +  cur_dist * np.sin((theta + i) * np.pi / 180) # 360 x 1
                

                i_x = (lx / self.map_resolution).astype(int)
                i_y = (ly / self.map_resolution).astype(int)
                self.laser_map[i_x, i_y] = 0
                cur_dist = cur_dist + self.map_resolution
        
        
        ang = np.arange(360) + theta # 0 ~ 359 degree
        ang = ang * np.pi / 180 # in radian

        # make sure the max range reading laser doesn't assume there is an object at the end of the laser range
        laser_nonmax_mask = self.laser_data < self.laser_max
        self.laser_data = self.laser_data[laser_nonmax_mask]
        ang = ang[laser_nonmax_mask]
        
        laser_x = self.robot_pose[0] +  self.laser_data * np.cos(ang) - self.map_origin[0]
        laser_y = self.robot_pose[1] +  self.laser_data * np.sin(ang) - self.map_origin[1]


        # visualize the laser results, this is only for testing purposes
        # self.laser_visualization_rviz(laser_x, laser_y)

        ind_x = (laser_x / self.map_resolution).astype(int)
        ind_y = (laser_y / self.map_resolution).astype(int)

        # set the detected points to 1
        self.laser_map[ind_x, ind_y] = 1

        # visualize the laser_map for testing purposes
        # self.laser_map_visual()



    def laser_map_visual(self):
        # tell imshow about color map so that only set colors are used
        pyplot.close()
        cmap = matplotlib.colors.ListedColormap(['grey', 'white','black'])
        bounds=[-2,0,1,10]
        norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
        robot_xind = int((self.robot_pose[0] - self.map_origin[0]) / self.map_resolution)
        robot_yind = int((self.robot_pose[1] - self.map_origin[1]) / self.map_resolution)
        self.laser_map[robot_xind, robot_yind] = 1
        
        img = pyplot.imshow(self.laser_map,cmap = cmap,norm=norm)
        pyplot.show(block=False)
        

    def laser_visualization_rviz(self, x_data, y_data):
        
        num = len(x_data)
        # num = 1 #debug 
        # print("visualize laser pose")
        # print(x_data, y_data)
        topic = 'visualization_marker_array'
        publisher = rospy.Publisher(topic, MarkerArray, queue_size=1)

        markerArray = MarkerArray()

        for count in range(num):
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.id = count
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # marker.pose.position.x = x_data #debug
            # marker.pose.position.y = y_data #debug

            marker.pose.position.x = x_data[count] 
            marker.pose.position.y = y_data[count]
            marker.pose.position.z = 1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            markerArray.markers.append(marker)
        publisher.publish(markerArray)


class SD_algo(object):
    def __init__(self, height, width, resolution):
        self.height = height
        self.width = width
        self.static_map = -1  * np.ones((height, width))
        self.dynamic_map = -1 * np.ones((height, width))
        self.counter = 0
        self.resolution = resolution
        print("initialized static/ dynamic map")
        self.pub_stat = rospy.Publisher('/static_map', OccupancyGrid, queue_size=10)
        self.pub_dyn = rospy.Publisher('/dynamic_map', OccupancyGrid, queue_size=10)
        
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

        #laser masks
        laser_free_mask = laser_measure == 0  # H x W
        laser_occ_mask = laser_measure == 1 # H x W
        laser_unk_mask = laser_measure == -1 # H x W


 
        #dynamic map
                        #   ff   uf   of   fo   uo   oo
        dynamic_map_para = [0.1, 0.4, 0.1, 0.9, 0.2, 0.05]

        # deal with the numerical issue when unknown cell is intialized to -1
        reformed_dynamic_map = self.dynamic_map * nonzero_mask_d + eps *np.ones_like(self.dynamic_map) * ~nonzero_mask_d

        tmp = np.log(reformed_dynamic_map / (101 - reformed_dynamic_map))

        #free | free
        tmp1 = np.exp(np.log(dynamic_map_para[0] / (1 - dynamic_map_para[0])) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(dynamic_map_para[1] / (1 - dynamic_map_para[1])) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(dynamic_map_para[2] / (1 - dynamic_map_para[2])) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(dynamic_map_para[3] / (1 - dynamic_map_para[3])) + tmp)
        map_fo = tmp4 /  (1 + tmp4)      
        mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(dynamic_map_para[4] / (1 - dynamic_map_para[4])))
        map_uo = tmp5 /  (1 + tmp5)
        mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(dynamic_map_para[5] / (1 -dynamic_map_para[5])) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        mask_oo = occ_mask & laser_occ_mask

        mask_not = ~(mask_ff | mask_uf | mask_of | mask_fo | mask_uo | mask_oo)
        # print(self.dynamic_map)

        self.dynamic_map = self.dynamic_map * mask_not + 100 * (map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo) 

        unk_mask_d = self.dynamic_map == -1
        self.dynamic_map = np.clip(self.dynamic_map, 10, 100) * ~unk_mask_d + self.dynamic_map * unk_mask_d         
        #static map
                        #   ff   uf   of   fo   uo   oo
        static_map_param = [0.1, 0.1, 0.3, 0.3, 0.9, 0.9]


        # deal with the numerical issue when unknown cell is intialized to -1
        reformed_static_map = self.static_map * nonzero_mask + eps *np.ones_like(self.static_map) * ~nonzero_mask

        tmp = np.log(reformed_static_map / (101 - reformed_static_map))

        #free | free
        tmp1 = np.exp(np.log(static_map_param[0] / (1 - static_map_param[0])) + tmp)
        map_ff = tmp1 /  (1 + tmp1)
        # mask_ff = free_mask & laser_free_mask

        #unk | free
        tmp2 = np.exp(np.log(static_map_param[1] / (1 - static_map_param[1])) + tmp)
        map_uf = tmp2 /  (1 + tmp2)
        # mask_uf = unk_mask & laser_free_mask

        #occ | free
        tmp3 = np.exp(np.log(static_map_param[2] / (1 - static_map_param[2])) + tmp)
        map_of = tmp3 /  (1 + tmp3)
        # mask_of = occ_mask & laser_free_mask
 
        #free | occ
        tmp4 = np.exp(np.log(static_map_param[3] / (1 - static_map_param[3])) + tmp)
        map_fo = tmp4 /  (1 + tmp4)
        # mask_fo = free_mask & laser_occ_mask

        #unk | occ
        tmp5 = np.exp(np.log(static_map_param[4] / (1 - static_map_param[4])))
        map_uo = tmp5 /  (1 + tmp5)
        # mask_uo = unk_mask & laser_occ_mask

        #occ | occ
        tmp6 = np.exp(np.log(static_map_param[5] / (1 - static_map_param[5])) + tmp)
        map_oo = tmp6 /  (1 + tmp6)
        # mask_oo = occ_mask & laser_occ_mask


        self.static_map = self.static_map * mask_not + 100 * (map_ff * mask_ff + map_uf * mask_uf + map_of * mask_of \
                            + map_fo * mask_fo + map_uo * mask_uo + map_oo * mask_oo) 
        
        unk_mask = self.static_map == -1
        self.static_map = np.clip(self.static_map, 10, 100) * ~unk_mask + self.static_map * unk_mask 

        # self.counter +=1
        # if self.counter == 2:
            # self.counter = 0
        self.static_map_visual()
        self.dynamic_map_visual()

    def static_map_visual(self):
        # tell imshow about color map so that only set colors are used
        # pyplot.close()
        # cmap = matplotlib.colors.ListedColormap(['grey', 'white','black'])
        # bounds=[-2,0,90,120]
        # norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
        # img = pyplot.imshow(self.static_map,cmap = cmap,norm=norm)
        # pyplot.show(block=False)
        map_stat = OccupancyGrid()
        map_stat.info.width = self.width
        map_stat.info.height = self.height
        map_stat.info.origin.position.x = -10.0
        map_stat.info.origin.position.y = -10.0
        map_stat.info.origin.position.z =   0.0
        map_stat.info.origin.orientation.x = 0.0
        map_stat.info.origin.orientation.y = 0.0
        map_stat.info.origin.orientation.z = 0.0
        map_stat.info.origin.orientation.w = 1.0
        map_stat.info.resolution = self.resolution
        stat = (self.static_map.T).astype(int)
        map_stat.data = np.reshape( stat.tolist(), (self.width*self.height))
        self.pub_stat.publish(map_stat)
 
    def dynamic_map_visual(self):
        # tell imshow about color map so that only set colors are used
        # pyplot.close()
        # cmap = matplotlib.colors.ListedColormap(['grey', 'white','black'])
        # bounds=[-2,0,90,120]
        # norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
        # img = pyplot.imshow(self.dynamic_map,cmap = cmap,norm=norm)
        # pyplot.show(block=False)

        map_dyn = OccupancyGrid()
        map_dyn.info.width = self.width
        map_dyn.info.height = self.height
        map_dyn.info.origin.position.x = -10.0
        map_dyn.info.origin.position.y = -10.0
        map_dyn.info.origin.position.z =   0.0
        map_dyn.info.origin.orientation.x = 0.0
        map_dyn.info.origin.orientation.y = 0.0
        map_dyn.info.origin.orientation.z = 0.0
        map_dyn.info.origin.orientation.w = 1.0
        map_dyn.info.resolution = self.resolution
        dyn = (self.dynamic_map.T).astype(int)
        map_dyn.data = np.reshape(dyn.tolist(), (self.width * self.height))
        self.pub_dyn.publish(map_dyn)


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
        self.resolution = 0.05
        self.sd_al = SD_algo(self.map_height, self.map_width, 0.05)
        # for debugging purposes
        self.map_o = rospy.Publisher('original_map', OccupancyGrid, queue_size=10)
        self.robot_pose_pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=1)
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
        # pyplot.close()
        # print(np.array(map.data).shape)
        self.map = np.array(map.data).reshape((self.map_height, self.map_width))
        lv = map_reconstructor(self.robot_pose, self.detection) 
        lv.create_laser_map()
        self.sd_al.update_sd_map(lv.laser_map)
        # self.map_visualization()


    def map_visualization(self):
        # tell imshow about color map so that only set colors are used
        # self.map = self.map.reshape((self.map_width, self.map_height))
        # cmap = matplotlib.colors.ListedColormap(['grey', 'white','black'])
        # bounds=[-6,0,90,200]
        # norm = matplotlib.colors.BoundaryNorm(bounds, cmap.N)
        # img = pyplot.imshow(self.map,cmap = cmap,norm=norm)
        # pyplot.show(block=False)


        self.map = self.map.reshape((self.map_width, self.map_height))
        map_o = OccupancyGrid()
        map_o.info.width = self.map_width
        map_o.info.height = self.map_height
        map_o.info.origin.position.x = -10.0
        map_o.info.origin.position.y = -10.0
        map_o.info.origin.position.z =   0.0
        map_o.info.origin.orientation.x = 0.0
        map_o.info.origin.orientation.y = 0.0
        map_o.info.origin.orientation.z = 0.0
        map_o.info.origin.orientation.w = 1.0
        map_o.info.resolution = self.resolution
        data = self.map.astype(int)
        map_o.data = np.reshape(data.tolist(), (self.map_width * self.map_height))
        # stat = (self.map.T).astype(int)
        # map_o.data = np.reshape(self.tolist(), (self.width * self.height))
        self.map_o.publish(map_o)
        

    def robot_states(self, state):
        # turtlebot3 is at the forth element
        turtlebot3_ind = 3
        pose = state.pose[turtlebot3_ind].position
        orien = state.pose[turtlebot3_ind].orientation
        self.robot_pose[:3] = [pose.x, pose.y, pose.z]
        self.robot_pose[3:] = [orien.x, orien.y, orien.z, orien.w]

        # debug: visualizing robot pose in rviz
        pose_pub = PoseStamped()
        pose_pub.header.stamp = rospy.Time.now()
        pose_pub.header.frame_id = "/map"
        pose_pub.pose.position.x = pose.x
        pose_pub.pose.position.y = pose.y
        pose_pub.pose.position.z = pose.z
        pose_pub.pose.orientation.x = orien.x
        pose_pub.pose.orientation.y = orien.y
        pose_pub.pose.orientation.z = orien.z
        pose_pub.pose.orientation.w = orien.w
        # self.create_linear_motion_task(pose_pub).result()
        self.robot_pose_pub.publish(pose_pub) 


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