#!/usr/bin/env python

# Brief: This node filters the /tracked_humans from the laser data and publishes the filtered laser scan used for hateb
# Author: Phani Teja Singamaneni

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import LaserScan
from human_msgs.msg import TrackedHumans, TrackedSegmentType
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped

class HumanFilter(object):
    def __init__(self):
        rospy.init_node('human_filter')
        self.rate = rospy.Rate(50.0)
        self.filtered_scan = LaserScan()
        self.segment_type = TrackedSegmentType.TORSO
        self.humans = []
        self.laser_transform = TransformStamped()

        rospy.Subscriber('base_scan', LaserScan, self.laserCB)
        rospy.Subscriber('tracked_humans', TrackedHumans, self.humansCB)
        self.laser_pub = rospy.Publisher('/base_scan_filtered', LaserScan, queue_size=10)

        #Intialize tf2 transform listener
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf)

        # Keep the node alive
        rospy.spin()

    def laserCB(self, scan):
        filtered_scan = scan
        filtered_scan.ranges = list(scan.ranges)
        filtered_scan.header.stamp = rospy.Time.now()

        try:
            self.laser_transform = self.tf.lookup_transform('map', 'base_laser_link', rospy.Time(),rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

        if(self.laser_transform.header.frame_id is not ''):
            laser_pose = self.laser_transform.transform.translation

            rot = self.laser_transform.transform.rotation
            r,p,y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            base_laser_dir = [np.cos(y), np.sin(y)] 


            # Filtering humans from the scan
            for hpose in self.humans:
                rh_vec = [hpose.position.x - laser_pose.x, hpose.position.y - laser_pose.y]
                sign = base_laser_dir[0]*-rh_vec[1] + base_laser_dir[1]*rh_vec[0]
                sign = sign/abs(sign)
                t_angle = scan.angle_max - scan.angle_min
                mid_angle = t_angle/2 - sign*np.arccos((base_laser_dir[0]*rh_vec[0]+base_laser_dir[1]*rh_vec[1])/(np.linalg.norm(rh_vec)))
                
                mid_idx = int((mid_angle)/scan.angle_increment)
                if(mid_idx>=len(scan.ranges)):
                    continue
                
                r = 0.4
                d = np.linalg.norm(rh_vec)
                mr = scan.ranges[mid_idx]
                
                if(mr<=(d-r)):
                    continue

                if(r<=d):
                    beta = np.arcsin(r/d)
                else:
                    beta = np.pi/2
            
                min_idx = int(np.floor((mid_angle-beta)/scan.angle_increment))
                max_idx = int(np.ceil((mid_angle+beta)/scan.angle_increment))

                for i in range(min_idx, max_idx):
                    if(i<len(scan.ranges)):
                        filtered_scan.ranges[i] = float('NaN')

        #print (filtered_scan.ranges)
        self.laser_pub.publish(filtered_scan)

        
    def humansCB(self,msg):
        for human in msg.humans:
            for segment in human.segments:
                if(segment.type == self.segment_type):
                    if(len(self.humans)<human.track_id):
                        self.humans.append(segment.pose.pose)
                    else:
                        self.humans[human.track_id-1] = segment.pose.pose


if __name__ == '__main__':
    hfilter = HumanFilter()
