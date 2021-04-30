#!/usr/bin/env python

# Brief: This node subscribes to /tracked_humans and publishes the predicted goal to humans based on their trajectory
# Author: Phani Teja Singamaneni

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PoseStamped
from human_msgs.msg import TrackedHumans, TrackedHuman, TrackedSegmentType
from human_path_prediction.msg import PredictedGoal
from scipy.stats import multivariate_normal
from std_srvs.srv import SetBool, Trigger, TriggerResponse
import matplotlib.pyplot as plt
EPS = 1e-12

class PredictGoal(object):
    def __init__(self, human_num=1):
        self.human_num = human_num

        # laas_adream
        self.goals_x = [1.5, 7.0, 9.0, 10.5, 1.5, 10.3, 8.5]
        self.goals_y = [2.0, 8.0, 12.5, 15.0, 15.0, 1.5, -4.5]
        self.goal_num = 7

        # maze
        # self.goals_x = [1.5,1.5,1.5,1.5,1.5,7.5,25,42,42,41.5,42,37,22,15.5,28.5,37,23.5,10.5,15.5,31.5,20,25.5,7]
        # self.goals_y = [45,15,30,60,87,87,81.5,81.5,66,41.5,22,3,3,12.5,12.5,20.5,21.5,28.5,39.5,47,53,59,59]

        self.predicted_goal = PoseStamped()
        self.last_idx = 0
        self.changed = False
        self.current_poses = [[] for i in range(self.human_num)]
        self.prev_poses = [[] for i in range(self.human_num)]
        self.mv_nd = multivariate_normal(mean=0,cov=0.1)
        self.theta_phi = [[0]*self.goal_num for i in range(self.human_num)]
        self.window_size = 10
        self.probability_goal = [np.array([1.0/self.goal_num]*self.goal_num) for i in range(self.human_num)]
        self.probability_goal_window = [np.array([[1.0/self.goal_num]*self.goal_num]*self.window_size) for i in range(self.human_num)]
        self.done = False
        self.itr = 0

        NODE_NAME = "human_goal_predict"
        rospy.init_node(NODE_NAME)
        self.humans_sub_ = rospy.Subscriber("/tracked_humans",TrackedHumans,self.tracked_humansCB)
        self.goal_pub_ = rospy.Publisher(NODE_NAME+"/predicted_goal",PredictedGoal, queue_size=2)
        self.goal_srv_ = rospy.Service("goal_changed", Trigger, self.goal_changed)
        rospy.spin()

    def tracked_humansCB(self,msg):
        self.prev_poses = self.current_poses
        self.current_poses = [[] for i in range(self.human_num)]

        for human in msg.humans:
            for segment in human.segments:
                if segment.type == TrackedSegmentType.TORSO:
                    self.current_poses[human.track_id-1].append(segment.pose.pose)
        if not self.done:
            self.prev_poses = self.current_poses

        for i in range(0,len(self.current_poses[0])):
            diff = np.linalg.norm([self.current_poses[0][i].position.x - self.prev_poses[0][i].position.x, self.current_poses[0][i].position.y - self.prev_poses[0][i].position.y])

            if diff > EPS or not self.done:
                dist = []
                for j in range(0,len(self.goals_x)):
                    vec1 = np.array([self.goals_x[j],self.goals_y[j],0.0]) - np.array([self.current_poses[0][i].position.x,self.current_poses[0][i].position.y,0.0])  #Vector from current position to a goal
                    rotation = (self.current_poses[0][i].orientation.x,self.current_poses[0][i].orientation.y,self.current_poses[0][i].orientation.z,self.current_poses[0][i].orientation.w)
                    roll,pitch,yaw = tf.transformations.euler_from_quaternion(rotation)
                    unit_vec = np.array([np.cos(yaw), np.sin(yaw),0.0])
                    self.theta_phi[i][j] = (np.arccos(np.dot(vec1,unit_vec)/np.linalg.norm(vec1)))
                    dist.append(np.linalg.norm([self.current_poses[0][i].position.x - self.goals_x[j],self.current_poses[0][i].position.y - self.goals_y[j]]))

                self.probability_goal_window[i][self.itr] = self.mv_nd.pdf(np.array(self.theta_phi[i]));

                self.probability_goal[i] = np.array([1.0]*self.goal_num)
                for k in range(0,len(self.probability_goal_window[i])):
                    gf = np.exp((k-self.window_size)/5)
                    self.probability_goal[i] =  np.power(self.probability_goal_window[i][k],gf)* np.array(self.probability_goal[i]) # Linear prediction of goal

                for ln in range(0,len(self.goals_x)):
                    self.probability_goal[i][ln] = (1/dist[ln])*self.probability_goal[i][ln];

                self.probability_goal[i] = (self.probability_goal[i]-np.min(self.probability_goal[i]))/(np.max(self.probability_goal[i])-np.min(self.probability_goal[i]))


                self.itr = self.itr + 1
                if self.itr == self.window_size:
                    self.itr = 0

                self.done = True

        self.predict_goal()


    def predict_goal(self):
        idx = 0
        max_prob = 0.0
        p_goal = PredictedGoal()

        for i in range(0,len(self.current_poses[0])):
            for j in range(0,len(self.goals_x)):
                if(max_prob<self.probability_goal[i][j]):
                    idx = j
                    max_prob = self.probability_goal[i][j]

            self.predicted_goal.header.stamp = rospy.Time.now()
            self.predicted_goal.header.frame_id = 'map'
            self.predicted_goal.pose.position.x = self.goals_x[idx]
            self.predicted_goal.pose.position.y = self.goals_y[idx]
            self.predicted_goal.pose.position.z = 0.0
            self.predicted_goal.pose.orientation = self.current_poses[0][i].orientation

            if self.last_idx != idx:
                p_goal.changed = True
                self.changed = True

        self.last_idx = idx
        p_goal.goal = self.predicted_goal
        self.goal_pub_.publish(p_goal)


    def goal_changed(self,req):
        if self.changed:
            self.changed = False
            return TriggerResponse(True,"Goal Changed")
        return TriggerResponse(False, "Goal not changed")

if __name__ == '__main__':
    predict_srv = PredictGoal(60)
