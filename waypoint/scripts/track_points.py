#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import geonav_transform.geonav_conversions as gc
import tf

def feedback_callback(feedback):
    if feedback.base_position.status.status ==3:
        rospy.loginfo("Goal reached")
    else:
        rospy.loginfo("Moving towards the goal...")


def movebase_client(points_list):
    e = 0.52
    for j,i in enumerate(points_list):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = round(i[0],2)/(1 + e)   #The output of "get_xy_based_on_lat_long" assumes north is in Y direction
        goal.target_pose.pose.position.y = round(i[1],2)/(1 + e) #The output of "get_xy_based_on_lat_long" assumes east is in X direction
        
        
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,i[2]*(np.pi/180))
      
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        rospy.loginfo("Sending point:{} ({},{})".format(j,i[0],i[1],i[2]))

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        rospy.loginfo("trying to go into next point")
    rospy.signal_shutdown("Shutdown requested")
       



if __name__ == '__main__':
    x1, y1, th1 = 4, 0, 0
    x2, y2, th2 = x1 + 2*np.cos(np.deg2rad(60)), -2*np.sin(np.deg2rad(60)), -60
    x3, y3, th3 = x2 + 5*np.cos(np.deg2rad(30)), y2 + 5*np.sin(np.deg2rad(30)), 30
    x4, y4, th4 = x3 -4*np.sin(np.deg2rad(30)), y3 + 4*np.cos(np.deg2rad(30)), 120
   
   

    points = [(x1,y1,th1), (x2,y2,th2), (x3,y3,th3), (x4,y4,th4)]
    rospy.init_node('movebase_client_py')      
    movebase_client(points)
    