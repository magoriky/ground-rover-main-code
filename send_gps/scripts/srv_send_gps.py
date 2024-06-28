#!/usr/bin/env python2.7

import rospy
from send_gps.srv import GpsGoal
import geonav_transform.geonav_conversions as gc
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import numpy as np
import tf

class GpsGoalServer:
    def __init__(self):
        rospy.init_node("gps_goal_server")
        self.service = rospy.Service('gps_goal', GpsGoal, self.handle_gps_goal)
        self.goalReached = False

    def get_xy_based_on_lat_long(self,lat,lon):
        olon = 126.667148937
        olat = 37.3736563615
        xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
        return xg2, yg2



    def handle_gps_goal(self, request):
        print("Goal received latitude: {}, longitude: {}".format(request.latitude, request.longitude))
        self.move_2_goal(request.latitude, request.longitude)
        print("Goal Reached = {}".format(self.goalReached))
        self.goalReached = False

    def move_2_goal(self, lat, lon):
        e = 0.0
        x, y = self.get_xy_based_on_lat_long(lat, lon)
        print("waiting for server")
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        xt = round(y,2)/(1 - e)
        yt = round(-x,2)/(1 - e)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xt*np.cos(np.deg2rad(4)) - yt*np.sin(np.deg2rad(4))   #The output of "get_xy_based_on_lat_long" assumes north is in Y direction
        goal.target_pose.pose.position.y = xt*np.sin(np.deg2rad(4)) + yt*np.cos(np.deg2rad(4)) #The output of "get_xy_based_on_lat_long" assumes east is in X direction
        vector = complex(y,-x)
        ang = np.angle(vector)
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,np.angle(vector))
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        print("sending goal")
        client.send_goal(goal)
        print("goal sent")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        self.goalReached = True
        rospy.signal_shutdown("Shutdown requested")
if __name__ == "__main__":
    try:
        server = GpsGoalServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass





