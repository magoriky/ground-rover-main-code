#!/usr/bin/env python2.7
# license removed for brevity

import rospy
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import geonav_transform.geonav_conversions as gc
import tf
waypointTextFile = "/home/jeewon/Desktop/catkin_folder/src/waypoint/waypoint_files/waypoints.txt" 






def transformG2R(lat, lon): # This transform from google to RTK coordinates
    transformation = np.array([[1.006554327206685, -0.001933981529874],[0.00057828, 0.999829510241398]])
    rtkCoordinate = transformation.dot(np.array([[lat],[lon]]))
    rtklat, rtklon  = rtkCoordinate[0], rtkCoordinate[1]
    return rtklat, rtklon

def transformPoint(point):
    lat, lon = point[0], point[1]
    transformedLat, transformedLon = transformG2R(lat,lon)
    transformedPoint = [transformedLat, transformedLon]
    return transformedPoint 

def feedback_callback(feedback):
    if feedback.base_position.status.status ==3:
        rospy.loginfo("Goal reached")
    else:
        rospy.loginfo("Moving towards the goal...")

def get_xy_based_on_lat_long(lat,lon):
    olon = 126.667148937
    olat = 37.3736563615
    #olat = rospy.get_param('starting_point/latitude')
    #olon = rospy.get_param('starting_point/longitude')

    olat, olon = transformG2R(olat, olon)
    xg2, yg2 = gc.ll2xy(lat,lon,olat,olon)
    return xg2, yg2

def movebase_client(points_list):
    e = 0.0
    for i in range(len(points_list)-1):
        p0 = points_list[i]
        p1 = points_list[i+1]
        p0 = transformPoint(p0)
        p1 = transformPoint(p1)
        print("this is p0")
        print(p0)
        print("this is p1")
        print(p1)

        x, y =get_xy_based_on_lat_long(p1[0],p1[1])
        x2, y2 =gc.ll2xy(p1[0],p1[1],p0[0],p0[1])
        
        print("Moving {} forward (x) and {} left (y), distance {} from origin".format(y, -x,np.sqrt((y)**2 + (x)**2) ))

        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        xt = round(y,2)/(1 - e)
        yt = round(-x,2)/(1 - e)
        

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = xt*np.cos(np.deg2rad(4)) - yt*np.sin(np.deg2rad(4))   #The output of "get_xy_based_on_lat_long" assumes north is in Y direction
        goal.target_pose.pose.position.y = xt*np.sin(np.deg2rad(4)) + yt*np.cos(np.deg2rad(4)) #The output of "get_xy_based_on_lat_long" assumes east is in X direction
        vector = complex(y2,-x2)
        ang = np.angle(vector)
        quat = tf.transformations.quaternion_from_euler(0.0,0.0,np.angle(vector))

        #goal = MoveBaseGoal()
        #goal.target_pose.header.frame_id = "map"
        #goal.target_pose.header.stamp = rospy.Time.now()
        #goal.target_pose.pose.position.x = round(y,2)/(1 + e)   #The output of "get_xy_based_on_lat_long" assumes north is in Y direction
        #goal.target_pose.pose.position.y = round(-x,2)/(1 + e) #The output of "get_xy_based_on_lat_long" assumes east is in X direction
        #vector = complex(y2,-x2)
        #ang = np.angle(vector)
        #quat = tf.transformations.quaternion_from_euler(0.0,0.0,np.angle(vector))
      
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        rospy.loginfo("Sending point:{} ({},{}) , latitud, longitud".format(i+1,p1[0],p1[1]))

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        rospy.loginfo("trying to go into next point")
    rospy.signal_shutdown("Shutdown requested")

def getPointsList(fileName):
    Points = []
    with open (fileName, 'r') as textFile:
        for line in textFile:
            pairCoordinate = line.strip().split()
            lat, long = float(pairCoordinate[0]), float(pairCoordinate[1])
            Points.append((lat, long))
    return Points
           


if __name__ == '__main__':
    points_circle= [(37.373682, 126.667121), (37.373670, 126.667136),
                    (37.373656, 126.667146), (37.373647, 126.667153),
                    (37.373625, 126.667160),(37.373608, 126.667159),
                    (37.373592, 126.667158), (37.373570,126.667144),
                    (37.373564, 126.667139),(37.373554, 126.667121),
                    (37.373543, 126.667111), (37.373540, 126.667096),
                    (37.373535, 126.667067), (37.373533, 126.667060),
                    (37.373538, 126.667038),(37.373544, 126.667009),
                    (37.373551, 126.666995) ]
    points3 = [points_circle[0], points_circle[8], points_circle[16]]
    points5 = [points_circle[0], points_circle[4], points_circle[8], points_circle[12], points_circle[16]]
    points9 = [points_circle[0], points_circle[2], points_circle[4], points_circle[6], points_circle[8], points_circle[10], 
                points_circle[12], points_circle[14], points_circle[16]]
    points17 = points_circle
    #points = [(37.373825, 126.667260),(37.373818, 126.667251), (37.373811, 126.667246), (37.373803, 126.667237),(37.373795, 126.667228), 
    #(37.373791, 126.667222), (37.373783, 126.667216), (37.373774, 126.667209), (37.373767, 126.667202),
    #(37.373759, 126.667196), (37.373751, 126.667190), (37.373742, 126.667180), (37.373736,126.667173), (37.373728,126.667165),
    # (37.373721,126.667158), (37.373712, 126.667149), (37.373704, 126.667141), (37.373696, 126.667132), ( 37.373689, 126.667128), (37.373683, 126.667120)]

    #points2 = [(37.373682, 126.667119),(37.373676, 126.667130), (37.373673, 126.667133), (37.373665, 126.667139),(37.373656, 126.667146), 
    ##(37.373650, 126.667150), (37.373643, 126.667153), (37.373636, 126.667155), (37.373627, 126.667158),
    #(37.373619, 126.667158), (37.373611, 126.667159), (37.373604, 126.667157), (37.373595, 126.667155), (37.373586, 126.667151), (37.373577, 126.667147),
    #(37.373569,126.667140)]

    #points3 = [(37.373825, 126.667260), (37.373912,126.666940), (37.373628, 126.666669), (37.373551,126.666995), (37.373824, 126.667257)]
    #points4 = [(37.373825,126.667260),(37.373683,126.667118), (37.373667,126.666976),(37.373552,126.666992),(37.373567,126.667136),(37.373683,126.667118)]
    #points = [(37.2225,126.402)]
    #points = [(37.373561,126.667138),(37.373623,126.667061)]
    #points = [ (37.373683,126.667119), (37.373623,126.667061)]
    #points = [(2,0),(3,0.5),(4.5,2),(5.5,1),(6.5,0),(6.5,1.5)]
    #points = [(1*2/3,0) , (2*2/3,0.5*2/3), (3*2/3,1*2/3), (2*2/3,2.5*2/3), (4*2/3,3*2/3)]
    #points = [(1,0) , (2,0)]
    #points = [(4,0)]
    #points = [(8/3,0)]
    #points = [(0,-3*2/3)]
    #points = [(0,0)]
    #points = [(37.373667, 126.667135), (37.373554, 126.667153), (37.373538, 126.667009), (37.373658, 126.666997),(37.373667, 126.667135)]
    #points = [(37.373683, 126.667116), (37.373573, 126.667141), (37.373551, 126.666997), (37.373671, 126.666978),(37.373683, 126.667116)]
    
    #newpoints = divideSeveralPoints(points,1)
    newpoints = getPointsList(waypointTextFile)
    print(newpoints)
    rospy.init_node('movebase_client_py')      
    movebase_client(newpoints)
    