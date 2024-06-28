#!/usr/bin/env python2.7

import sys
import rospy
from gps_collector.srv import *
import csv


def get_data_client():
    rospy.wait_for_service('get_data')
    getPoint = rospy.ServiceProxy('get_data', EntregarData)
    response = getPoint(True)
    return response
     

def save_points(savingAddress):
    response = get_data_client()
    with open(savingAddress, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([str(response[0], str(response[1]))]) #latitude and longitude

def get_several_points(savingAddress, numberOfPoints):
    for _ in range(numberOfPoints):
        save_points(savingAddress)


if __name__ == "__main__":
    print("the length is: {}".format(len(sys.argv)))
    if len(sys.argv) == 3:
        name = sys.argv[1]
        numberOfPoints = sys.argv[2]
        savingAddress = "../target_points/" + name + ".csv"
        get_several_points(savingAddress, numberOfPoints)
    else:
        print("Insert the name first, and then the number of points")
        




    #if len(sys.argv) == 3:
    #    x = int(sys.argv[1])
    #    y = int(sys.argv[2])
    #else:
    #    print(usage())
    #    sys.exit(1)
    #print("Requesting %s+%s"%(x, y))
    #print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))