#!/usr/bin/env python2.7

import rospy
from gps_collector.srv import EntregarData
from sensor_msgs.msg import NavSatFix
from time import sleep

class EntregarDataServer:
    def __init__(self):
        rospy.init_node('get_data_server')
        self.service = rospy.Service('get_data', EntregarData, self.handle_entregar_data)
        self.latitude_sum = 0.0
        self.longitude_sum = 0.0
        self.count = 0.0

    def handle_entregar_data(self, request):
        if request.get == True:
            print("I am here True")
            self.latitude_sum = 0.0
            self.longitude_sum = 0.0
            self.count = 0.0
            start_time = rospy.get_time()
            

            while(rospy.get_time() - start_time) < 10.0:
                print("I am iterating")
                fix_data = rospy.wait_for_message('cell/fix', NavSatFix)
                self.latitude_sum += fix_data.latitude
                self.longitude_sum += fix_data.longitude
                self.count = self.count + 1
                rospy.sleep(0.1)
            #print("this is next time {}".format(rospy.Time.now().to_sec))
            #print("this is the dif {}".format ((rospy.Time.now() - start_time)))
            print("I am here count: {}".format(self.count))
            avg_latitude = self.latitude_sum/self.count
            avg_longitude = self.longitude_sum/self.count
            return[avg_latitude,avg_longitude]
        else:
            rospy.logwarn("Invalid request, returning zeros.")
            return [0.0,0.0]
    
if __name__ == "__main__":
    try:
        server = EntregarDataServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
            

