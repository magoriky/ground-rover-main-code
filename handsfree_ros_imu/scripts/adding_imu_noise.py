#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
class NoisyIMU:
    def __init__(self):
        rospy.init_node('correcting_imu', anonymous=True)
        self.subscriber = rospy.Subscriber("handsfree/imu0",Imu, self.callback, queue_size=10)
        self.publisher = rospy.Publisher("handsfree/imu", Imu,queue_size=10)
    
    def callback(self, msg):
        receivedMessage = msg
        receivedMessage.orientation_covariance = [0.008**2, 0, 0, 0,0.005**2, 0, 0, 0, 0.044**2]
        receivedMessage.angular_velocity_covariance = [0.013**2, 0, 0, 0,0.021**2, 0, 0, 0, 0.009**2]
        receivedMessage.linear_acceleration_covariance = [0.060**2, 0, 0, 0,0.058**2, 0, 0, 0, 0.125**2]
        self.publisher.publish(receivedMessage)
        

if __name__ == '__main__':
    noisyIMU = NoisyIMU()
    rospy.spin()
    