#!/usr/bin/env python3
import serial
import rospy
from sensor_msgs.msg import NavSatFix

code_text ="$GNRMC"
code_text2 ="GNGST"
""" code_text ="$GNGGA" """

pub = rospy.Publisher('cell/fix', NavSatFix, queue_size=10)
navsat_msg = NavSatFix()

def str2numCorrected(num):
    num = float(num)
    residue = num % 100
    num = (num - residue)/100
    residue = residue/60
    correctedNum = num + residue
    return correctedNum

def serial_com_init():      
    ser = serial.Serial(
        port='/dev/ttyUSB1',\
        baudrate=115200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
            timeout=20000/115200)
    print("connected to: " + ser.portstr)
    read_line(ser)

def  read_line(ser):
    coordinates =[]
    deviations =[]
    try:
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode().strip()  # Read a line of data and decode it from bytes to string
                #print(data)
                coordinates = get_info_from_string(data)
                while not deviations:
                    data = ser.readline().decode().strip()
                    deviations = get_noise_from_string(data)
                print("deviations are: ".format(deviations))
                if coordinates and deviations:
                    navsat_msg.header.frame_id = "base_link"
                    navsat_msg.latitude = coordinates[0]
                    navsat_msg.longitude = coordinates[1]
                    navsat_msg.position_covariance = [deviations[0]**2, 0, 0, 0, deviations[1]**2, 0, 0,0,0]
                    navsat_msg.position_covariance_type = 3#We are working in 2D so i does not matter
                    pub.publish(navsat_msg)
                    coordinates =[]
                    deviations = []
                
    except KeyboardInterrupt:
        ser.close()
        print("Serial communication stopped.")



def get_info_from_string(string_message):
    if code_text in string_message:
        line = string_message.split(",")
        latitude , longitude = str2numCorrected(line[3]) , str2numCorrected(line[5])
        print("lat: {}, lon: {}".format(latitude, longitude))
        if latitude =="" and longitude =="":
            latitude = 0.666
            longitude = 0.666
        return [latitude , longitude]
        
        
def get_noise_from_string(string_message):
    if code_text2 in string_message:
        line = string_message.split(",")
        latitudeDeviationError , longitudeDeviationError = float(line[6]) , float(line[7])
        print("latError: {}, lonError: {}".format(latitudeDeviationError, longitudeDeviationError))
        if latitudeDeviationError =="" and longitudeDeviationError =="":
            latitudeDeviationError = 0
            longitudeDeviationError = 0
        return [latitudeDeviationError , longitudeDeviationError]
        
        
        
        #on_message(latitude,longitude)

#class async_client:
#    def __init__(self):
#        self.pub = rospy.Publisher('gps/fix', NavSatFix,queue_size=1)
#     
#def on_message(self, latitude , longitude):
#        try:      
#            navsat_msg.latitude = latitude
#            navsat_msg.longitude = longitude
#            self.pub.publish(navsat_msg)
#
#        except:
#            print("error on_message")
    



if __name__ == '__main__':
    rospy.init_node('rtk-gps-receiver', anonymous=True)
    serial_com_init()
  




""" if __name__ == '__main__':
    try:
        rospy.init_node('mqtt_subscriber', anonymous = True)
        while not rospy.is_shutdown():
            client_initializer = mqtt.Client(client_id="suny", clean_session = True)
            client_initializer.on_connect = client.on_connect
            client_initializer.on_message = client.on_message
            rospy.loginfo("connecting_talker")
            client_initializer.connect(client.address, 1883,60)
            client_initializer.loop_forever()

    except rospy.ROSInterruptException:
        pass    """