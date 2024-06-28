import rospy

if __name__ == "__main__":
    rospy.init_node('testing')
    start_time = rospy.get_time()
    #start_time = start_time.to_sec
    print("this is st {}".format(start_time))
    rospy.sleep(2.)
    next_time = rospy.get_time()
    #next_time = next_time.to_sec
    print("this is nt {}".format(next_time))
    dif = next_time - start_time
    print("this is dif {}".format(dif))
        
   