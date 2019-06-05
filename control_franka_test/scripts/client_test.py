#!/usr/bin/env python2

from control_franka_test.srv import *
import rospy
import time

def print_info():
  
        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('boardcase')

        try:
            boardcase = rospy.ServiceProxy('boardcase', board_detector)

            # Insert your message variables to be sent as a service request
            resp = boardcase(2)

            print ("Response: ",resp.pt1,resp.chess_location,resp.boardcase)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
 
if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('client_test', anonymous=True)

    # call the robot mover function defined above
    while(True):
        time.sleep(3)
    
        print_info()
    
    # Spin while node is not shutdown
#    while not rospy.is_shutdown():
#    	rospy.spin()

