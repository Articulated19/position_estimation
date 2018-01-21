#!/usr/bin/env python

import rospy
import cv2
import numpy as np
#from custom_msgs import *
#from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive

cap = cv2.VideoCapture(0)

pp = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi', pp, 20.0, (640,480))
text_file = open("ackermann_values.txt", "w")

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    if cap.isOpened():
        ret, frame = cap.read()
        if (ret):
            out.write(frame)
            text_file.write("%f  %f\n"%(data.steering_angle, data.speed))
        else:
            rospy.loginfo("could not create frame\n")   
    else:
        rospy.loginfo("cam not opened")            


def gulliCallback(data):
    text_file.write("%f %f %f\n"%(data.p.x, data.p.y, data.theta1))


def shutdown_ros():
   text_file.close()
   cap.release()
   out.release()


def listener():
#    import os
#    os.system("sudo modprobe bcm2835-v4l2")
    rospy.init_node("listener", anonymous = True)
#    rospy.Subscriber("ACtopic", AckermannDrive, callback)
    rospy.Subscriber("master_drive", AckermannDrive, callback)
#    rospy.Subscriber("truck_state", TruckState, gulliCallback)
    rospy.on_shutdown(shutdown_ros)
    rospy.loginfo("started listener node")
    rospy.spin()
    

if __name__ == '__main__':
    listener()
    #cap.release()
    #out.release()
    #text_file.close()   
