#!/usr/bin/env python


import rospy
from position_estimation.msg import AckermannDrive

class Database:
   def __init__(self):
      rospy.init_node("db_listener", anonymous=True)
      self.sub = rospy.Subscriber("truck_cmd", AckermannDrive, self.callback)
      rospy.spin();


   def callback(self, data):
      rospy.loginfo(rospy.get_caller_id() + "I got %s", data.speed)
    

if __name__ == "__main__":
   d = Database() 
