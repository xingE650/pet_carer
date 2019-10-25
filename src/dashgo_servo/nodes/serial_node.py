#!/usr/bin/env python
#coding:utf8
from __future__ import print_function
import rospy
from serial_ctrl import servoCtrl
from dashgo_msgs.srv import *

class servoNode():
    def __init__(self):
        rospy.init_node("servo_ctrl")
        rospy.on_shutdown(self.shutdown)
        self.servo_ctrl = servoCtrl()
        self.count =0

        self.server = rospy.Service("servo_ctrl",servo_ctrl,self.response) 
        rospy.spin()

    def response(self,req):
        res = servo_ctrlResponse()
        position = req.angles
        self.servo_ctrl.setPosition(position)
        res.result = True
        rospy.loginfo('servo control compelete')
        return res
       
      
    def shutdown(self):
        rospy.loginfo("i will stop spl function")
        self.servo_ctrl.stop()

if __name__ =="__main__":
    servoNode()
