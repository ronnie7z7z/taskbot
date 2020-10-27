#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('task_des')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class PID:
        
    def __init__(self, kp, ki, kd, outMin, outMax, iMin, iMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.outMin = outMin
        self.outMax = outMax
        self.intMin = iMin
        self.intMax = iMax

        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def resetValues(self):
        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def pidExecute(self, should, actValue):
        now = rospy.Time.now().to_nsec()
        timeChange = now - self.lastTime
        error = should - actValue
        newErrorSum = self.sumError + (error * timeChange)

        if((newErrorSum >= self.intMin) and (newErrorSum <= self.intMax)):
            self.sumError = newErrorSum
        dError = (error - self.lastError) / timeChange
        output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)
        self.lastError = error
        self.last = now
        if(output > self.outMax):
            output = self.outMax
        if(output < self.outMin):
            output = self.outMin
        return output

class pid_solver:

  def __init__(self):
    self.data_pub = rospy.Publisher("task_des/joint1_position_controller/command", Float64, queue_size = 10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("task_des/taskbot/camera/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower = np.array([0,50,20])
    upper= np.array([5,255,255])
    mask1 = cv2.inRange(hsv, lower, upper)

    lower = np.array([175,150,20])
    upper= np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower, upper)

    mask = mask1 + mask2

    cnt, h, z = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    x,y,b,h = cv2.boundingRect(cnt)

    i = x+b/2
    kp = 0.015
    kd = 0.002
    ki = 0.005
    outMin = -5
    outMax = 5
    iMin = -0.01
    iMax = 0.01
    
    pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)

    output = pid.pidExecute(400 ,i)


    try:
      self.data_pub.publish(output)
    except:
      print("Error")


def main(args):
  rospy.init_node('difference_creator', anonymous=True)
  ic = pid_solver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)