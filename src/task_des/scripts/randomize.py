#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

def sin_multiple():
    comm0 = rospy.Publisher('/task_des/joint0_position_controller/command', 
                            Float64, 
                            queue_size=10)
    comm2 = rospy.Publisher('/task_des/joint2_position_controller/command', 
                            Float64, 
                            queue_size=10)
    rospy.init_node('sine_multiple', anonymous=True)
    rate = rospy.Rate(60) # 60hz
    while not rospy.is_shutdown():
        a = 0.15*math.sin(0.2*float(rospy.get_time())) + 0.08*math.sin(3*float(rospy.get_time()) + 5) #Oscillation given to Base

        b = 0.1*math.sin(2*float(rospy.get_time())+3.2) + 0.2*math.sin(0.4*float(rospy.get_time())+1.2) #Oscillation given to Red Ball
        #rospy.loginfo()
        comm0.publish(a)
        comm2.publish(b)
        rate.sleep()

if __name__ == '__main__':
    try:
        sin_multiple()
    except rospy.ROSInterruptException:
        pass