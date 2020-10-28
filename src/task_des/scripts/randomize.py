#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkState
import math

def sin_multiple():
    comm2 = rospy.Publisher('/task_des/joint2_position_controller/command', 
                            Float64, 
                            queue_size=10)
    linkpub1 = rospy.Publisher('/gazebo/set_link_state',
                            LinkState,
                            queue_size=10)
    linkpub2 = rospy.Publisher('/gazebo/set_link_state',
                            LinkState,
                            queue_size=10)
    
    rospy.init_node('sine_multiple', anonymous=True)
    rate = rospy.Rate(60) # 60hz
    while not rospy.is_shutdown():
        b = 0.1*math.sin(2*float(rospy.get_time())+3.2) + 0.2*math.sin(0.4*float(rospy.get_time())+1.2) #Oscillation given to Red Ball in y

        base_msg1 = LinkState()
        base_msg1.link_name = 'base'
        base_msg1.pose.position.x = 0.3*math.sin(0.7*float(rospy.get_time()) + 5) + 0.03*math.sin(3*float(rospy.get_time()) + 2) #Oscillation given to 
        base_msg1.pose.position.y = 0.15*math.sin(2*float(rospy.get_time()) + 3) + 0.08*math.sin(0.3*float(rospy.get_time()) + 1) #the base of the setup with the camera
        base_msg1.pose.position.z = 0.049
        base_msg1.pose.orientation.x = 0 
        base_msg1.pose.orientation.y = 0
        base_msg1.pose.orientation.z = 0
        base_msg1.pose.orientation.w = 1

        base_msg2 = LinkState()
        base_msg2.link_name = 'base2'
        base_msg2.pose.position.x = 2 + 0.2*math.sin(2*float(rospy.get_time()) + 1) + 0.03*math.sin(4*float(rospy.get_time()) + 2) #Oscillation given to 
        base_msg2.pose.position.y = 0.1*math.sin(2*float(rospy.get_time()) + 1) - 0.1*math.sin(0.5*float(rospy.get_time()) + 1) #the base of the ball
        base_msg2.pose.position.z = 0.049
        base_msg2.pose.orientation.x = 0 
        base_msg2.pose.orientation.y = 0
        base_msg2.pose.orientation.z = 0
        base_msg2.pose.orientation.w = 1

        linkpub1.publish(base_msg1)
        linkpub2.publish(base_msg2)
        comm2.publish(b)
        rate.sleep()

if __name__ == '__main__':
    try:
        sin_multiple()
    except rospy.ROSInterruptException:
        pass