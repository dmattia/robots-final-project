#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def web_pub():
    pub_twist = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
    rospy.init_node('web_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0.3

    while not rospy.is_shutdown():
        pub_twist.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        web_pub()
    except rospy.ROSInterruptException:
        pass
