#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
 
def cb(msg):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = (- msg.axes[5] + 1) / 2
    cmd_vel_msg.linear.y = msg.axes[0]
    if msg.buttons[0]:
        cmd_vel_msg.linear.x = - cmd_vel_msg.linear.x
        cmd_vel_msg.linear.y = - cmd_vel_msg.linear.y
    print("linear x = {}, linear y = {}".format(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y))
    pub.publish(cmd_vel_msg)


def ds4_cmdvel():
    rospy.Subscriber("joy", Joy, cb)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("ds4_cmdvel", anonymous=True)
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    msg = Twist()
    r = rospy.Rate(10)
    ds4_cmdvel()