#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

agent_name = 'acl_alpha'

if __name__ == '__main__':
    rospy.init_node('spoofed_mocap')

    rate = rospy.Rate(30)
    pub = rospy.Publisher(f'/{agent_name}/world', PoseStamped, queue_size=10)

    msg = PoseStamped()
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    msg.pose.orientation.x = 0
    msg.pose.orientation.y = 0
    msg.pose.orientation.z = 0
    msg.pose.orientation.w = 1

    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

    rospy.spin()
