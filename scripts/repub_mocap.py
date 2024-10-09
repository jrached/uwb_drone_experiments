#! /usr/bin/env python3
"""A script to republish motion capture data from a ROS topic to a different topic."""

import rospy
from geometry_msgs.msg import PoseStamped
from threading import Thread

# get the current namespace
NAMESPACE = rospy.get_namespace()


class MocapRepublisher:
    def __init__(self, pub_hz=30.0):
        self.mocap_sub_topic_name = rospy.get_param(
            "~mocap_sub_topic_name", "/optitrack" + NAMESPACE + "world"
            # "~mocap_sub_topic_name", NAMESPACE + "world"
        )
        self.mocap_pub_topic_name = rospy.get_param(
            "~mocap_pub_topic_name", NAMESPACE + "mavros/vision_pose/pose"
        )

        self._mocap_sub = rospy.Subscriber(
            self.mocap_sub_topic_name, PoseStamped, self._mocap_cb
        )
        self._last_msg = None

        # Publish at the specified rate in a separate thread
        self._pub_hz = pub_hz
        self._mocap_pub = rospy.Publisher(self.mocap_pub_topic_name, PoseStamped, queue_size=10)
        self._pub_thread = Thread(target=self._publish_loop, args=())
        self._pub_thread.daemon = True
        self._pub_thread.start()


    def _mocap_cb(self, msg):
        self._last_msg = msg

    def _publish_loop(self):
        rate = rospy.Rate(self._pub_hz)
        while not rospy.is_shutdown():
            if self._last_msg:
                self._mocap_pub.publish(self._last_msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mocap_republisher")
    MocapRepublisher(pub_hz=20.0)
    rospy.spin()
