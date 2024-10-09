#! /usr/bin/env python3

import rospy
import math
from threading import Thread
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from mavros import mavlink
from mavros_msgs.msg import (
    Altitude,
    ExtendedState,
    HomePosition,
    ParamValue,
    State,
    WaypointList,
    Waypoint,
    Mavlink,
    CommandCode,
)
from mavros_msgs.srv import (
    CommandBool,
    ParamGet,
    ParamSet,
    SetMode,
    SetModeRequest,
    WaypointClear,
    WaypointPush,
)
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import NavSatFix, Imu
from pymavlink import mavutil

from typing import List, Tuple

from six.moves import xrange

from base_mavros_interface import BasicMavrosInterface

LOCAL_NAVIGATION = 0  # x/y/z relative to home position
GLOBAL_NAVIGATION = 1  # lat/lon/alt
NAVIGATION_MODES = [LOCAL_NAVIGATION, GLOBAL_NAVIGATION]


class OffboardPathFollower(BasicMavrosInterface):
    def __init__(
        self,
        node_name: str = "offboard_path_follower",
        navigation_mode: int = LOCAL_NAVIGATION,
    ):
        super().__init__(node_name=node_name)
        self.navigation_mode = navigation_mode
        assert (
            self.navigation_mode in NAVIGATION_MODES
        ), f"Invalid navigation mode: {self.navigation_mode}"

        self.current_setpoint = None
        self.setpoint_publish_thread = Thread(
            target=self._publish_current_setpoint, args=()
        )
        self.setpoint_publish_thread.daemon = True
        self.setpoint_publish_thread.start()

        # set up capacity to listen for custom setpoints
        self.outside_setpoint_sub = rospy.Subscriber(
            "offboard/setpoint", PoseStamped, self._outside_setpoint_callback
        )
        self.received_outside_setpoint = False

    def _outside_setpoint_callback(self, msg: PoseStamped):
        self.current_setpoint = msg
        self.received_outside_setpoint = True

    def track_setpoints(self, setpoints: List[PoseStamped]):
        # This mode requires position or pose/attitude information - e.g. GPS, optical flow, visual-inertial odometry, mocap, etc.
        # RC control is disabled except to change modes (you can also fly without any manual controller at all by setting the parameter COM_RC_IN_MODE to 4: Stick input disabled).
        # The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
        # The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.
        # Not all coordinate frames and field values allowed by MAVLink are supported for all setpoint messages and vehicles. Read the sections below carefully to ensure only supported values are used.

        """NOTE
        -The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
        -The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.
        -Not all coordinate frames and field values allowed by MAVLink are supported for all setpoint messages and vehicles. Read the sections below carefully to ensure only supported values are used.
        """
        if self.received_outside_setpoint:
            rospy.logerr("Received outside setpoint. Ignoring track_setpoints command")
            return

        cur_setpoint_idx = 0
        self.current_setpoint = setpoints[cur_setpoint_idx]

        # wait 1 second for FCU connection
        rospy.sleep(1)

        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            # if 1 second has passed, move to the next setpoint
            # if (rospy.Time.now() - last_time).to_sec() > 1:
            #     last_time = rospy.Time.now()
            #     cur_setpoint_idx = (cur_setpoint_idx + 1) % len(setpoints)

            # if we've reached the current setpoint, move to the next one,
            # looping back to the first one if necessary
            if self.setpoint_reached(setpoints[cur_setpoint_idx]):
                cur_setpoint_idx = (cur_setpoint_idx + 1) % len(setpoints)

            self.current_setpoint = setpoints[cur_setpoint_idx]
            # rospy.loginfo(f"Current setpoint: {self.current_setpoint}")

            rospy.sleep(0.2)


    def _publish_current_setpoint(self):
        """Publishes the current setpoint"""
        setpoint_publish_rate = 20  # Hz
        rate = rospy.Rate(setpoint_publish_rate)
        while not rospy.is_shutdown():
            if (
                self.navigation_mode == LOCAL_NAVIGATION
                and self.current_setpoint is not None
            ):
                self.setpoint_position_pub.publish(self.current_setpoint)
            rate.sleep()

    def _pack_into_waypoints(
        self, points: List[Tuple[float, float, float]]
    ) -> List[Waypoint]:
        """Takes a list of points (represented as 3-tuples) and packs them into a list of Waypoints

        Args:
            waypoints (List[Tuple[float]]): list of points to pack as

        Returns:
            List[Waypoint]: list of Waypoints
        """
        if self.navigation_mode == LOCAL_NAVIGATION:
            frame = Waypoint.FRAME_LOCAL_ENU
        elif self.navigation_mode == GLOBAL_NAVIGATION:
            frame = Waypoint.FRAME_GLOBAL_REL_ALT
        else:
            raise ValueError(f"Invalid navigation mode: {self.navigation_mode}")

        waypoints = [
            Waypoint(
                frame=frame,
                command=CommandCode.NAV_WAYPOINT,
                is_current=False,
                autocontinue=True,
                param1=0,
                param2=0,
                param3=0,
                param4=0,
                x_lat=waypoint[0],
                y_long=waypoint[1],
                z_alt=waypoint[2],
            )
            for waypoint in points
        ]

        return waypoints

    def _pack_into_setpoints(
        self, points: List[Tuple[float, float, float]]
    ) -> List[PoseStamped]:
        assert self.navigation_mode == LOCAL_NAVIGATION, (
            f"Invalid navigation mode: {self.navigation_mode}. "
            f"Only local navigation is supported for this method"
        )

        setpoints = [
            PoseStamped(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id="map",
                ),
                pose=Pose(
                    position=Point(
                        x=point[0],
                        y=point[1],
                        z=point[2],
                    ),
                    orientation=Quaternion(
                        x=0,
                        y=0,
                        z=0,
                        w=1,
                    ),
                ),
            )
            for point in points
        ]

        return setpoints


if __name__ == "__main__":
    node_name = "offboard_path_follower"
    path_follower = OffboardPathFollower(
        node_name=node_name, navigation_mode=LOCAL_NAVIGATION
    )
    rospy.spin()

