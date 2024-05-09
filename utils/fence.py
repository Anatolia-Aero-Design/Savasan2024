#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList
from mavros_msgs.srv import WaypointClear, WaypointPush
from mavros_msgs.msg import CommandCode, Waypoint, WaypointReached
from mavros_msgs.srv import SetMode, CommandBool
from geographic_msgs.msg import GeoPoint

class FenceExclusion:
    def __init__(self):
        rospy.init_node('fence_exclusion', anonymous=True)

        # Set up ROS subscribers and publishers
        self.waypoint_push_srv = rospy.ServiceProxy('/plane1/mavros/mission/push', WaypointPush)
        self.waypoint_clear_srv = rospy.ServiceProxy('/plane1/mavros/mission/clear', WaypointClear)

        # Set fence exclusion parameters
        self.center_lat = 47.6062  # Example latitude
        self.center_lon = -122.3321  # Example longitude
        self.radius_meters = 100  # Example radius in meters

    def clear_waypoints(self):
        # Clear existing waypoints
        try:
            self.waypoint_clear_srv.call().wp_clear
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def push_waypoint(self, waypoint):
        # Push a single waypoint to the flight controller
        try:
            self.waypoint_push_srv.call(0, [waypoint])
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def create_fence_exclusion(self):
        # Clear existing waypoints
        self.clear_waypoints()

        # Create fence exclusion waypoint
        waypoint = Waypoint()
        waypoint.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        waypoint.command = 5004
        waypoint.is_current = False
        waypoint.autocontinue = True
        waypoint.param1 = self.center_lat  # Latitude of circle center
        waypoint.param2 = self.center_lon  # Longitude of circle center
        waypoint.param3 = self.radius_meters  # Radius of exclusion circle
        waypoint.param4 = 0  # Unused parameter
        waypoint.x_lat = 0  # Unused parameter
        waypoint.y_long = 0  # Unused parameter
        waypoint.z_alt = 0  # Unused parameter

        # Push the fence exclusion waypoint
        self.push_waypoint(waypoint)

if __name__ == '__main__':
    fence_exclusion = FenceExclusion()
    fence_exclusion.create_fence_exclusion()
