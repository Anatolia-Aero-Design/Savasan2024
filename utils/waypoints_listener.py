#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList


# for checking the waypoints are published correctly
def waypoints_callback(data):
    rospy.loginfo("Received {} waypoints:".format(len(data.waypoints)))
    for i, waypoint in enumerate(data.waypoints):
        rospy.loginfo("Waypoint {}: lat={}, lon={}, alt={}".format(i+1, waypoint.x_lat, waypoint.y_long, waypoint.z_alt))

def listener():
    rospy.init_node('waypoints_listener', anonymous=True)
    rospy.Subscriber("/plane1/mavros/mission/waypoints", WaypointList, waypoints_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
