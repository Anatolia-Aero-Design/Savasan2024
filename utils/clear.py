#!/usr/bin/env python

import rospy
from mavros_msgs.srv import WaypointClear

def clear_waypoints():
    rospy.wait_for_service('/plane1/mavros/mission/clear')
    try:
        clear_service = rospy.ServiceProxy('/plane1/mavros/mission/clear', WaypointClear)
        response = clear_service()
        if response.success:
            rospy.loginfo("Waypoints cleared successfully!")
            return True
        else:
            rospy.logwarn("Failed to clear waypoints!")
            return False
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

def main():
    rospy.init_node('waypoint_clearer', anonymous=True)
    success = clear_waypoints()
    if not success:
        rospy.logerr("Failed to clear waypoints!")
    rospy.spin()

if __name__ == '__main__':
    main()
