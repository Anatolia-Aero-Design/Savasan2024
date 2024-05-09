#!/usr/bin/env python

import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush,WaypointClear
# SET WAYPOINTS MANUALLY

# 1st wp 
START_LATITUDE = -35.363235 
START_LONGITUDE = 149.156825
START_ALTITUDE = 100.0  

# 2nd wp
ROAD_LATITUDE = -35.363237
ROAD_LONGITUDE = 149.160179
ROAD_ALTITUDE = 100.0  

# target coordinates
TARGET_LATITUDE = -35.363276
TARGET_LONGITUDE = 149.165283
TARGET_ALTITUDE = 15.0  

#wp4 for leaving
LEAVE_LATITUDE = -35.362733
LEAVE_LONGITUDE = 149.171008

def push_waypoints(waypoints): # pushing the wps to flight controller
    rospy.wait_for_service('/plane1/mavros/mission/push')
    clear_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/clear',WaypointClear)
    push_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/push', WaypointPush)
    try:
        response = clear_waypoints_service()
        response = push_waypoints_service(0, waypoints.waypoints) #expected tuple
        return response.wp_transfered
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

def main():
    rospy.init_node('waypoint_publisher', anonymous=True)
    
    # Define waypoints
    waypoints = WaypointList()

    # Waypoint 1: home position
    wp1 = Waypoint()
    wp1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp1.command = 16
    wp1.is_current = True  
    wp1.autocontinue = True
    wp1.param1 = 0
    wp1.param2 = 0
    wp1.param3 = 0
    wp1.x_lat = START_LATITUDE
    wp1.y_long = START_LONGITUDE
    wp1.z_alt = START_ALTITUDE
    waypoints.waypoints.append(wp1)
    
    #starting wp
    wp2 = Waypoint()
    wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp2.command = 16
    wp2.is_current = False  
    wp2.autocontinue = True
    wp2.param1 = 0
    wp2.param2 = 0
    wp2.param3 = 0
    wp2.x_lat = START_LATITUDE
    wp2.y_long = START_LONGITUDE
    wp2.z_alt = START_ALTITUDE
    waypoints.waypoints.append(wp2)

    # Waypoint 2: Road for the third waypoint
    wp3 = Waypoint()
    wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp3.command = 16
    wp3.is_current = False  
    wp3.autocontinue = True
    wp3.param1 = 0
    wp3.param2 = 0
    wp3.param3 = 0
    wp3.x_lat = ROAD_LATITUDE
    wp3.y_long = ROAD_LONGITUDE
    wp3.z_alt = ROAD_ALTITUDE
    waypoints.waypoints.append(wp3)

    # Waypoint 3: Target coordinates 
    wp4 = Waypoint()
    wp4.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp4.command = 16
    wp4.is_current = False  
    wp4.autocontinue = True
    wp4.param1 = 0
    wp4.param2 = 0
    wp4.param3 = 0
    wp4.x_lat = TARGET_LATITUDE
    wp4.y_long = TARGET_LONGITUDE
    wp4.z_alt = TARGET_ALTITUDE
    waypoints.waypoints.append(wp4)

    # Waypoint 4: Leave the target coordinates 
    wp5 = Waypoint()
    wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp5.command = 16
    wp5.is_current = False 
    wp5.autocontinue = True
    wp5.param1 = 0
    wp5.param2 = 0
    wp5.param3 = 0
    wp5.x_lat = LEAVE_LATITUDE
    wp5.y_long = LEAVE_LONGITUDE
    wp5.z_alt = START_ALTITUDE
    waypoints.waypoints.append(wp5)
    
    

    # Push waypoints to the flight controller
    success = push_waypoints(waypoints)
    if success:
        rospy.loginfo("Waypoints pushed successfully!")
    else:
        rospy.logerr("Failed to push waypoints!")
        
    rospy.spin()

if __name__ == '__main__':
    main()
