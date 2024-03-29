import rospy
from mavros_msgs.msg import WaypointList, Waypoint
from mavros_msgs.srv import WaypointPush, WaypointClear
import math

HOME_LATITUDE = -35.363212
HOME_LONGITUDE = 149.165210
HOME_ALTITUDE = 100.0

ROAD_DISTANCE = 300  # Distance between road waypoints in meters
TARGET_LATITUDE = -35.363276
TARGET_LONGITUDE = 149.165283
TARGET_ALTITUDE = 15.0  

def calculate_waypoints_with_angle(target_lat, target_lon, angle_degrees):
    R = 6371000  # Radius of the Earth

    # Convert latitude and longitude from degrees to radians
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)

    # Calculate the distance in radians for 100 meters
    delta_lon = math.atan2(ROAD_DISTANCE, R * math.cos(target_lat_rad))

    # Calculate the first waypoint (left) with the specified angle
    wp1_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(ROAD_DISTANCE / R) +
                    math.cos(target_lat_rad) * math.sin(ROAD_DISTANCE / R) * math.cos(math.radians(angle_degrees)))
    wp1_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin(ROAD_DISTANCE / R) * math.cos(target_lat_rad),
                    math.cos(ROAD_DISTANCE / R) - math.sin(target_lat_rad) * math.sin(wp1_lat_rad))

    # Calculate the second waypoint (right) with the specified angle
    wp2_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(ROAD_DISTANCE / R) +
                    math.cos(target_lat_rad) * math.sin(ROAD_DISTANCE / R) * math.cos(math.radians(angle_degrees - 90)))
    wp2_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees - 90)) * math.sin(ROAD_DISTANCE / R) * math.cos(target_lat_rad),
                    math.cos(ROAD_DISTANCE / R) - math.sin(target_lat_rad) * math.sin(wp2_lat_rad))

    # Calculate the leaving waypoint (same latitude as target, different longitude)
    wp5_lat_rad = target_lat_rad
    wp5_lon_rad = target_lon_rad + delta_lon  # Adjusted longitude

    return [(math.degrees(wp1_lat_rad), math.degrees(wp1_lon_rad)),
            (math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad)),
            (math.degrees(target_lat_rad), math.degrees(target_lon_rad)),
            (math.degrees(wp5_lat_rad), math.degrees(wp5_lon_rad))]

def push_waypoints(waypoints):
    rospy.wait_for_service('/plane1/mavros/mission/push')
    clear_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/clear', WaypointClear)
    push_waypoints_service = rospy.ServiceProxy('/plane1/mavros/mission/push', WaypointPush)
    try:
        response = clear_waypoints_service()
        response = push_waypoints_service(0, waypoints.waypoints)  # Pass waypoints.waypoints instead of waypoints
        return response.wp_transfered
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

# Modify waypoints_with_angle() function to accept an angle_degrees parameter
def waypoints_with_angle(angle_degrees=0):
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
    wp1.x_lat = HOME_LATITUDE
    wp1.y_long = HOME_LONGITUDE
    wp1.z_alt = HOME_ALTITUDE
    waypoints.waypoints.append(wp1)

    # Calculate waypoints with the specified angle
    calculated_waypoints = calculate_waypoints_with_angle(TARGET_LATITUDE, TARGET_LONGITUDE, angle_degrees)

    # Existing code...
    # Waypoint 2: Road waypoint 1
    wp2 = Waypoint()
    wp2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp2.command = 16
    wp2.is_current = False  
    wp2.autocontinue = True
    wp2.param1 = 0
    wp2.param2 = 0
    wp2.param3 = 0
    wp2.x_lat = calculated_waypoints[0][0]
    wp2.y_long = calculated_waypoints[0][1]
    wp2.z_alt = HOME_ALTITUDE
    waypoints.waypoints.append(wp2)

    # Waypoint 3: Road waypoint 2
    wp3 = Waypoint()
    wp3.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp3.command = 16
    wp3.is_current = False  
    wp3.autocontinue = True
    wp3.param1 = 0
    wp3.param2 = 0
    wp3.param3 = 0
    wp3.x_lat = calculated_waypoints[1][0]
    wp3.y_long = calculated_waypoints[1][1]
    wp3.z_alt = HOME_ALTITUDE
    waypoints.waypoints.append(wp3)

    # Waypoint 4: Target coordinates and target altitude
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

    # Waypoint 5: Leave the target coordinates
    wp5 = Waypoint()
    wp5.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    wp5.command = 16
    wp5.is_current = False 
    wp5.autocontinue = True
    wp5.param1 = 0
    wp5.param2 = 0
    wp5.param3 = 0
    wp5.x_lat = calculated_waypoints[3][0]
    wp5.y_long = calculated_waypoints[3][1]
    wp5.z_alt = HOME_ALTITUDE
    waypoints.waypoints.append(wp5)

    # Push waypoints to the flight controller
    success = push_waypoints(waypoints)
    if success:
        rospy.loginfo("Waypoints pushed successfully!")
    else:
        rospy.logerr("Failed to push waypoints!")
        
    rospy.spin()
if __name__ == '__main__':
    waypoints_with_angle()
    