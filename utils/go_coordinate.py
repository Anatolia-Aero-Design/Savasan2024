#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix

# go to coordinate published from send target coordinates

def send_goto_position(target_latitude, target_longitude, target_altitude):
    rospy.wait_for_service('/plane1/mavros/cmd/command_int')
    try:
        command_int = rospy.ServiceProxy('/plane1/mavros/cmd/command_int', CommandInt)
        response = command_int(
            frame = 3,  
            command = 192,  
            current = 2,  
            autocontinue = 0, 
            param1 = 30,  
            param2 = 0,  
            param3 = 0,
            param4 = 0, 
            x = int(target_latitude * 1e7),  
            y = int(target_longitude * 1e7),
            z = target_altitude  
        )
        rospy.loginfo("Position command sent")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def callback(data):
    rospy.loginfo("Leader Position: Lat %f, Lon %f, Alt %f", data.latitude, data.longitude, data.altitude)

if __name__ == '__main__':
    rospy.init_node('send_goto_position_node')
    rospy.Subscriber('/plane2/mavros/global_position/global', NavSatFix,callback)
    leader_position = NavSatFix()
    # Example usage
    target_latitude = -35.360119
    target_longitude = 149.159641
    target_altitude = 100.0   # in meters
    send_goto_position(target_latitude, target_longitude, target_altitude)