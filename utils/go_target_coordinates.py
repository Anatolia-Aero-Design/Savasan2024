#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import GlobalPositionTarget

# get target coordinates and go there

def send_goto_position(target_latitude, target_longitude, target_altitude):
    # func for do reposition command
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
    rospy.loginfo("Target coordinates : Lat %f, Lon %f, Alt %f", data.latitude, data.longitude, data.altitude)  
    target_latitude = data.latitude
    target_longitude = data.longitude
    target_altitude = data.altitude   
    send_goto_position(target_latitude, target_longitude, target_altitude)
    
def callback2(data):
    rospy.loginfo("OUR Lat %s, Lon %s, Alt %s", data.latitude, data.longitude, data.altitude)
    

if __name__ == '__main__':
    rospy.init_node('send_goto_position_node')
    rospy.Subscriber('/plane1/mavros/setpoint_raw/global', GlobalPositionTarget,callback) # getting target coordinates from publisher we have written
    rospy.Subscriber('/plane1/mavros/global_position/global', NavSatFix, callback2) # getting our coordinates
    rospy.spin() # works till node is shut down