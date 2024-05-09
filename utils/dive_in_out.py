#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandLong,CommandInt
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix
import time

def dive(target_altitude): #unsupported?????????
    rospy.wait_for_service('/plane1/mavros/cmd/command_int') 
    try:
        command_int = rospy.ServiceProxy('/plane1/mavros/cmd/command_int', CommandInt) 
        response = command_int(  
            frame = 3,  
            command = 30,  
            current = 2, 
            autocontinue = 0, 
            param1 = target_altitude,  
            param2 = 0,  
            param3 = 0,  
            param4 = 0,  
            x = 0,  
            y = 0,  
            z = 0  
        )
        rospy.loginfo("Position command sent")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def send_goto_position(target_latitude, target_longitude, target_altitude):
    rospy.wait_for_service('/plane1/mavros/cmd/command')
    try:
        command_long = rospy.ServiceProxy('/plane1/mavros/cmd/command', CommandLong)
        response = command_long(
            broadcast=False,
            command=192,  
            confirmation=0,
            param1=30,  
            param2=0,   
            param3=0,   
            param4=0,   
            param5=int(target_latitude * 1e7),  
            param6=int(target_longitude * 1e7), 
            param7=target_altitude  
        )
        rospy.loginfo("Position command sent")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('altitude_change_node')
    target_latitude = -35.360119
    target_longitude = 149.159641
    target_altitude = 30.0  
    dive(target_altitude)
    time.sleep(10)
    send_goto_position(target_latitude, target_longitude, target_altitude)
