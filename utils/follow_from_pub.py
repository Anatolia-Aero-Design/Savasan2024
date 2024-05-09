#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PlaneControl

leader_position = NavSatFix() # creating msg object to reach attributes

def send_goto_position(target_latitude, target_longitude, target_altitude): # func for sending target coordinates to vehicle
    rospy.wait_for_service('/plane1/mavros/cmd/command_int') # waiting for the service to send commands to plane
    try:
        command_int = rospy.ServiceProxy('/plane1/mavros/cmd/command_int', CommandInt) # after service is on creating service obj to use functions of server
        response = command_int( # creating comman_int obj to send command to vehicle (above service)
            # we can add target_system and target_component to the beginning 
            frame = 3,  # the coordinate system of the component
            command = 192,   # command id
            current = 2,  #not used
            autocontinue = 0, #not used
            param1 = 30,  # param1 = ground speed for command 192
            param2 = 0,  # param2 = Bitmask of option flags.
            param3 = 0,  # param3 = Loiter radius for planes.
            param4 = 0,  #param4 = Yaw heading. 0 clockwise 1 counter clockwise
            x = int(target_latitude * 1e7),  #param 5 = target latitude
            y = int(target_longitude * 1e7), # param 6 = target longitude
            z = target_altitude  # param 7 = target altitude
        )
        rospy.loginfo("Position command sent")
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s"%e)

def callback(data):
    global leader_position
    leader_position = data
    rospy.loginfo("Leader Position: Lat %f, Lon %f, Alt %f", data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)
    target_latitude = leader_position.IHA_enlem # sending coordinates to vehicle
    target_longitude = leader_position.IHA_boylam
    target_altitude = leader_position.IHA_irtifa - 5.0  # in meters
    send_goto_position(target_latitude, target_longitude, target_altitude)

if __name__ == '__main__':
    rospy.init_node('send_goto_position_node') # initializing node
    rospy.Subscriber('plane_coordinates_topic', PlaneControl ,callback) #subscribing to plane position topic to follow it
    rospy.spin()  # Keeps the node running until shutdown
