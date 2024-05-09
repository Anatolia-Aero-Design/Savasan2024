#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PlaneControl

# FOLLOW PLANE FROM SERVER WITH PARAMETERS 

# variables 
team_number = 0
follow = False

leader_position = PlaneControl() # our message file 

def send_goto_position(target_latitude, target_longitude, target_altitude): # for sending command to the plane
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
    global leader_position, team_number
    # update team_number when received new data
    team_number = rospy.get_param('~team_number', leader_position.takim_numarasi) # team number is the parameter we entered
    leader_position = data
    rospy.loginfo("Target Position: Lat %f, Lon %f, Alt %f", data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)
    # check if follow is enabled and the team number matches
    if rospy.get_param('~team_number') == leader_position.takim_numarasi:
        if follow and leader_position.takim_numarasi == team_number: 
            target_latitude = leader_position.IHA_enlem
            target_longitude = leader_position.IHA_boylam
            target_altitude = leader_position.IHA_irtifa   
            send_goto_position(target_latitude, target_longitude, target_altitude) # send target teams coordinatess
        else:
            rospy.loginfo("Follow is disabled or team number doesn't match.")


def main():
    global team_number, follow
    rospy.init_node('send_goto_position_node')
    # initial values
    team_number = rospy.get_param('~team_number', 0)
    follow = rospy.get_param('~follow', False)
    rospy.loginfo("Team Number: %d", team_number)
    rospy.loginfo("Follow: %s", follow)
    rospy.Subscriber('plane_coordinates_topic', PlaneControl, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
