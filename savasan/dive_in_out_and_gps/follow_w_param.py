#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandInt
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import PlaneControl
 
follow = False
leader_position = PlaneControl() 

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
    global leader_position
    leader_position = data
    if follow and rospy.get_param('~target_id') == leader_position.takim_numarasi:
        rospy.loginfo("Target Position: Lat %f, Lon %f, Alt %f", data.IHA_enlem, data.IHA_boylam, data.IHA_irtifa)
        target_latitude = leader_position.IHA_enlem
        target_longitude = leader_position.IHA_boylam
        target_altitude = leader_position.IHA_irtifa   
        send_goto_position(target_latitude, target_longitude, target_altitude) 
    else:
        rospy.loginfo("target_id doesn't match.")

def main():
    global follow
    rospy.init_node('gps_node')
    follow = rospy.get_param('~follow', False)
    rospy.loginfo("Follow: %s", follow)
    rospy.Subscriber('plane_coordinates_topic', PlaneControl, callback)
    rospy.Timer(rospy.Duration(1),callback)
    rospy.spin()

def callback(event):
    global follow
    follow = rospy.get_param('~follow', False)
    rospy.loginfo("follow is: %s", follow)

if __name__ == '__main__':
    main()
