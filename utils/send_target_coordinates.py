import rospy
from mavros_msgs.msg import GlobalPositionTarget

# publishing target coordinates on /plane1/mavros/setpoint_raw/global topic

def send_global_position_setpoint(latitude, longitude, altitude):
    rospy.init_node('global_position_setpoint_publisher')
    
    pub = rospy.Publisher('/plane1/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=10)

    msg = GlobalPositionTarget()
    msg.latitude = latitude
    msg.longitude = longitude
    msg.altitude = altitude
    msg.coordinate_frame = 6  # FRAME_GLOBAL_INT: global coordinate frame (Lat, Lon, Alt)

    pub.publish(msg) 

if __name__ == '__main__':
    # target coordinates
    latitude = -35.363605
    longitude = 149.165448
    altitude = 30.0
    while not rospy.is_shutdown():
        send_global_position_setpoint(latitude, longitude, altitude)
        rospy.sleep(1) 
