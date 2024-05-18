#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 

def camera_publisher():
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Load camera index from parameter server
    camera_index = rospy.get_param('~camera_index', 0)  # Default index is 0
    
    # Open the camera
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        rospy.logerr("Cannot open camera!")
        return

    # Initialize publisher
    image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
    
    
    bridge = CvBridge()

 #   rate = rospy.Rate(30)  # Adjust the rate if needed
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # Convert OpenCV image to ROS image and publish
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)

    # Release the camera and shutdown
    cap.release()
    rospy.loginfo("Camera Publisher Node Shutdown")

if __name__ == '__main__':
    try:
        if rospy.get_param('/start_camera', True):
            camera_publisher()
        else:
            pass
    except rospy.ROSInterruptException:
        pass
