#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge 
from pyzbar import pyzbar
import webbrowser
import datetime

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

    rate = rospy.Rate(30)  # Adjust the rate if needed
    
    qr_scanned = False # Control condition for scanning QR just once
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # Convert OpenCV image to ROS image and publish
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)

            # Find and decode QR codes in the frame
            barcodes = pyzbar.decode(frame)
            for barcode in barcodes:
                
                # Extract QR code data
                qr_data = barcode.data.decode("utf-8")
   
                # Process QR code data (e.g., open URL)
                if qr_data.startswith("http") and not qr_scanned:
                    webbrowser.open(qr_data)
                    qr_scanned = True
                elif qr_scanned:
                    # Get the current system time
                    current_time = datetime.datetime.now()
                    print(str(current_time) + " QR Scanned")
                else:
                    print("QR Code Text:", qr_data)

        rate.sleep()

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
