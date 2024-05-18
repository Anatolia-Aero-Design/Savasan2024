#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge as bridge
from pyzbar import pyzbar
import webbrowser
import datetime

def qr_code_scanner():
    rospy.init_node('qr_code_scanner', anonymous=True)

    # initialize the video capture
    print("[INFO] starting video stream...")
    cap = cv2.VideoCapture("camera/image")  # Use the default camera (index 0)

    # check if the video capture is successful
    if not cap.isOpened():
        print("Error: Unable to open video capture.")
        exit()

    qr_scanned = False # Control condition for scanning QR just once

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # Convert OpenCV image to ROS image and publish
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
        
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

        # show the output frame
        cv2.imshow("Barcode Scanner", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

    # release the video capture and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    qr_code_scanner()
