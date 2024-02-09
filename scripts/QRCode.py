#!/usr/bin/env python3
import rospy 
from pyzbar import pyzbar
import cv2
import webbrowser

def qr_code_scanner():
    rospy.init_node('qr_code_scanner', anonymous=True)

    # initialize the video capture
    print("[INFO] starting video stream...")
    cap = cv2.VideoCapture(0)  # Use the default camera (index 0)

    # check if the video capture is successful
    if not cap.isOpened():
        print("Error: Unable to open video capture.")
        exit()

    # loop over the frames from the video capture
    while not rospy.is_shutdown():
        # grab the frame from the video capture
        ret, frame = cap.read()
        
        # check if the frame is valid
        if not ret:
            print("Error: Unable to capture frame from video.")
            break

        # find the barcodes in the frame and decode each of the barcodes
        barcodes = pyzbar.decode(frame)

        # loop over the detected barcodes
        for barcode in barcodes:
            # extract the barcode data
            barcodeData = barcode.data.decode("utf-8")

            # if the barcode data starts with http, it's likely a URL
            if barcodeData.startswith("http"):
                # open the URL in a web browser
                webbrowser.open(barcodeData)
            else:
                # display the text content of the QR code
                print("QR Code Text:", barcodeData)

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
