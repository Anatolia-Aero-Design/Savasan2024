#!/usr/bin/env python3

from collections import defaultdict
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np

from ultralytics import YOLO

# Define the callback function to process the received image
def image_callback(msg): # TODO Yaw, Pitch, Roll will be calculated
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(cv_image, persist=True)
        
        # Initialize annotated_frame
        annotated_frame = None

        if results and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.numpy().astype(int)
            track_ids = results[0].boxes.id.numpy().astype(int)

            # Visualize the results on the frame
            annotated_frame = results[0].plot()
            
            # Plot the tracks
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                annotated_frame = cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 0, 255), 2)  # Red color (BGR: 0, 0, 255)
                track = track_history[track_id]
                center = track.append((int((x+w)//2), int((y+h)//2)))  #center point of bounding box
                if len(track) > 30:  # retain 90 tracks for 90 frames
                    track.pop(0)

        if annotated_frame is not None:
            # Convert annotated frame back to ROS Image message
            annotated_frame_msg = CvBridge().cv2_to_imgmsg(annotated_frame, encoding="bgr8")

            # Publish the annotated frame
            annotated_frame_publisher.publish(annotated_frame_msg)
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

if __name__ == '__main__':
    try:
        rospy.init_node('track', anonymous=True)
        
        # Load the YOLOv8 model
        model = YOLO('/home/valvarn/catkin_ws/src/savasan/kenetlenme_gorevi/models/best_n.pt')
        
        # Store the track history
        track_history = defaultdict(lambda: [])

        # Define the topic to subscribe to
        image_topic = "camera/image"
        
        # Subscribe to the image topic
        rospy.Subscriber(image_topic, Image, image_callback)
        

        # Define the topic to publish the annotated frame
        annotated_frame_topic = "annotated_image"
        annotated_frame_publisher = rospy.Publisher(annotated_frame_topic, Image, queue_size=10)

        # Spin to keep the node running until shut down
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
