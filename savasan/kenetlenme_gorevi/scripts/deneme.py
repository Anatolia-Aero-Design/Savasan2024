#!/usr/bin/env python3

from collections import defaultdict
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
import cv2
import numpy as np

from ultralytics import YOLO

# Run YOLOv8 tracking on the frame, persisting tracks between frames
  # Load the YOLOv8 model
model = YOLO('/home/valvarn/catkin_ws/src/savasan/kenetlenme_gorevi/models/best_n.pt')
video_path = "/home/valvarn/Downloads/test.mp4"
cap = cv2.VideoCapture(video_path)

# Store the track history
track_history = defaultdict(lambda: [])

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = model.track(frame, persist=True)

        # Get the boxes and track IDs
        annotated_frame = None

        if results and results[0].boxes.id is not None:
            boxes = results[0].boxes.xyxy.numpy().astype(int)
            track_ids = results[0].boxes.id.numpy().astype(int)

            # Visualize the results on the frame
            annotated_frame = results[0].plot()

        # Plot the tracks
        for box, track_id in zip(boxes, track_ids):
            x, y, w, h = box
            track = track_history[track_id]
            track.append((float(x), float(y)))  # x, y center point
            if len(track) > 30:  # retain 90 tracks for 90 frames
                track.pop(0)

            # Draw the tracking lines
            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

        # Display the annotated frame
        cv2.imshow("YOLOv8 Tracking", annotated_frame)
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
      
        
