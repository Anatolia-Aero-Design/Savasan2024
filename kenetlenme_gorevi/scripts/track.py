from collections import defaultdict
import rospy
import cv2
import numpy as np

from ultralytics import YOLO

def tracker(MODEL):
    model = YOLO(MODEL) # Load the YOLOv8 model
    rospy.init_node('camera_publisher', anonymous=True)
    # Load camera index from parameter server
    camera_index = rospy.get_param('~camera_index', 0)  # Default index is 0
    
    # capture published image 
    cap = cv2.VideoCapture(camera_index)

    # Store the track history
    track_history = defaultdict(lambda: [])

    # Loop through the video frames
    while cap.isOpened():
        # Read a frame from the video
        success, frame = cap.read()

        if success:
            # Run YOLOv8 tracking on the frame, persisting tracks between frames
            results = model.track(frame, persist=True)
            
            if results[0].boxes.id != None:
                boxes = results[0].boxes.xyxy.cpu().numpy().astype(int)
                track_ids = results[0].boxes.id.cpu().numpy().astype(int)
                confidences = results[0].boxes.conf.cpu().numpy().astype(int)

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
        
        # Break the loop if the end of the video is reached
        else:
            break

if __name__ == '__main__':
    try:
        if rospy.get_param('/start_camera', True):
            tracker('/home/valvarn/savasan/kenetlenme_gorevi/models/best_n.pt')
        else:
            pass
    except rospy.ROSInterruptException:
        pass