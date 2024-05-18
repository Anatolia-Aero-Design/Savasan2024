from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO('/home/valvarn/catkin_ws/src/savasan/kenetlenme_gorevi/models/best_n.pt')

# Define path to video file
source = '/home/valvarn/Downloads/FoggyFPVHeaven.mp4'

# Run inference on the source
results = model.track(source, show=True)  # generator of Results objects