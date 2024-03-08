from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO('/home/valvarn/savasan/kenetlenme_gorevi/models/best_n.pt')

# Define path to video file
source = '/home/valvarn/Downloads/test.mp4'

# Run inference on the source
results = model(source, show=True)  # generator of Results objects