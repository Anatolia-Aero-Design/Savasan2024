from ultralytics import YOLO

# Load a model
model = YOLO('/home/valvarn/Savasan2024/kenetlenme_gorevi/models/best_n.pt')

# Export the model
model.export(format='engine') 