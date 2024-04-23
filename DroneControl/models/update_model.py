from ultralytics import YOLO

model = YOLO('yolov5nu.pt')

# Export the model to ONNX format
model.export(format='onnx', imgsz=224, optimize=True, int8=True, simplify=True)