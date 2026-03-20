# Import YOLO model from ultralytics library
from ultralytics import YOLO
import os

# ===================== Core Configuration =====================
# Path to the best trained model weights
model_path = "runs/detect/train2/weights/best.pt"
# 替换成查到的第一个真实图片名（关键！）
test_img_path = "test/images/-_20251215171952_80_58_jpg.rf.7c75671e7b7ad5d73e89492a46dd2c00.jpg"
# Confidence threshold (only show detections with confidence ≥ 0.5)
conf_threshold = 0.5

# ===================== Model Inference =====================
# Load the best model weights
model = YOLO(model_path)

# 关闭弹窗，只保存结果（适配服务器环境）
results = model(test_img_path, conf=conf_threshold, show=False, save=True)

# ===================== Result Processing =====================
# Print detection completion message
save_dir = results[0].save_dir
print(f"Detection completed! Result saved to: {os.path.abspath(save_dir)}")

# Print detailed detection information (class name + confidence)
for box in results[0].boxes:
    class_name = results[0].names[int(box.cls)]
    confidence = box.conf.item()
    print(f"Detected: {class_name} | Confidence: {confidence:.2f}")

# Print bounding box coordinates (x1, y1, x2, y2)
for box in results[0].boxes:
    bbox_coords = box.xyxy.tolist()[0]
    print(f"Bounding box for {class_name}: {[round(coord, 2) for coord in bbox_coords]}")

