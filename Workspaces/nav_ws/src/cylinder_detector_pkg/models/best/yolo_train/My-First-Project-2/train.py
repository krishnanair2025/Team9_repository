from ultralytics import YOLO

# 改用 YOLOv8s 预训练模型（自动下载，更稳定）
model = YOLO('yolov8s.pt')

# 开始训练（参数不变）
results = model.train(
    data='data.yaml',    # 当前目录的数据集配置
    epochs=100,          # 训练轮数
    batch=8,             # CPU 适配的批次大小
    imgsz=640,           # 图片尺寸
    device='cpu',        # CPU 训练
    patience=50,         # 早停策略
    save=True            # 自动保存 best.pt
)

# 输出 best.pt 路径
print("训练完成！best.pt 路径：", results.save_dir)

