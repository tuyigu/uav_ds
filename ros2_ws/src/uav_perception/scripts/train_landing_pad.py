#!/usr/bin/env python3
"""
Train YOLOv8n to detect landing pads (ArUco markers).

Usage:
    python3 train_landing_pad.py

This script:
  1. Downloads a landing pad dataset from Roboflow
  2. Trains YOLOv8n on it (~10 min on GPU, ~30 min on CPU)
  3. Saves the best model to ./runs/detect/landing_pad/weights/best.pt
"""
from ultralytics import YOLO
from roboflow import Roboflow
import os
import yaml

# ============================================================
# Step 1: Download dataset from Roboflow
# ============================================================
# This dataset contains ~1000+ images of landing pads from drone view
# Free tier API key — get your own at https://roboflow.com if this expires

ROBOFLOW_API_KEY = os.environ.get("ROBOFLOW_API_KEY", "")
WORKSPACE = "keito-kohinata"
PROJECT = "aruco-8finj"
VERSION = 1

print("=" * 60)
print("Step 1: Downloading landing pad dataset from Roboflow...")
print("=" * 60)

if not ROBOFLOW_API_KEY:
    print("\n⚠️  需要 Roboflow API Key!")
    print("   1. 打开 https://roboflow.com 注册免费账号")
    print("   2. 进入 Settings → API Key 复制")
    print("   3. 运行: export ROBOFLOW_API_KEY='你的key'")
    print("   4. 重新运行本脚本")
    print("\n   或者你也可以手动下载数据集:")
    print("   https://universe.roboflow.com/mti805-jpcrl/mti805-finding-landing-pad")
    print("   下载 YOLOv8 格式，解压到 ./datasets/landing_pad/")
    exit(1)

rf = Roboflow(api_key=ROBOFLOW_API_KEY)
project = rf.workspace(WORKSPACE).project(PROJECT)
version = project.version(VERSION)

# Download in YOLOv8 format
dataset = version.download("yolov8", location="./datasets/landing_pad")

# ============================================================
# Step 2: Fix data.yaml paths (Roboflow sometimes uses absolute)
# ============================================================
data_yaml = os.path.join(dataset.location, "data.yaml")
print(f"\nDataset location: {dataset.location}")
print(f"data.yaml: {data_yaml}")

# Ensure paths are correct
with open(data_yaml, 'r') as f:
    data = yaml.safe_load(f)

data['path'] = os.path.abspath(dataset.location)
data['train'] = 'train/images'
data['val'] = 'valid/images'
data['test'] = 'test/images'

with open(data_yaml, 'w') as f:
    yaml.dump(data, f, default_flow_style=False)

print(f"Classes: {data.get('names', 'unknown')}")
print(f"Train/Val/Test paths fixed.")

# ============================================================
# Step 3: Train YOLOv8n
# ============================================================
print("\n" + "=" * 60)
print("Step 2: Training YOLOv8n...")
print("  Model: yolov8n.pt (nano, 6MB)")
print("  Epochs: 50")
print("  Image size: 640")
print("=" * 60)

model = YOLO('yolov8n.pt')  # Pre-trained nano model

results = model.train(
    data=data_yaml,
    epochs=50,
    imgsz=640,
    batch=16,
    name='landing_pad',
    patience=10,       # Early stopping
    save=True,
    device='0' if os.path.exists('/dev/nvidia0') or os.environ.get('CUDA_VISIBLE_DEVICES') else 'cpu',
    workers=4,
    verbose=True,
)

# ============================================================
# Step 4: Copy best model to perception config
# ============================================================
best_pt = os.path.join('runs', 'detect', 'landing_pad', 'weights', 'best.pt')
target_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
os.makedirs(target_dir, exist_ok=True)
target_path = os.path.join(target_dir, 'landing_pad_yolov8n.pt')

if os.path.exists(best_pt):
    import shutil
    shutil.copy2(best_pt, target_path)
    print(f"\n✅ 模型已保存到: {os.path.abspath(target_path)}")
    print(f"   文件大小: {os.path.getsize(target_path) / 1e6:.1f} MB")
else:
    print(f"\n⚠️  找不到 {best_pt}，请手动查找 runs/ 目录")

# ============================================================
# Step 5: Quick validation
# ============================================================
print("\n" + "=" * 60)
print("Step 3: Validation...")
print("=" * 60)

val_model = YOLO(best_pt if os.path.exists(best_pt) else target_path)
metrics = val_model.val(data=data_yaml)

print(f"\n📊 Results:")
print(f"   mAP50:    {metrics.box.map50:.3f}")
print(f"   mAP50-95: {metrics.box.map:.3f}")
print(f"   Precision: {metrics.box.mp:.3f}")
print(f"   Recall:    {metrics.box.mr:.3f}")

print(f"\n🚀 Done! 使用方法:")
print(f"   ros2 run uav_perception yolo_detector --ros-args \\")
print(f"     -p model_path:={os.path.abspath(target_path)} \\")
print(f"     -p target_class:=landing_pad \\")
print(f"     -p confidence_threshold:=0.5")
