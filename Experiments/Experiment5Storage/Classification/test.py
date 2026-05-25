import os
import time

import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torchvision.models import ResNet18_Weights, resnet18

# --- PATHS ---
script_dir = os.path.dirname(os.path.abspath(__file__))
test_dir = os.path.join(script_dir, "Test_Data")

model_path = os.path.join(script_dir, "neural_network_figures", "optimized_resnet18.pth")

# --- DEVICE ---
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# --- SAME TRANSFORMS AS TRAINING ---
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

test_transform = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), normalize])

# --- LOAD TEST DATA ---
test_set = datasets.ImageFolder(test_dir, test_transform)

# Batch size = 1 because we want image-by-image timing
test_loader = DataLoader(test_set, batch_size=1, shuffle=False)

class_names = test_set.classes
num_classes = len(class_names)

# ----------------------------------------------------
# REBUILD MODEL EXACTLY AS USED DURING TRAINING
# ----------------------------------------------------

# IMPORTANT:
# Replace this with your Optuna best dropout value
BEST_DROPOUT = 0.5

model = resnet18(weights=ResNet18_Weights.DEFAULT)

for param in model.parameters():
    param.requires_grad = False

model.fc = nn.Sequential(nn.Dropout(BEST_DROPOUT), nn.Linear(model.fc.in_features, num_classes))

# Load trained weights
model.load_state_dict(torch.load(model_path, map_location=device))

model = model.to(device)
model.eval()

print("Model loaded successfully")

# ----------------------------------------------------
# GPU WARMUP
# ----------------------------------------------------

print("Running warmup...")

dummy = torch.randn(1, 3, 224, 224).to(device)

with torch.no_grad():
    for _ in range(20):
        _ = model(dummy)

if device.type == "cuda":
    torch.cuda.synchronize()

# ----------------------------------------------------
# TIMING
# ----------------------------------------------------

print("\nTiming inference...")

individual_times = []

with torch.no_grad():
    total_start = time.perf_counter()

    for image, _ in test_loader:
        image = image.to(device)

        if device.type == "cuda":
            torch.cuda.synchronize()

        start = time.perf_counter()

        _ = model(image)

        if device.type == "cuda":
            torch.cuda.synchronize()

        end = time.perf_counter()

        individual_times.append(end - start)

    total_end = time.perf_counter()

# ----------------------------------------------------
# RESULTS
# ----------------------------------------------------

avg_time = sum(individual_times) / len(individual_times)

print("\n========== RESULTS ==========")
print(f"Images tested: {len(test_set)}")

print(f"Average inference time/image: {avg_time * 1000:.3f} ms")

print(f"Images per second: {1 / avg_time:.2f}")

print(f"Total test set time: {total_end - total_start:.3f} sec")
