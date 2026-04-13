import copy
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.metrics import confusion_matrix
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torchvision.models import ResNet18_Weights, resnet18

# --- 1. SET UP PATHS & DEVICE ---
script_dir = os.path.dirname(os.path.abspath(__file__))
train_dir = os.path.join(script_dir, "Training_Data")
val_dir = os.path.join(script_dir, "Validation_Data")
test_dir = os.path.join(script_dir, "Test_Data")

# Automatically use the GPU if available, otherwise use CPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

batch_size = 32

# --- 2. DATA AUGMENTATION & PREPROCESSING ---
# PyTorch requires us to explicitly define how to transform the images
# ResNet18 expects images to be normalized with specific numbers
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

train_transforms = transforms.Compose(
    [
        transforms.RandomResizedCrop(224, scale=(0.8, 1.0)),  # Zoom/Crop
        transforms.RandomHorizontalFlip(),  # Flip
        transforms.RandomRotation(15),  # Rotate
        transforms.ToTensor(),  # Convert to PyTorch format
        normalize,  # Apply ResNet's required colors
    ]
)

# We DO NOT augment validation or test data, just resize and normalize it
test_transforms = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), normalize])

# --- 3. LOAD THE DATASETS ---
print("Loading datasets...")
train_data = datasets.ImageFolder(train_dir, transform=train_transforms)
val_data = datasets.ImageFolder(val_dir, transform=test_transforms)
test_data = datasets.ImageFolder(test_dir, transform=test_transforms)

class_names = train_data.classes
print(f"Classes found ({len(class_names)}):", class_names)

# DataLoaders handle the batching and shuffling
train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True)
val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False)
test_loader = DataLoader(test_data, batch_size=batch_size, shuffle=False)

# --- 4. BUILD THE TRANSFER LEARNING MODEL ---
print("Loading Pre-trained ResNet18...")

# Load the brain with the default optimal weights
model = resnet18(weights=ResNet18_Weights.DEFAULT)

# Freeze the brain so we don't destroy its knowledge
for param in model.parameters():
    param.requires_grad = False

# Replace the final layer (the "head") to match our 7 classes
# ResNet's final layer is called 'fc' (fully connected)
num_features = model.fc.in_features
model.fc = nn.Sequential(nn.Dropout(0.5), nn.Linear(num_features, len(class_names)))

# Move the model to the GPU/CPU
model = model.to(device)

# --- 5. COMPILE (LOSS & OPTIMIZER) ---
criterion = nn.CrossEntropyLoss()
# Notice we only pass model.fc.parameters() to the optimizer because the rest is frozen
optimizer = optim.Adam(model.fc.parameters(), lr=0.001)

# --- 6. THE TRAINING LOOP (WITH EARLY STOPPING) ---
print("Starting training...")
epochs = 100
patience = 15
best_val_loss = float("inf")
epochs_no_improve = 0
best_model_wts = copy.deepcopy(model.state_dict())

for epoch in range(epochs):
    # -- TRAINING PHASE --
    model.train()  # Tell PyTorch we are in training mode
    running_loss, running_corrects = 0.0, 0

    for inputs, labels in train_loader:
        inputs, labels = inputs.to(device), labels.to(device)

        optimizer.zero_grad()  # Clear old calculations
        outputs = model(inputs)  # Make predictions
        loss = criterion(outputs, labels)  # Calculate mistakes
        loss.backward()  # Calculate how to fix mistakes
        optimizer.step()  # Update the weights

        _, preds = torch.max(outputs, 1)
        running_loss += loss.item() * inputs.size(0)
        running_corrects += torch.sum(preds == labels.data)

    train_loss = running_loss / len(train_data)
    train_acc = running_corrects.double() / len(train_data)

    # -- VALIDATION PHASE --
    model.eval()  # Tell PyTorch we are in evaluation mode (turns off Dropout)
    val_loss, val_corrects = 0.0, 0

    # Don't calculate gradients during validation (saves memory/time)
    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            loss = criterion(outputs, labels)

            _, preds = torch.max(outputs, 1)
            val_loss += loss.item() * inputs.size(0)
            val_corrects += torch.sum(preds == labels.data)

    val_loss = val_loss / len(val_data)
    val_acc = val_corrects.double() / len(val_data)

    print(f"Epoch {epoch + 1}/{epochs} - Train Loss: {train_loss:.4f} Acc: {train_acc:.4f} | Val Loss: {val_loss:.4f} Acc: {val_acc:.4f}")

    # -- EARLY STOPPING CHECK --
    if val_loss < best_val_loss:
        best_val_loss = val_loss
        best_model_wts = copy.deepcopy(model.state_dict())
        epochs_no_improve = 0
    else:
        epochs_no_improve += 1
        if epochs_no_improve >= patience:
            print("Early stopping triggered!")
            break

# Load the best weights back into the model
model.load_state_dict(best_model_wts)

# --- 7. EVALUATE ON TEST SET & TRACK PREDICTIONS ---
print("\nEvaluating on test data...")
model.eval()
test_corrects = 0

# Track these for the confusion matrix
y_true = []
y_pred = []

# Start the stopwatch
start_time = time.perf_counter()

with torch.no_grad():
    for inputs, labels in test_loader:
        inputs, labels = inputs.to(device), labels.to(device)
        outputs = model(inputs)
        _, preds = torch.max(outputs, 1)
        test_corrects += torch.sum(preds == labels.data)

        # Save actual and predicted labels
        y_true.extend(labels.cpu().numpy())
        y_pred.extend(preds.cpu().numpy())

# Stop the stopwatch
end_time = time.perf_counter()

# Calculate the speeds
total_time = end_time - start_time
num_images = len(test_data)
avg_time_per_image_seconds = total_time / num_images
avg_time_per_image_ms = avg_time_per_image_seconds * 1000  # Convert to milliseconds

test_acc = test_corrects.double() / len(test_data)

print("====================================")
print(f"FINAL TEST ACCURACY:   {test_acc * 100:.2f}%")
print(f"TOTAL TEST TIME:       {total_time:.2f} seconds")
print(f"AVG TIME PER IMAGE:    {avg_time_per_image_ms:.2f} ms")
print("====================================")

# --- 8. SAVE THE TRAINED MODEL ---
print("\nSaving the model weights...")
save_path = os.path.join(script_dir, "best_param_classification_modular.pth")
torch.save(model.state_dict(), save_path)
print(f"Model successfully saved to: {save_path}")

# --- 9. GENERATE AND SAVE CONFUSION MATRIX (YOLO STYLE) ---
print("\nGenerating and saving Confusion Matrix...")

# 1. Generate the standard matrix
cm = confusion_matrix(y_true, y_pred)

# 2. TRANSPOSE the matrix (Actual/True on X, Predicted on Y)
cm_transposed = cm.T

# 3. Create a custom annotation array (Hides the '0' values to match YOLO)
annot_array = np.empty_like(cm_transposed, dtype=object)
for i in range(cm_transposed.shape[0]):
    for j in range(cm_transposed.shape[1]):
        annot_array[i, j] = str(cm_transposed[i, j]) if cm_transposed[i, j] > 0 else ""

# 4. Plot using Seaborn
plt.figure(figsize=(10, 10))  # Made slightly larger to accommodate square cells
sns.heatmap(
    cm_transposed,
    annot=annot_array,
    fmt="",  # Must be empty because annot_array contains strings
    cmap="Blues",
    cbar=True,  # Show the colorbar on the right
    square=True,  # Force the cells to be perfect squares (YOLO style)
    vmin=0,  # Force the bottom of the color scale to be pure white
    xticklabels=class_names,
    yticklabels=class_names,
    cbar_kws={"shrink": 0.8},  # Shrink colorbar slightly to match plot height
)

# 5. Format labels and titles to match YOLO perfectly
plt.xlabel("True")
plt.ylabel("Predicted")
plt.title("Confusion Matrix")

# Rotate X-axis labels 90 degrees (vertical) and Y-axis 0 degrees (horizontal)
plt.xticks(rotation=90)
plt.yticks(rotation=0)

plt.tight_layout()

# Save the plot as a high-resolution image file
cm_save_path = os.path.join(script_dir, "confusion_matrix.png")
plt.savefig(cm_save_path, dpi=300, bbox_inches="tight")  # High-res save
print(f"Confusion matrix plot successfully saved to: {cm_save_path}")
