import copy
import os
import time

import matplotlib.pyplot as plt
import optuna
import optuna.visualization.matplotlib as vis
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
from torchvision.models import ResNet18_Weights, resnet18

# --- 1. SET UP PATHS & DEVICE (Global) ---
script_dir = os.path.dirname(os.path.abspath(__file__))
train_dir = os.path.join(script_dir, "Training_Data")
val_dir = os.path.join(script_dir, "Validation_Data")
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def objective(trial):
    # --- A. SUGGEST HYPERPARAMETERS ---
    # Log scale is best for learning rates
    lr = trial.suggest_float("lr", 1e-4, 1e-2, log=True)
    # Batch size usually performs best in powers of 2
    batch_size = trial.suggest_categorical("batch_size", [4, 8, 16, 32])
    # Augmentation tweaks
    rotation_deg = trial.suggest_int("rotation", 0, 45)
    crop_scale_min = trial.suggest_float("crop_scale_min", 0.6, 0.9)
    # Architecture tweak (Dropout rate before the final layer)
    dropout_rate = trial.suggest_float("dropout", 0.2, 0.7)

    # --- B. DATA SETUP (Apply suggested augmentations) ---
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

    train_transforms = transforms.Compose(
        [
            transforms.RandomResizedCrop(224, scale=(crop_scale_min, 1.0)),
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(rotation_deg),  # Optuna controls this
            transforms.ToTensor(),
            normalize,
        ]
    )

    test_transforms = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), normalize])

    train_data = datasets.ImageFolder(train_dir, transform=train_transforms)
    val_data = datasets.ImageFolder(val_dir, transform=test_transforms)

    # We use the Optuna batch size here
    train_loader = DataLoader(train_data, batch_size=batch_size, shuffle=True, num_workers=2, pin_memory=True)
    val_loader = DataLoader(val_data, batch_size=batch_size, shuffle=False, num_workers=2, pin_memory=True)

    class_names = train_data.classes

    # --- C. MODEL SETUP (Fresh for every trial) ---
    model = resnet18(weights=ResNet18_Weights.DEFAULT)
    for param in model.parameters():
        param.requires_grad = False

    num_features = model.fc.in_features
    # Apply Optuna's dropout rate
    model.fc = nn.Sequential(nn.Dropout(dropout_rate), nn.Linear(num_features, len(class_names)))
    model = model.to(device)

    criterion = nn.CrossEntropyLoss()
    # Apply Optuna's learning rate
    optimizer = optim.Adam(model.fc.parameters(), lr=lr)

    # --- D. TRAINING LOOP ---
    epochs = 15  # Keep epochs lower for hyperparameter tuning!
    best_val_acc = 0.0

    for epoch in range(epochs):
        model.train()
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()

        # Validation Phase
        model.eval()
        val_corrects = 0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(device), labels.to(device)
                outputs = model(inputs)
                _, preds = torch.max(outputs, 1)
                val_corrects += torch.sum(preds == labels.data)

        val_acc = val_corrects.double() / len(val_data)

        # Keep track of the best accuracy for this trial
        if val_acc > best_val_acc:
            best_val_acc = val_acc

        # --- E. OPTUNA PRUNING (CRITICAL FOR DL) ---
        # Report intermediate accuracy to Optuna
        trial.report(val_acc, epoch)

        # Handle pruning based on the intermediate value
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    # We return the best validation accuracy achieved during this trial's epochs
    return best_val_acc


if __name__ == "__main__":
    print("Starting Deep Learning Hyperparameter Optimization...")

    # We use a MedianPruner. If a trial's intermediate result is worse than
    # the median of previous trials at the same step, it gets killed.
    pruner = optuna.pruners.MedianPruner(n_startup_trials=5, n_warmup_steps=3)
    study = optuna.create_study(direction="maximize", pruner=pruner)

    # Only run ~20-30 trials for deep learning unless you have a lot of time/GPUs
    study.optimize(objective, n_trials=30, show_progress_bar=True)

    print("\n--- OPTIMIZATION FINISHED ---")
    print(f"Best Validation Accuracy: {round(study.best_value * 100, 2)}%")
    print("Best Parameters:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")

    # Generate the exact same high-quality Matplotlib figures
    folder_path = os.path.join(script_dir, "neural_network_figures")
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    vis.plot_optimization_history(study)
    plt.savefig(os.path.join(folder_path, "optimization_history.png"), dpi=300)
    plt.close()

    vis.plot_param_importances(study)
    plt.savefig(os.path.join(folder_path, "param_importances.png"), dpi=300)
    plt.close()

    # =================================================================
    # --- PHASE 2: FINAL TRAINING WITH BEST PARAMETERS ---
    # =================================================================
    print("\nStarting final full training run with the best parameters...")

    best_params = study.best_params
    test_dir = os.path.join(script_dir, "Test_Data")

    # 1. Setup Final Transforms & DataLoaders
    train_transforms = transforms.Compose(
        [
            transforms.RandomResizedCrop(224, scale=(best_params["crop_scale_min"], 1.0)),
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(best_params["rotation"]),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ]
    )

    test_transforms = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])

    train_data = datasets.ImageFolder(train_dir, transform=train_transforms)
    val_data = datasets.ImageFolder(val_dir, transform=test_transforms)
    test_data = datasets.ImageFolder(test_dir, transform=test_transforms)

    final_batch_size = best_params["batch_size"]
    train_loader = DataLoader(train_data, batch_size=final_batch_size, shuffle=True)
    val_loader = DataLoader(val_data, batch_size=final_batch_size, shuffle=False)
    test_loader = DataLoader(test_data, batch_size=final_batch_size, shuffle=False)

    class_names = train_data.classes

    # 2. Build Final Model
    model = resnet18(weights=ResNet18_Weights.DEFAULT)
    for param in model.parameters():
        param.requires_grad = False

    num_features = model.fc.in_features
    model.fc = nn.Sequential(nn.Dropout(best_params["dropout"]), nn.Linear(num_features, len(class_names)))
    model = model.to(device)

    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.fc.parameters(), lr=best_params["lr"])

    # 3. Final Training Loop (Full 100 Epochs + Patience)
    epochs = 100
    patience = 15
    best_val_loss = float("inf")
    epochs_no_improve = 0
    best_model_wts = copy.deepcopy(model.state_dict())

    for epoch in range(epochs):
        model.train()
        running_loss = 0.0
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            optimizer.zero_grad()
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * inputs.size(0)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(device), labels.to(device)
                outputs = model(inputs)
                loss = criterion(outputs, labels)
                val_loss += loss.item() * inputs.size(0)

        val_loss = val_loss / len(val_data)

        print(f"Final Run - Epoch {epoch + 1}/{epochs} | Val Loss: {val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_model_wts = copy.deepcopy(model.state_dict())
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
            if epochs_no_improve >= patience:
                print("Early stopping triggered on final run!")
                break

    # Load the absolute best weights
    model.load_state_dict(best_model_wts)

    # 4. Save the Model
    save_path = os.path.join(folder_path, "resnet18_best_weights.pth")
    torch.save(model.state_dict(), save_path)
    print(f"\nModel successfully saved to: {save_path}")

    # 5. Evaluate on Test Data
    print("\nEvaluating final optimized model on test data...")
    model.eval()
    test_corrects = 0
    y_true = []
    y_pred = []

    start_time = time.perf_counter()
    with torch.no_grad():
        for inputs, labels in test_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, preds = torch.max(outputs, 1)
            test_corrects += torch.sum(preds == labels.data)

            y_true.extend(labels.cpu().numpy())
            y_pred.extend(preds.cpu().numpy())

    end_time = time.perf_counter()
    total_time = end_time - start_time
    test_acc = test_corrects.double() / len(test_data)

    print("====================================")
    print(f"FINAL TEST ACCURACY:   {test_acc * 100:.2f}%")
    print(f"TOTAL TEST TIME:       {total_time:.2f} seconds")
    print(f"AVG TIME PER IMAGE:    {(total_time / len(test_data)) * 1000:.2f} ms")
    print("====================================")

    # 6. Generate Confusion Matrix
    import numpy as np
    from sklearn.metrics import confusion_matrix

    display_labels = ["Unknown", "Green_Circle", "Green_Square", "Blue_Circle", "Blue_Square", "Red_Circle", "Red_Square", "background"]

    label_map = {
        "Unidentified_Object": "Unknown",
        "No_Object": "background",
        "Blue_Circle": "Blue_Circle",
        "Blue_Square": "Blue_Square",
        "Red_Circle": "Red_Circle",
        "Red_Square": "Red_Square",
        "Green_Circle": "Green_Circle",
        "Green_Square": "Green_Square",
    }

    # 1. Convert PyTorch integer indices back to strings
    y_true_strings = [class_names[idx] for idx in y_true]
    y_pred_strings = [class_names[idx] for idx in y_pred]

    # 2. Apply mappings
    y_true_mapped = [label_map.get(lbl, lbl) for lbl in y_true_strings]
    y_pred_mapped = [label_map.get(lbl, lbl) for lbl in y_pred_strings]

    # 3. Calculate matrix with SWAPPED inputs to force Predicted=Y, True=X
    cm = confusion_matrix(y_pred_mapped, y_true_mapped, labels=display_labels)

    # 4. Plotting (Your exact original styling)
    fig, ax = plt.subplots(figsize=(10, 8), dpi=300)
    im = ax.imshow(cm, interpolation="nearest", cmap=plt.cm.Blues)
    ax.figure.colorbar(im, ax=ax)

    # Styles (Updated the x/y labels so they match the swapped data!)
    ax.set(xticks=np.arange(cm.shape[1]), yticks=np.arange(cm.shape[0]), xticklabels=display_labels, yticklabels=display_labels, title="Confusion Matrix", ylabel="Predicted Label", xlabel="True Label")
    ax.grid(False)

    plt.setp(ax.get_xticklabels(), rotation=90, ha="right", rotation_mode="anchor")

    # Add text annotations
    thresh = cm.max() / 2.0
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            if cm[i, j] > 0:
                ax.text(j, i, format(cm[i, j], "d"), ha="center", va="center", color="white" if cm[i, j] > thresh else "black")

    fig.tight_layout(pad=2.0)
    plt.savefig(os.path.join(folder_path, "confusion_matrix.png"))
    plt.close()

    print(f"Done! High-res figures and results table are in /{folder_path}")
