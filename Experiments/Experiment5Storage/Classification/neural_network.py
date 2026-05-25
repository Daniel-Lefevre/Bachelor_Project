import copy
import os
import time

import matplotlib.pyplot as plt
import numpy as np
import optuna
import optuna.visualization.matplotlib as vis
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
output_folder = os.path.join(script_dir, "neural_network_figures")

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")


# --- 2. OPTUNA OBJECTIVE FUNCTION ---
def objective(trial):
    lr = trial.suggest_float("lr", 1e-4, 1e-2, log=True)
    batch_size = trial.suggest_categorical("batch_size", [4, 8, 16, 32])
    rotation_deg = trial.suggest_int("rotation", 0, 45)
    crop_scale_min = trial.suggest_float("crop_scale_min", 0.6, 0.9)
    dropout_rate = trial.suggest_float("dropout", 0.2, 0.7)

    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    t_transforms = transforms.Compose(
        [
            transforms.RandomResizedCrop(224, scale=(crop_scale_min, 1.0)),
            transforms.RandomHorizontalFlip(),
            transforms.RandomRotation(rotation_deg),
            transforms.ToTensor(),
            normalize,
        ]
    )
    v_transforms = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), normalize])

    train_loader = DataLoader(datasets.ImageFolder(train_dir, t_transforms), batch_size=batch_size, shuffle=True, num_workers=2, pin_memory=True)
    val_loader = DataLoader(datasets.ImageFolder(val_dir, v_transforms), batch_size=batch_size, shuffle=False, num_workers=2, pin_memory=True)

    model = resnet18(weights=ResNet18_Weights.DEFAULT)
    for param in model.parameters():
        param.requires_grad = False

    num_features = model.fc.in_features
    class_names = datasets.ImageFolder(train_dir).classes
    model.fc = nn.Sequential(nn.Dropout(dropout_rate), nn.Linear(num_features, len(class_names)))
    model = model.to(device)

    optimizer = optim.Adam(model.fc.parameters(), lr=lr)
    criterion = nn.CrossEntropyLoss()

    epochs = 30
    best_val_acc = 0.0
    for epoch in range(epochs):
        model.train()
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            optimizer.zero_grad()
            loss = criterion(model(inputs), labels)
            loss.backward()
            optimizer.step()

        model.eval()
        val_corrects = 0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(device), labels.to(device)
                outputs = model(inputs)
                _, preds = torch.max(outputs, 1)
                val_corrects += torch.sum(preds == labels.data)

        accuracy = val_corrects.double() / len(val_loader.dataset)
        if accuracy > best_val_acc:
            best_val_acc = accuracy

        trial.report(accuracy, epoch)
        if trial.should_prune():
            raise optuna.exceptions.TrialPruned()

    return best_val_acc


if __name__ == "__main__":
    # --- 3. RUN OPTIMIZATION ---
    print("\n[PHASE 1] Starting Optuna Study...")
    pruner = optuna.pruners.MedianPruner(n_startup_trials=5, n_warmup_steps=3)
    study = optuna.create_study(direction="maximize", pruner=pruner)
    study.optimize(objective, n_trials=30, show_progress_bar=True)

    print("\n===== BEST HYPERPARAMETERS =====")
    for key, value in study.best_params.items():
        print(f"{key}: {value}")

    print(f"\nBest validation accuracy: {study.best_value:.4f}")

    # Save Optuna Plots
    vis.plot_optimization_history(study)
    plt.savefig(os.path.join(output_folder, "opt_history.png"), dpi=300)
    plt.close()

    # --- 4. FINAL TRAINING WITH BEST PARAMS ---
    print("\n[PHASE 2] Starting Final Training Run...")
    best = study.best_params
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    train_t = transforms.Compose([transforms.RandomResizedCrop(224, scale=(best["crop_scale_min"], 1.0)), transforms.RandomHorizontalFlip(), transforms.RandomRotation(best["rotation"]), transforms.ToTensor(), normalize])
    test_t = transforms.Compose([transforms.Resize(256), transforms.CenterCrop(224), transforms.ToTensor(), normalize])

    train_set = datasets.ImageFolder(train_dir, train_t)
    val_set = datasets.ImageFolder(val_dir, test_t)
    test_set = datasets.ImageFolder(test_dir, test_t)

    train_loader = DataLoader(train_set, batch_size=best["batch_size"], shuffle=True)
    val_loader = DataLoader(val_set, batch_size=best["batch_size"], shuffle=False)
    test_loader = DataLoader(test_set, batch_size=best["batch_size"], shuffle=False)

    class_names = train_set.classes
    model = resnet18(weights=ResNet18_Weights.DEFAULT)
    for param in model.parameters():
        param.requires_grad = False

    model.fc = nn.Sequential(nn.Dropout(best["dropout"]), nn.Linear(model.fc.in_features, len(class_names)))
    model = model.to(device)

    optimizer = optim.Adam(model.fc.parameters(), lr=best["lr"])
    criterion = nn.CrossEntropyLoss()

    best_val_loss = float("inf")
    patience, epochs_no_improve = 15, 0
    best_wts = copy.deepcopy(model.state_dict())

    for epoch in range(100):
        model.train()
        for inputs, labels in train_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            optimizer.zero_grad()
            criterion(model(inputs), labels).backward()
            optimizer.step()

        model.eval()
        v_loss = 0.0
        with torch.no_grad():
            for inputs, labels in val_loader:
                inputs, labels = inputs.to(device), labels.to(device)
                v_loss += criterion(model(inputs), labels).item() * inputs.size(0)
        v_loss /= len(val_set)
        if v_loss < best_val_loss:
            best_val_loss = v_loss
            best_wts = copy.deepcopy(model.state_dict())
            epochs_no_improve = 0
        else:
            epochs_no_improve += 1
            if epochs_no_improve >= patience:
                break

    model.load_state_dict(best_wts)

    # --- 5. EVALUATION & EXACT CONFUSION MATRIX ---
    print("\n[PHASE 3] Final Evaluation...")
    y_true, y_pred = [], []
    model.eval()
    test_corrects = 0
    start_time = time.perf_counter()
    with torch.no_grad():
        for inputs, labels in test_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, preds = torch.max(outputs, 1)
            test_corrects += torch.sum(preds == labels.data)
            y_true.extend(labels.cpu().numpy())
            y_pred.extend(preds.cpu().numpy())

    total_time = time.perf_counter() - start_time
    print(f"FINAL TEST ACCURACY: {(test_corrects.double() / len(test_set)) * 100:.2f}%")

    # Fix Labels: unidentified_object -> unknown
    display_labels = [n.replace("unidentified_object", "unknown").replace("Unidentified_Object", "unknown") for n in class_names]

    # Calculate Matrix and Transpose for Predicted=Y, True=X
    cm = confusion_matrix(y_true, y_pred)
    cm_t = cm.T

    # Create Selective Annotations (only show values > 0)
    annot = np.empty_like(cm_t, dtype=object)
    for i in range(cm_t.shape[0]):
        for j in range(cm_t.shape[1]):
            annot[i, j] = str(cm_t[i, j]) if cm_t[i, j] > 0 else ""

    # Plotting with Seaborn for original look
    plt.figure(figsize=(10, 10))
    sns.heatmap(
        cm_t,
        annot=annot,
        fmt="",
        cmap="Blues",
        square=True,
        xticklabels=display_labels,
        yticklabels=display_labels,
        cbar=True,
        cbar_kws={"shrink": 0.7},  # Shrink color bar to 0.7
    )

    plt.xlabel("True")
    plt.ylabel("Predicted")
    plt.title("Confusion Matrix")
    plt.xticks(rotation=90)
    plt.yticks(rotation=0)
    plt.grid(False)  # Ensure no grid lines
    plt.tight_layout()

    plt.savefig(os.path.join(output_folder, "final_confusion_matrix.png"), dpi=300)
    torch.save(model.state_dict(), os.path.join(output_folder, "optimized_resnet18.pth"))
    print(f"Done! Results saved in {output_folder}")
