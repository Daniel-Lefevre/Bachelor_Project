from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def inv(x):
    return 1 / x


def best_fit(robot_id, conveyor_id, x1, x0):
    # Ensure x1 and x0 are at least small non-zero to avoid log(0) or div by zero
    x1 = np.where(x1 <= 1, 1.01, x1)
    x0 = np.where(x0 <= 0, 0.01, x0)

    if robot_id == 0:
        if conveyor_id == 0:
            return (((x1 * -1.0953979e-6) + 0.00166411) * x1) - -0.06900252
            # cube(sqrt((sqrt(x1) * 0.030723851) + 0.06802121))
        elif conveyor_id == 1:
            return (x1 * ((x1 * 9.2536857e-7) + -0.0015947099)) + 0.6181552
            # (sqrt(x1) * -0.03247589) + 0.7859344
    elif robot_id == 1:
        if conveyor_id == 0:
            return 0.64444643 - (x1 * (0.0017557096 - (x1 * 1.1290276e-6)))
            # (sqrt(x1) * -0.034916986) + 0.82572114
        elif conveyor_id == 1:
            return (((x1 * -1.5215395e-6) + 0.0018650172) * x1) - -0.07937143
            # square((sqrt(x1) * -0.027879689) - 0.22453631)


def calculate_metrics(y_true, y_pred):
    # RMSE calculation
    rmse = np.sqrt(np.mean((y_true - y_pred) ** 2))

    # R-squared calculation
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    r2 = 1 - (ss_res / ss_tot)

    return rmse, r2


BASE_DIR = Path(__file__).resolve().parent
TEST_POINTS_CONVEYOR_DISTANCE = [[26, 16.5, 43.5, 27, 22.5, 27, 51.5, 26, 40.5, 47.5], [21, 14.5, 37, 9.5, 52, 28.5, 45, 11, 33.5, 58]]

if __name__ == "__main__":
    CURRENT_DIR = Path(__file__).resolve().parent
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(12, 10), sharex=True, sharey=True)

    x1_range = np.linspace(0, 480, 962)

    for robot_id in [0, 1]:
        for conveyor_id in [0, 1]:
            # --- Data Loading ---
            images_path = CURRENT_DIR / "Test_Data" / f"Conveyor_{conveyor_id}" / f"Robot_{robot_id}"
            csv_path = images_path / "object_centers.csv"

            # Assuming data_array[:, 1] contains the X values used for regression
            data_array = np.loadtxt(str(csv_path), delimiter=",")
            x_test = data_array[:, 1]
            y_actual = np.array(TEST_POINTS_CONVEYOR_DISTANCE[conveyor_id])

            # --- Metrics Calculation ---
            # Get predictions for the specific test points
            y_pred = best_fit(robot_id, conveyor_id, x_test, 0) * 100
            rmse, r2 = calculate_metrics(y_actual, y_pred)

            # --- Plotting ---
            ax = axes[robot_id, conveyor_id]

            # Plot the continuous fit line
            ax.plot(x1_range, best_fit(robot_id, conveyor_id, x1_range, 0) * 100, color="red", alpha=0.7, label="Best Fit")

            # Plot the actual test scatter points
            ax.scatter(x_test, y_actual, color="blue", s=20, label="Test points")

            # --- Add Metrics Box ---
            # --- Add Metrics Box ---
            stats_text = f"$R^2$: {r2:.3f}\nRMSE: {rmse:.2f}"
            props = dict(boxstyle="round", facecolor="white", alpha=0.8)

            # Conditionally set the position based on the robot and conveyor IDs
            if (robot_id == 0 and conveyor_id == 1) or (robot_id == 1 and conveyor_id == 0):
                # Place box in the lower left for the downward-sloping graphs
                y_pos = 0.05
                v_align = "bottom"
            else:
                # Keep box in the upper left for the upward-sloping graphs
                y_pos = 0.95
                v_align = "top"

            ax.text(0.05, y_pos, stats_text, transform=ax.transAxes, fontsize=10, verticalalignment=v_align, bbox=props)
            ax.set_title(f"Robot {robot_id} | Conveyor {conveyor_id}")
            ax.grid(True, linestyle="--", alpha=0.6)
            ax.set_xlim(0, 480)
            # REMOVED: The ax.legend() call that was here

    # --- Global Labels & Legend ---
    fig.suptitle("Pixel-to-Position Regression", fontsize=16)
    fig.supxlabel("Object Center Image Coordinate on the y-axis (px)")
    fig.supylabel("Object Conveyor Position (cm)")

    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.94), ncol=2, frameon=False)

    # You can leave the bottom margin standard now, it will auto-adjust
    plt.tight_layout(rect=[0.02, 0.02, 1, 0.90], h_pad=5.0)
    plt.show()
