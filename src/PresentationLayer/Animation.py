import tkinter as tk
from dataclasses import dataclass
from math import atan2, cos, pi, sin
from typing import Any, List, Literal

from pyniryo import ObjectColor, ObjectShape

from src.BusinessLayer.DT.states import ObjectState, ObjectStates


@dataclass
class AnimationObject:
    name: str
    shape: Literal[ObjectShape.CIRCLE, ObjectShape.SQUARE]
    color: Literal[ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]
    state: ObjectState
    storagePosition: List[int]
    canvasRef: Any = None


class Animation:
    def __init__(self, root, objects, stepSize=0.1):
        self.storagePosDict = {
            "Red Square": [125, 550, 175, 600],
            "Blue Square": [50, 550, 100, 600],
            "Green Square": [200, 550, 250, 600],
            "Red Circle": [75, 475],
            "Blue Circle": [225, 475],
            "Green Circle": [150, 475],
        }

        self.root = root
        self.objects = [self.storageObjectToAnimationObject(obj) for obj in objects]

        self.canvas = tk.Canvas(self.root, bg="#ffffff")
        self.canvas.grid(row=1, column=0, columnspan=2, sticky="news", padx=(30, 0), pady=30)

        self.storageOffset = 985
        self.robot0_base = (150, 200)

        self.robotPositions = [
            # Robot 0's positions
            {"storage": [150, 500], "place": [450, 175], "ir": [], "observation": []},
            # Robot 1's positions
            {"storage": [], "place": [], "ir": [], "observation": []},
        ]

        self.createConveyors()
        self.createIR()
        self.createStorages()
        self.createObjects()
        self.createRobotArms()

        self.animate("place", "storage")

    def create_circle(self, cx, cy, r, **kwargs):
        return self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, **kwargs)

    def storageObjectToAnimationObject(self, object):
        return AnimationObject(
            object.name,
            object.shape,
            object.color,
            ObjectStates[f"Storage_{object.position[-1]}"],
            self.storagePosDict[object.name],
        )

    def calculateVelocity(self, progressGoal, beginning, end, robotID, dt=0.02):
        beginning_x, beginning_y = self.robotPositions[robotID][beginning]
        end_x, end_y = self.robotPositions[robotID][end]

        steps = progressGoal / dt
        dx = (end_x - beginning_x) / steps
        dy = (end_y - beginning_y) / steps
        return dx, dy

    def animate(self, beginning, end, step=0, progressGoal=5, dt=0.02):
        dx, dy = self.calculateVelocity(progressGoal, beginning, end, 0, dt)

        self.canvas.move(self.robot0, dx, dy)
        x1, y1, x2, y2 = self.canvas.coords(self.robot0)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        x0, y0 = self.robot0_base
        y_diff = cy - y0
        x_diff = cx - x0
        slope = y_diff / x_diff if x_diff != 0 else float("inf")
        orth_slope = -1 / slope if slope != float("inf") else 0
        angle = atan2(1, orth_slope)
        point1 = (cos(angle), sin(angle))
        point2 = (cos(angle + pi), sin(angle + pi))
        point1 = [point1[0] * 20 + cx, point1[1] * 20 + cy]
        point2 = [point2[0] * 20 + cx, point2[1] * 20 + cy]
        point3 = [point2[0] - x_diff, point2[1] - y_diff]
        point4 = [point1[0] - x_diff, point1[1] - y_diff]
        points = [*point1, *point2, *point3, *point4]
        self.canvas.coords(self.robotArm, *points)

        if step < progressGoal / dt:
            self.root.after(int(dt * 1000), lambda: self.animate(beginning, end, step + 1, progressGoal, dt))

    def createRobotArms(self):
        # Robot 0
        self.canvas.create_polygon(50, 100, 250, 100, 250, 300, 50, 300, fill="#4167B6")
        self.create_circle(150, 200, 80, fill="black", outline="")
        self.create_circle(150, 200, 20, fill="#D1D8DE", outline="")
        self.robotArm = self.canvas.create_polygon(150, 220, 450, 220, 450, 180, 150, 180, fill="#D1D8DE")

        self.robot0 = self.create_circle(450, 200, 20, fill="black", outline="")

        # Robot 1
        self.canvas.create_polygon(1035, 100, 1235, 100, 1235, 300, 1035, 300, fill="#4167B6")
        self.create_circle(1135, 200, 80, fill="black", outline="")
        self.create_circle(1135, 200, 20, fill="#D1D8DE", outline="")
        self.canvas.create_polygon(835, 220, 1135, 220, 1135, 180, 835, 180, fill="#D1D8DE")
        self.create_circle(835, 200, 20, fill="black", outline="")

    def createObjects(self):
        for obj in self.objects:
            color = None
            if obj.color == ObjectColor.BLUE:
                color = "blue"
            elif obj.color == ObjectColor.RED:
                color = "red"
            elif obj.color == ObjectColor.GREEN:
                color = "green"

            pos = obj.storagePosition.copy()

            if obj.shape == ObjectShape.CIRCLE:
                if obj.state.id:
                    pos[0] += self.storageOffset
                obj.canvasRef = self.create_circle(pos[0], pos[1], 25, fill=color, outline="")

            elif obj.shape == ObjectShape.SQUARE:
                if obj.state.id:
                    for i in range(0, 4, 2):
                        pos[i] += self.storageOffset
                obj.canvasRef = self.canvas.create_rectangle(*pos, fill=color, outline="")

    def createStorages(self):
        self.canvas.create_rectangle(25, 400, 275, 650, fill="#ffffff", outline="#000000", width=5)
        self.canvas.create_rectangle(1010, 400, 1260, 650, fill="#ffffff", outline="#000000", width=5)

    def createConveyors(self):
        self.canvas.create_rectangle(375, 125, 910, 275, fill="#484A4D")
        self.canvas.create_rectangle(375, 425, 910, 575, fill="#484A4D")

    def createIR(self):
        # IR 1
        self.create_circle(830, 103, 17, fill="#61130B", outline="")
        self.canvas.create_rectangle(815, 45, 845, 95, fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(810, 85, 850, 105, fill="black", outline="")

        # IR 0
        self.create_circle(450, 403, 17, fill="#61130B", outline="")
        self.canvas.create_rectangle(435, 345, 465, 395, fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(430, 385, 470, 405, fill="black", outline="")
