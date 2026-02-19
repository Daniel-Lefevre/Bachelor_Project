import tkinter as tk
from dataclasses import dataclass
from math import atan2, cos, pi, sin
from types import SimpleNamespace
from typing import Any, List, Literal

from pyniryo import ObjectColor, ObjectShape

from src.BusinessLayer.DT.states import ObjectState, ObjectStates


@dataclass
class AnimationObject:
    name: str
    shape: Literal[ObjectShape.CIRCLE, ObjectShape.SQUARE]
    color: Literal[ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]
    state: ObjectState
    storage_position: List[int]
    canvas_ref: Any = None


class Animation:
    def __init__(self, root, objects, dt=0.1):
        self.storage_pos_dict = {
            "Red Square": [125, 550, 175, 600],
            "Blue Square": [50, 550, 100, 600],
            "Green Square": [200, 550, 250, 600],
            "Red Circle": [75, 475],
            "Blue Circle": [225, 475],
            "Green Circle": [150, 475],
        }
        self.root = root
        self.objects = [self._storage_object_to_animation_object(obj) for obj in objects]

        self.canvas = tk.Canvas(self.root, bg="#ffffff")
        self.canvas.grid(row=1, column=0, columnspan=2, sticky="news", padx=(30, 0), pady=30)
        self.storage_offset = 985
        self.dt = dt

        self.robot_positions = [
            # Robot 0's positions
            {"Storage": [150, 500], "Conveyor_Place": [450, 200], "Conveyor_IR": [450, 470], "Observation": [350, 495]},
            # Robot 1's positions
            {"Storage": [1135, 500], "Conveyor_Place": [830, 495], "Conveyor_IR": [830, 200], "Observation": [930, 200]},
        ]

        self._create_conveyors()
        self._create_ir()
        self._create_storages()
        self._create_objects()
        self._create_robot_arms()

    def set_info_dt(self, info):
        self.info_dt = info

        # Use info to create next frame in Animation
        for robot_id, robot in enumerate(self.info_dt["robots"]):
            origin = ""
            if robot[0].origin == "Conveyor":
                if robot[0].destination == "Observation":
                    origin = "Conveyor_Place"
                else:
                    origin = "Conveyor_IR"
            else:
                origin = robot[0].origin

            destination = ""
            if robot[0].destination == "Conveyor":
                if robot[0].origin == "Observation":
                    destination = "Conveyor_IR"
                else:
                    destination = "Conveyor_Place"
            else:
                destination = robot[0].destination

            self._animate_robot_frame(origin, destination, robot_id, robot[1])

    def _create_circle(self, cx, cy, r, **kwargs):
        return self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, **kwargs)

    def _storage_object_to_animation_object(self, object):
        return AnimationObject(
            object.name,
            object.shape,
            object.color,
            ObjectStates[f"Storage_{object.position[-1]}"],
            self.storage_pos_dict[object.name],
        )

    def _connect_head_to_base(self, robot_id):
        x1, y1, x2, y2 = self.canvas.coords(self.animation_robots[robot_id].head)
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        x0, y0 = self.animation_robots[robot_id].base
        y_diff = cy - y0
        x_diff = cx - x0

        # 1. Calculate the angle of the arm itself
        arm_angle = atan2(y_diff, x_diff)

        # 2. The perpendicular angle (for the width/thickness)
        # This is always 90 degrees (pi/2) offset from the arm
        orth_angle = arm_angle + (pi / 2)

        # 3. Calculate the offset vector based on your desired thickness (e.g., 20px)
        thickness = 20
        dx_orth = cos(orth_angle) * thickness
        dy_orth = sin(orth_angle) * thickness

        # 4. Define the 4 corners of the rectangle
        # Points at the "head" (cx, cy)
        point1 = [cx + dx_orth, cy + dy_orth]
        point2 = [cx - dx_orth, cy - dy_orth]

        # Points at the "base" (x0, y0)
        point3 = [x0 - dx_orth, y0 - dy_orth]
        point4 = [x0 + dx_orth, y0 + dy_orth]

        points = [*point1, *point2, *point3, *point4]
        self.canvas.coords(self.animation_robots[robot_id].arm, *points)

    def _animate_robot_frame(self, origin, destination, robot_id, progress):
        r = 20
        x, y = (0, 0)
        # If progress is 'float('inf)', then we are in a waiting state, which means there is no destination
        if progress == float("inf"):
            x, y = self.robot_positions[robot_id][origin]
        else:
            progress = progress if progress <= 1 else 1
            origin_x, origin_y = self.robot_positions[robot_id][origin]
            destination_x, destination_y = self.robot_positions[robot_id][destination]
            x = origin_x + (destination_x - origin_x) * progress
            y = origin_y + (destination_y - origin_y) * progress

        self.canvas.coords(self.animation_robots[robot_id].head, x - r, y - r, x + r, y + r)

        self._connect_head_to_base(robot_id)

    def _create_robot_arms(self):
        # Robot 0 base (static parts of robot)
        self.canvas.create_polygon(50, 100, 250, 100, 250, 300, 50, 300, fill="#4167B6")
        self._create_circle(150, 200, 80, fill="black", outline="")
        self._create_circle(150, 200, 20, fill="#D1D8DE", outline="")

        # Robot 1 base (static parts of robot)
        self.canvas.create_polygon(1035, 100, 1235, 100, 1235, 300, 1035, 300, fill="#4167B6")
        self._create_circle(1135, 200, 80, fill="black", outline="")
        self._create_circle(1135, 200, 20, fill="#D1D8DE", outline="")

        self.animation_robots = [
            SimpleNamespace(
                head=self._create_circle(*self.robot_positions[0]["Observation"], 20, fill="black", outline=""),
                arm=self.canvas.create_polygon(150, 220, 450, 220, 450, 180, 150, 180, fill="#D1D8DE"),
                base=(150, 200),
            ),
            SimpleNamespace(
                head=self._create_circle(*self.robot_positions[1]["Observation"], 20, fill="black", outline=""),
                arm=self.canvas.create_polygon(835, 220, 1135, 220, 1135, 180, 835, 180, fill="#D1D8DE"),
                base=(1135, 200),
            ),
        ]

        self._connect_head_to_base(0)
        self._connect_head_to_base(1)

        # Raise the heads to always stay on top of everything
        self.canvas.tag_raise(self.animation_robots[0].head)
        self.canvas.tag_raise(self.animation_robots[1].head)

    def _create_objects(self):
        for obj in self.objects:
            color = None
            if obj.color == ObjectColor.BLUE:
                color = "blue"
            elif obj.color == ObjectColor.RED:
                color = "red"
            elif obj.color == ObjectColor.GREEN:
                color = "green"

            pos = obj.storage_position.copy()

            if obj.shape == ObjectShape.CIRCLE:
                if obj.state.id:
                    pos[0] += self.storage_offset
                obj.canvas_ref = self._create_circle(pos[0], pos[1], 25, fill=color, outline="")

            elif obj.shape == ObjectShape.SQUARE:
                if obj.state.id:
                    for i in range(0, 4, 2):
                        pos[i] += self.storage_offset
                obj.canvas_ref = self.canvas.create_rectangle(*pos, fill=color, outline="")

    def _create_storages(self):
        self.canvas.create_rectangle(25, 400, 275, 650, fill="#ffffff", outline="#000000", width=5)
        self.canvas.create_rectangle(1010, 400, 1260, 650, fill="#ffffff", outline="#000000", width=5)

    def _create_conveyors(self):
        self.canvas.create_rectangle(375, 125, 910, 275, fill="#484A4D")
        self.canvas.create_rectangle(375, 425, 910, 575, fill="#484A4D")

    def _create_ir(self):
        # IR 1
        self._create_circle(830, 103, 17, fill="#61130B", outline="")
        self.canvas.create_rectangle(815, 45, 845, 95, fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(810, 85, 850, 105, fill="black", outline="")

        # IR 0
        self._create_circle(450, 403, 17, fill="#61130B", outline="")
        self.canvas.create_rectangle(435, 345, 465, 395, fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(430, 385, 470, 405, fill="black", outline="")
