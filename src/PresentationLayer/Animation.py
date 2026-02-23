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
        self.dt = dt
        self.canvas_has_been_resized = False
        self.original_canvas_width = 1280
        self.original_canvas_height = 704
        self.storage_objects = objects
        self.root = root
        self.canvas = tk.Canvas(self.root, bg="#ffffff")
        self.canvas.grid(row=1, column=0, columnspan=2, sticky="news", padx=(30, 0), pady=30)
        self.canvas.bind("<Configure>", self.on_resize)

    def _sx(self, x):
        return x * self.width_scale

    def _sy(self, y):
        return y * self.height_scale

    def _initialize_canvas_values(self):
        self.storage_pos_dict = {
            "Red Square": [self._sx(125), self._sy(550), self._sx(175), self._sy(600)],
            "Blue Square": [self._sx(50), self._sy(550), self._sx(100), self._sy(600)],
            "Green Square": [self._sx(200), self._sy(550), self._sx(250), self._sy(600)],
            "Red Circle": [self._sx(75), self._sy(475)],
            "Blue Circle": [self._sx(225), self._sy(475)],
            "Green Circle": [self._sx(150), self._sy(475)],
        }

        self.objects = [self._storage_object_to_animation_object(obj) for obj in self.storage_objects]

        self.storage_offset = self._sx(985)

        self.robot_positions = [
            # Robot 0's positions
            {
                "Place_Storage": [self._sx(150), self._sy(500)], 
                "Pickup_Storage": [self._sx(150), self._sy(500)], 
                "Place_Conveyor": [self._sx(450), self._sy(200)],
                "Standby": [self._sx(350), self._sy(200)], 
                "Pickup_Conveyor": [self._sx(450), self._sy(495)], 
                "Observation": [self._sx(350), self._sy(495)]
            },
            # Robot 1's positions
            {
                "Place_Storage": [self._sx(1135), self._sy(500)], 
                "Pickup_Storage": [self._sx(1135), self._sy(500)], 
                "Place_Conveyor": [self._sx(830), self._sy(495)], 
                "Standby": [self._sx(930), self._sy(495)], 
                "Pickup_Conveyor": [self._sx(830), self._sy(200)], 
                "Observation": [self._sx(950), self._sy(200)]},
        ]

        self._create_conveyors()
        self._create_ir()
        self._create_storages()
        self._create_objects()
        self._create_robot_arms()

    def on_resize(self, event):
        self.canvas.delete("all")
        self.width_scale = event.width / self.original_canvas_width
        self.height_scale = event.height / self.original_canvas_height
        self.canvas_has_been_resized = True
        self._initialize_canvas_values()

    def set_info_dt(self, info):
        if self.canvas_has_been_resized:
            self.info_dt = info

            # Dont animate anything if we are in setup
            if info["robots"][0][0].key != "Setup" and info["robots"][1][0].key != "Setup":
                # Animate the robot
                for robot_id, robot in enumerate(self.info_dt["robots"]):
                    origin = robot[0].origin
                    destination = robot[0].destination

                    self._animate_robot_frame(origin, destination, robot_id, robot[1])
                
                # Animate the objects
                for virtual_object in self.info_dt["objects"]:
                    self._animate_object(virtual_object)
                print("-----------")

    def _animate_object(self, virtual_object):
        shape, color = virtual_object[0]
        state = virtual_object[1][0]
        progress = virtual_object[1][1]

        if (shape == ObjectShape.CIRCLE and color == ObjectColor.RED):
            print(state)

        origin = state.origin
        animation_object = self._get_animation_objec(shape, color)
        if origin == "Storage":
            if shape == ObjectShape.CIRCLE:
                x, y = animation_object.storage_position
                if state.id == 1:
                    x += self.storage_offset
                self.canvas.coords(animation_object.canvas_ref, x - self._sx(25), y - self._sy(25), x + self._sx(25), y + self._sy(25))
            elif shape == ObjectShape.SQUARE:
                x1, y1, x2, y2 = animation_object.storage_position
                if state.id == 1:
                    x1 += self.storage_offset
                    x2 += self.storage_offset
                self.canvas.coords(animation_object.canvas_ref, x1, y1, x2, y2)
        elif origin == "Conveyor":
            start_x, start_y = self.robot_positions[int(not state.id)]["Place_Conveyor"]
            end_x, end_y = self.robot_positions[state.id]["Pickup_Conveyor"]
            x = (end_x - start_x) * progress + start_x
            y = (end_y - start_y) * progress + start_y
            self.canvas.coords(animation_object.canvas_ref, x - self._sx(25), y - self._sy(25), x + self._sx(25), y + self._sy(25))
        elif origin == "Robot":
            x1, y1, x2, y2 = self.canvas.coords(self.animation_robots[state.id].head)
            self.canvas.coords(animation_object.canvas_ref, x1 - self._sx(5), y1 - self._sy(5), x2 + self._sx(5), y2 + self._sy(5))
        elif origin == "IR":
            x, y  = self.robot_positions[state.id]["Pickup_Conveyor"]
            self.canvas.coords(animation_object.canvas_ref, x - self._sx(25), y - self._sy(25), x + self._sx(25), y + self._sy(25))


    def _get_animation_objec(self, shape, color):
        for obj in self.objects:
            if obj.shape == shape and obj.color == color:
                return obj


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

        # 3. Calculate the offset vector based on your desired thickness
        thickness = self._sx(20)
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
        r = self._sx(20)
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
        self.canvas.create_polygon(self._sx(50), self._sy(100), self._sx(250), self._sy(100), self._sx(250), self._sy(300), self._sx(50), self._sy(300), fill="#4167B6")
        self._create_circle(self._sx(150), self._sy(200), self._sx(80), fill="black", outline="")
        self._create_circle(self._sx(150), self._sy(200), self._sx(20), fill="#D1D8DE", outline="")

        # Robot 1 base (static parts of robot)
        self.canvas.create_polygon(self._sx(1035), self._sy(100), self._sx(1235), self._sy(100), self._sx(1235), self._sy(300), self._sx(1035), self._sy(300), fill="#4167B6")
        self._create_circle(self._sx(1135), self._sy(200), self._sx(80), fill="black", outline="")
        self._create_circle(self._sx(1135), self._sy(200), self._sx(20), fill="#D1D8DE", outline="")

        self.animation_robots = [
            SimpleNamespace(
                head=self._create_circle(*self.robot_positions[0]["Observation"], self._sx(20), fill="black", outline=""),
                arm=self.canvas.create_polygon(self._sx(150), self._sy(220), self._sx(450), self._sy(220), self._sx(450), self._sy(180), self._sx(150), self._sy(180), fill="#D1D8DE"),
                base=(self._sx(150), self._sy(200)),
            ),
            SimpleNamespace(
                head=self._create_circle(*self.robot_positions[1]["Observation"], self._sx(20), fill="black", outline=""),
                arm=self.canvas.create_polygon(self._sx(835), self._sy(220), self._sx(1135), self._sy(220), self._sx(1135), self._sy(180), self._sx(835), self._sy(180), fill="#D1D8DE"),
                base=(self._sx(1135), self._sy(200)),
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
                obj.canvas_ref = self._create_circle(pos[0], pos[1], self._sx(25), fill=color, outline="")

            elif obj.shape == ObjectShape.SQUARE:
                if obj.state.id:
                    for i in range(0, 4, 2):
                        pos[i] += self.storage_offset
                obj.canvas_ref = self.canvas.create_rectangle(*pos, fill=color, outline="")

    def _create_storages(self):
        self.canvas.create_rectangle(self._sx(25), self._sy(400), self._sx(275), self._sy(650), fill="#ffffff", outline="#000000", width=5)
        self.canvas.create_rectangle(self._sx(1010), self._sy(400), self._sx(1260), self._sy(650), fill="#ffffff", outline="#000000", width=5)

    def _create_conveyors(self):
        self.canvas.create_rectangle(self._sx(375), self._sy(125), self._sx(910), self._sy(275), fill="#484A4D")
        self.canvas.create_rectangle(self._sx(375), self._sy(425), self._sx(910), self._sy(575), fill="#484A4D")

    def _create_ir(self):
        # IR 1
        self._create_circle(self._sx(830), self._sy(103), self._sx(17), fill="#61130B", outline="")
        self.canvas.create_rectangle(self._sx(815), self._sy(45), self._sx(845), self._sy(95), fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(self._sx(810), self._sy(85), self._sx(850), self._sy(105), fill="black", outline="")

        # IR 0
        self._create_circle(self._sx(450), self._sy(403), self._sx(17), fill="#61130B", outline="")
        self.canvas.create_rectangle(self._sx(435), self._sy(345), self._sx(465), self._sy(395), fill="#CC9C3F", outline="")
        self.canvas.create_rectangle(self._sx(430), self._sy(385), self._sx(470), self._sy(405), fill="black", outline="")
