import tkinter as tk
from dataclasses import dataclass
from typing import Any, List, Literal

from pyniryo import ObjectColor, ObjectShape

from src.BusinessLayer.DT.States import ObjectState


@dataclass
class AnimationObject:
    name: str
    shape: Literal[ObjectShape.CIRCLE, ObjectShape.SQUARE]
    color: Literal[ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]
    state: ObjectState
    storage_index: int
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
        self.canvas = tk.Canvas(self.root, bg="#6C757D")
