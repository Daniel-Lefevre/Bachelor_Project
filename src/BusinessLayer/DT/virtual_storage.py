from __future__ import annotations

import copy
from typing import TYPE_CHECKING

from resources.environment import configuration

if TYPE_CHECKING:
    from pyniryo import ObjectColor, ObjectShape


class VirtualStorage:
    def __init__(self, id):
        self.id = id
        self.objects = copy.deepcopy(configuration["StorageOccupancy"][self.id])

    def add_object(self, shape: ObjectShape, color: ObjectColor) -> None:
        for i in range(len(self.objects)):
            if self.objects[i] is None:
                self.objects[i] = (shape, color)
                break

    def remove_object(self, shape: ObjectShape, color: ObjectColor) -> None:
        for i, obj in enumerate(self.objects):
            if obj is not None and obj[0] == shape and obj[1] == color:
                self.objects[i] = None
                break

    def get_storage_position(self, shape: ObjectShape, color: ObjectColor) -> int | None:
        for i in range(len(self.objects)):
            obj = self.objects[i]
            if obj is not None and obj[0] == shape and obj[1] == color:
                return i
        return None
