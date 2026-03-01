from resources.environment import configuration
from pyniryo import ObjectColor, ObjectShape


class VirtualStorage:
    def __init__(self, id):
        self.id = id
        self.objects = configuration["StorageOccupancy"][self.id]

    def add_object(self, shape: ObjectShape, color: ObjectColor) -> None:
        for i in range(len(self.objects)):
            if (self.objects[i] is None):
                self.objects[i] = (shape, color)

    def remove_object(self, shape: ObjectShape, color: ObjectColor) -> None:
        for i, object in enumerate(self.objects):
            if (object[0] == shape and object[1] == color):
                self.objects[i] = None
                break

    def get_storage_position(self, shape: ObjectShape, color: ObjectColor) -> int | None:
        for i in range(len(self.objects)):
            object = self.objects[i]
            if (object is not None and object[0] == shape and object[1] == color):
                return i
        return None