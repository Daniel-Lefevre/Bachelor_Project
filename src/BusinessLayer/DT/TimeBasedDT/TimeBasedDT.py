from src.BusinessLayer.DT.VirtualObject import VirtualObject
from pyniryo import *
import time


class TimeBasedDT():

    def __init__(self, stepSize):
        self.virtualObjects = [VirtualObject(ObjectShape.CIRCLE, ObjectColor.RED, "Storage_0", stepSize)]
        self.events = []

    def step(self):
        if len(self.events):
            self.virtualObjects[0].pickUpObject()
            print("Event: " + str(self.events[0]))
            self.events = []
        for vObject in self.virtualObjects:
            vObject.IncrementInnerClock()

    def event(self, event):
        self.events.append(event)
