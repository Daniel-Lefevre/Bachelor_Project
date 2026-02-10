from src.BusinessLayer.DT.VirtualObject import VirtualObject
from pyniryo import *
import time
from queue import PriorityQueue, Queue
from resources.environment import configuration, StorageObject

class TimeBasedDT():

    def __init__(self, stepSize):
        self.stepSize = stepSize
        self.virtualObjects = [
            VirtualObject(ObjectShape.CIRCLE, ObjectColor.RED, "Storage_0", self.stepSize)
        ]
        self.events = Queue()
        self.rules = {}
        self.queues = [PriorityQueue(), PriorityQueue()]
        self.timeSinceLastPops = [float("inf"), float("inf")]
        self.timeouts = [0,0]

        for i in range(len(self.timeouts)):
            self.timeouts[i] = configuration["Observed_times"][f"Robot_{i}_Storage_to_Conveyor"] + configuration["Observed_times"][f"Robot_{i}_Conveyor_to_Safe"]

    def objectToVirtualObject(self, object : StorageObject):
        return VirtualObject(object.shape, object.color, object.position, self.stepSize)

    def findVirtualObject(self, shape, color):
        for object in self.virtualObjects:
            if (object.shape == shape and object.color == color):
                return object

    def step(self):
        # If an event has occured since last step
        while (not self.events.empty()):
            eventType, obj = self.events.get()
            robotID = int(obj.position[-1])
            if (eventType == "Pick Up"):
                self.queues[robotID].put((configuration["PickFromStoragePriority"], self.findVirtualObject(obj.shape, obj.color)))

        # Check if we can pop something from queue
        for robotID in range(len(self.queues)):
            prioQueue = self.queues[robotID]
            if (not prioQueue.empty() and self.timeSinceLastPops[robotID] > self.timeouts[robotID]):
                _, virtualObj =  prioQueue.get()
                virtualObj.pickUpObject()
                self.timeouts[robotID] = 0

        # Increment the time in all objects
        for vObject in self.virtualObjects:
            vObject.IncrementInnerClock(self.rules[(vObject.shape, vObject.color)])

        # Increment time since lost pop timers
        for i in range(len(self.timeouts)):
            self.timeouts[i] += self.stepSize

    def event(self, event):
        self.events.put(event)

    def setRules(self, rules):
        self.rules = rules
