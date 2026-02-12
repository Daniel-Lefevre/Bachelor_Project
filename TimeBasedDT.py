from src.BusinessLayer.DT.VirtualObject import VirtualObject
from pyniryo import *
import time
from queue import Queue
from resources.PriorityQueue import CustomPriorityQueue
from resources.environment import configuration, StorageObject

class TimeBasedDT():

    def __init__(self, stepSize):
        self.stepSize = stepSize
        self.virtualObjects = [
            VirtualObject(ObjectShape.CIRCLE, ObjectColor.RED, "Storage_0", self.stepSize),
            VirtualObject(ObjectShape.SQUARE, ObjectColor.BLUE, "Storage_0", self.stepSize)
        ]
        self.events = Queue()
        self.rules = {}
        self.queues = [CustomPriorityQueue(2), CustomPriorityQueue(2)]
        self.timeSinceLastPops = [float("inf"), float("inf")]
        self.conveyors = [False, False]

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
            robotID = int(obj.state[-1])
            if (eventType == "Pick Up"):
                self.queues[robotID].put((configuration["PickFromStoragePriority"],
                                          (self.findVirtualObject(obj.shape, obj.color),
                                           configuration["Observed_times"][f"Robot_{robotID}_Storage_to_Conveyor"] +
                                           configuration["Observed_times"][f"Robot_{robotID}_Conveyor_to_Obs"])))

        # Check if we can pop something from queue
        for robotID in range(len(self.queues)):
            prioQueue = self.queues[robotID]
            if (not prioQueue.empty() and self.timeSinceLastPops[robotID] > prioQueue.peek()[1]):
                virtualObj, _ = prioQueue.get()
                virtualObj.pickUpObject()
                self.timeSinceLastPops[robotID] = 0

        # Increment the time in all objects
        print(self.conveyors)
        new_conveyors = [False, False]
        for vObject in self.virtualObjects:
            robotID, conveyors, destination = vObject.IncrementInnerClock(self.rules[(vObject.shape, vObject.color)], self.conveyors)
            new_conveyors = [new_conveyors[0] or conveyors[0], new_conveyors[1] or conveyors[1]]
            if (robotID != None and destination != None):
                self.queues[robotID].put((configuration["PickFromIRSensorPriority"],
                                          (vObject,
                                           configuration["Observed_times"][f"Robot_{robotID}_Obs_to_IR"] +
                                           configuration["Observed_times"][f"Robot_{robotID}_Conveyor_to_{destination}"] +
                                           configuration["Observed_times"][f"Robot_{robotID}_{destination}_to_Obs"])))
        self.conveyors = new_conveyors
        # Increment time since lost pop timers
        for i in range(len(self.timeSinceLastPops)):
            self.timeSinceLastPops[i] += self.stepSize


    def event(self, event):
        self.events.put((event[0],self.objectToVirtualObject(event[1])))

    def setRules(self, rules):
        self.rules = rules
