from src.BusinessLayer.DT.VirtualObject import VirtualObject
from pyniryo import *
import time
from queue import Queue
from resources.PriorityQueue import CustomPriorityQueue
from resources.environment import configuration, StorageObject
from src.BusinessLayer.DT.VirtualConveyor import VirtualConveyor
from src.BusinessLayer.DT.VirtualRobot import VirtualRobot
class TimeBasedDT():

    def __init__(self, stepSize):
        self.stepSize = stepSize
        self.rules = {}
        self.virtualObjects = [self.objectToVirtualObject(obj) for obj in [configuration["StorageObjects"][3]]]
        self.events = Queue()
        self.virtualConveyors = [VirtualConveyor(id) for id in range(configuration["NumberOfConveyors"])]
        self.virtualRobots = [VirtualRobot(id, self.stepSize, self.virtualConveyors[id]) for id in range(configuration["NumberOfRobotArms"])]


    def objectToVirtualObject(self, object : StorageObject):
        return VirtualObject(object.shape, object.color, self.stepSize, int(object.position[-1]))

    def findVirtualObject(self, shape, color):
        for object in self.virtualObjects:
            if (object.shape == shape and object.color == color):
                return object

    def step(self):
        # If an event has occured since last step
        while (not self.events.empty()):
            eventType, obj = self.events.get()
            robotID = int(obj.state.id)
            if (eventType == "Pick Up"):
                print("Event pick up")
                self.virtualRobots[robotID].addToQueue(configuration["PickFromStoragePriority"], self.findVirtualObject(obj.shape, obj.color))

        workingObjectsInfo = []
        # Increment the time in all objects
        for i in range(len(self.virtualRobots)):
            virtualRobot = self.virtualRobots[i]
            print(f"Robot{virtualRobot.id}: {virtualRobot.state.key}")
            workingObject, pickUpDestination, placedPosition = virtualRobot.step()
            if (pickUpDestination != None or placedPosition != None):
                workingObjectsInfo.append((workingObject, pickUpDestination, placedPosition))

        for virtualObject in self.virtualObjects:
            print(f"{virtualObject.state.key}")
            pickUpDestination = None
            placedPosition = None
            for Info in workingObjectsInfo:
                if Info[0].state.key == virtualObject.state.key:
                    pickUpDestination = Info[1]
                    placedPosition = Info[2]

            ID = virtualObject.state.id
            conveyorRunning = self.virtualConveyors[ID].getInfo()
            virtualObject.step(pickUpDestination, placedPosition, conveyorRunning)
            # Check if virtual object has reached in IR sensor
            if (virtualObject.hasReachedIR):
                self.virtualRobots[ID].addToQueue(configuration["PickFromIRSensorPriority"], virtualObject)
                virtualObject.hasReachedIR = False

        print("-------------------")

    def createEvent(self, event):
        self.events.put((event[0], self.objectToVirtualObject(event[1])))

    def setRules(self, rules):
        # Set the rules on the virtual robot arms
        for i in range(len(rules)):
            self.virtualRobots[i].setRules(rules[i])
