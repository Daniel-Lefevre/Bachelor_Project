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
        self.virtualObjects = [self.objectToVirtualObject(obj) for obj in configuration["StorageObjects"]]
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
        conveyorIDToBeLeft = None
        # If an event has occured since last step
        while (not self.events.empty()):
            eventType, obj = self.events.get()
            if (eventType == "Pick Up"):
                robotID = int(obj.state.id)
                self.virtualRobots[robotID].addToQueue(configuration["PickFromStoragePriority"], self.findVirtualObject(obj.shape, obj.color))
            elif (eventType == "IR"):
                conveyorIDToBeLeft = obj
            else:
                print(f"Unknown Event: {eventType}")

        workingObjectsInfo = []
        # Increment the time in all objects
        for i in range(len(self.virtualRobots)):
            virtualRobot = self.virtualRobots[i]
            workingObject, pickUpDestination, placedPosition = virtualRobot.step()
            if (pickUpDestination != None or placedPosition != None):
                workingObjectsInfo.append((workingObject, pickUpDestination, placedPosition))

        for virtualObject in self.virtualObjects:
            if (virtualObject.color == ObjectColor.BLUE and virtualObject.shape == ObjectShape.CIRCLE):
                print(f"Blue Circle: {virtualObject.state.key}")
            if (virtualObject.color == ObjectColor.RED and virtualObject.shape == ObjectShape.CIRCLE):
                print(f"Red Cirlce: {virtualObject.state.key}")
            pickUpDestination = None
            placedPosition = None
            for Info in workingObjectsInfo:
                if Info[0] == virtualObject:
                    pickUpDestination = Info[1]
                    placedPosition = Info[2]

            ID = virtualObject.state.id
            conveyorRunning = self.virtualConveyors[ID].getInfo()
            virtualObject.step(pickUpDestination, placedPosition, conveyorRunning, conveyorIDToBeLeft)
            # Check if virtual object has reached in IR sensor
            if (virtualObject.hasReachedIR):
                self.virtualRobots[ID].addToQueue(configuration["PickFromIRSensorPriority"], virtualObject)
                virtualObject.hasReachedIR = False

        print("-------------------")

    def createEvent(self, event):
        eventype = event[0]
        if (eventype == "Pick Up"):
            self.events.put((eventype, self.objectToVirtualObject(event[1])))
        else:
            self.events.put(event)


    def setRules(self, rules):
        # Set the rules on the virtual robot arms
        for i in range(len(rules)):
            self.virtualRobots[i].setRules(rules[i])
