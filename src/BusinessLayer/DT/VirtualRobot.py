from resources.environment import configuration
from typing import Literal, TypeAlias
from queue import PriorityQueue
from src.BusinessLayer.DT.States import createRobotStates
from resources.PriorityQueue import CustomPriorityQueue

class VirtualRobot():
    def __init__(self, id, stepSize, conveyor):
        self.rules = {}
        self.queue = CustomPriorityQueue(configuration["NumberOfPriorities"])
        self.id = id
        self.states = createRobotStates(self.id)
        self.state = self.states["Observation"]
        self.stepSize = stepSize
        self.currentStateProgressGoal = float('inf')
        self.currentStateProgress = 0
        self.workingObject = None
        self.conveyor = conveyor

    def setRules(self, rules):
        self.rules = rules

    def addToQueue(self, priority, virtualObject):
        self.queue.put((priority, virtualObject))

    def step(self):
        if (self.state.key == "Observation"):
            if (not self.queue.empty()):
                self.workingObject = self.queue.get()
                destination = "Conveyor" if self.workingObject.state.origin == "IR" else self.workingObject.state.origin
                self.state = self.states[f"Observation_to_{destination}"]
                self.currentStateProgressGoal = self.state.time
                self.currentStateProgress = 0

        destination = None
        placedPosition = None

        if (self.currentStateProgress >= self.currentStateProgressGoal):
            destination, placedPosition = self.stateTransition()
            self.currentStateProgress = 0

        self.currentStateProgress += self.stepSize

        if (self.state.key == "Observation"):
            self.conveyor.start()
        else:
            self.conveyor.stop()

        return (self.workingObject, destination, placedPosition)

    def stateTransition(self):
        destination = None
        placedPosition = None

        if(self.state.key == "Observation_to_Storage"):
            destination = "Conveyor"
            self.state = self.states[f"Storage_to_{destination}"]

        elif(self.state.key == "Observation_to_Conveyor"):
            destination = self.rules[(self.workingObject.shape, self.workingObject.color)]
            self.state = self.states[f"Conveyor_to_{destination}"]

        elif(self.state.key == "Storage_to_Conveyor"):
            self.state = self.states["Conveyor_to_Observation"]
            placedPosition = "Conveyor"

        elif(self.state.key == "Storage_to_Observation"):
            self.state = self.states["Observation"]

        elif(self.state.key == "Conveyor_to_Conveyor"):
            self.state = self.states["Conveyor_to_Observation"]
            placedPosition = "Conveyor"

        elif(self.state.key == "Conveyor_to_Storage"):
            self.state = self.states["Storage_to_Observation"]
            placedPosition = "Storage"

        elif(self.state.key == "Conveyor_to_Observation"):
            self.state = self.states["Observation"]

        self.currentStateProgressGoal = self.state.time


        return (destination, placedPosition)
