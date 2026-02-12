from src.BusinessLayer.DT.States import ObjectStates
class VirtualObject:
    def __init__(self, shape, color, stepSize, startingStorageID):
        self.states = ObjectStates
        self.state = self.states[f"Storage_{startingStorageID}"]
        self.color = color
        self.shape = shape
        self.currentStateProgressGoal = float('inf')
        self.currentStateProgress = 0
        self.stepSize = stepSize
        self.hasReachedIR = False

    def step(self, pickUpDestination, placedPosition, conveyorRunning):
        if (pickUpDestination):
            origin = "Conveyor" if self.state.origin == "IR" else "Storage"
            self.state = self.states[f"Robot_{self.state.id}_{origin}_to_{pickUpDestination}"]
            self.currentStateProgressGoal = float('inf')
            self.currentStateProgress = 0
        elif (placedPosition == "Conveyor"):
            opposite_id = int(not self.state.id)
            self.state = self.states[f"Conveyor_{opposite_id}"]
            self.currentStateProgressGoal = self.state.time
            self.currentStateProgress = 0
        elif (placedPosition == "Storage"):
            self.state = self.states[f"Storage_{self.state.id}"]
            self.currentStateProgressGoal = float('inf')
            self.currentStateProgress = 0
        elif (self.currentStateProgress > self.currentStateProgressGoal):
            self.state = self.states[f"IR_{self.state.id}"]
            self.currentStateProgressGoal = float('inf')
            self.currentStateProgress = 0
            self.hasReachedIR = True

        # Increment time if conveyor belt is running
        if (conveyorRunning):
            self.currentStateProgress += self.stepSize
