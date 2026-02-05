from typing import Literal
from dataclasses import dataclass
from robot import *

@dataclass
class StorageObject:
    name: str
    shape: Literal[ObjectShape.CIRCLE, ObjectShape.SQUARE]
    color: Literal[ObjectColor.RED, ObjectColor.BLUE, ObjectColor.GREEN]
    position: Literal["Storage 1", "In Transit", "Storage 0"]

class System:
    def __init__(self, ips, positions):
        self.RobotArms = []
        self.running = True
        self.storageObjects = [
            StorageObject("Red Square", ObjectShape.SQUARE, ObjectColor.RED, "Storage 1"),
            StorageObject("Blue Square", ObjectShape.SQUARE, ObjectColor.BLUE, "Storage 0"),
            StorageObject("Green Square", ObjectShape.SQUARE, ObjectColor.GREEN, "Storage 1"),
            StorageObject("Red Circle", ObjectShape.CIRCLE, ObjectColor.RED, "Storage 0"),
            StorageObject("Blue Circle", ObjectShape.CIRCLE, ObjectColor.BLUE, "Storage 1"),
            StorageObject("Green Circle", ObjectShape.CIRCLE, ObjectColor.GREEN, "Storage 0"),
        ]


        # Add all the robot arms
        for i in range(len(ips)):
            ID = i
            IP_address = ips[i]
            poses = positions[i]
            self.RobotArms.append(RobotArm(IP_address, poses, ID))

    def robot_worker(self, arm):
        # Setup Phase
        print(f"Robot {arm.ID} is initializing")
        arm.setUp()
        print(f"Robot {arm.ID} setup finished")

        # 2. Monitoring Phase
        while self.running:
            arm.Loop()

    def listenToIR(self):
        self.threads = []

        # Start one thread per robot arm
        for arm in self.RobotArms:
            t = threading.Thread(target=self.robot_worker, args=(arm,))
            t.daemon = True
            self.threads.append(t)
            t.start()

        # Check for events to do
        while self.running:
            # Robot 0 grab object from storage
            if keyboard.is_pressed('0'):
                self.RobotArms[0].takeObjectFromStorage()
                time.sleep(0.5)

            # Robot 1 grab object from storage
            elif keyboard.is_pressed('1'):
                self.RobotArms[1].takeObjectFromStorage()
                time.sleep(0.5)

            # Robot 0 take image
            elif keyboard.is_pressed('9'):
                self.RobotArms[0].takeImage()
                time.sleep(0.5)

            # Robot 1 take image
            elif keyboard.is_pressed('2'):
                self.RobotArms[1].takeImage()
                time.sleep(0.5)

            # Shutdown event
            elif keyboard.is_pressed('s'):
                print("Shutting Down")
                self.running = False
            else:
                time.sleep(0.1)

        # Wait for the threads to finnish their task before shutting down
        for t in self.threads:
            t.join()

        # Disconnect the arms
        for arm in self.RobotArms:
            arm.disconnect()

        print("Everything has been shut down")

    def updateObject(self, shape, color, position):
        for i in range(len(self.storageObjects)):
            object = self.storageObjects[i]
            if (object.shape == shape and object.color == color):
                self.storageObjects[i].position = position

    def getObjects(self):
        # Retrieve updates from the robot arms
        for arm in self.RobotArms:
            for update in arm.getObjectUpdates():
                self.updateObject(*update)

        return self.storageObjects

    def findObjectByName(self, objectName):
        for object in self.storageObjects:
            if (object.name == objectName):
                return object

    def moveObject(self, name, destination):
        object = self.findObjectByName(name)

        # Tell to pick up from storage
        self.RobotArms[int(object.position[-1])].addToQueue(2, "Storage", object.shape, object.color)

        # Move to different storage
        if (object.position != destination):
            self.RobotArms[int(not int(object.position[-1]))].addRule(object.shape, object.color, "Storage")
        # Move to same storage
        else:
            self.RobotArms[int(not int(object.position[-1]))].addRule(object.shape, object.color, "Conveyor")
            self.RobotArms[int(object.position[-1])].addRule(object.shape, object.color, "Storage")

