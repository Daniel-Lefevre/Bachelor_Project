from resources.environment import configuration

class VirtualConveyor:
    def __init__(self, id):
        self.id = id
        print("Turning on")
        self.running = True

    def start(self):
        self.running = True
        print("Turning on")

    def stop(self):
        self.running = False
        print("Turning off")


    def getInfo(self):
        return self.running
