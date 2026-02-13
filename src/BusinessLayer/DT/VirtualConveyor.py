from resources.environment import configuration

class VirtualConveyor:
    def __init__(self, id):
        self.id = id
        self.running = True

    def start(self):
        self.running = True

    def stop(self):
        self.running = False


    def getInfo(self):
        return self.running
