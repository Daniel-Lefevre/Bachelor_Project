class VirtualConveyor:
    def __init__(self, id: int):
        self.id = id
        self.running = True

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def get_info(self) -> bool:
        return self.running
