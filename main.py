import threading

from resources.environment import configuration
from src.BusinessLayer.system import System
from src.PresentationLayer.GUI import GUI

if __name__ == "__main__":
    IPs = configuration["ips"]
    positions = configuration["positions"]
    system = System(IPs, positions)
    t = threading.Thread(target=system.set_up, daemon=True)
    t.start()
    gui = GUI(system)