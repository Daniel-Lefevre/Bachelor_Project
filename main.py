from GUI import *
from environment import configuration
from system import System
import threading

if __name__ == "__main__":
    IPs = configuration["ips"]
    positions = configuration["positions"]
    system = System(IPs, positions)
    t = threading.Thread(target=system.listenToIR, daemon=True)
    t.start()
    gui = GUI(system)
