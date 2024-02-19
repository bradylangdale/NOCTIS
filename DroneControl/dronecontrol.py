from dronehandler import DroneHandler
from zmqmanager import ZMQManager


# this is the entry point for the drone control software
if __name__ == "__main__":
    drone = DroneHandler()

    # init drone stream
    drone.start()
    zmqManager = ZMQManager(drone)
