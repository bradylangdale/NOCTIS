from zmqmanager import ZMQManager


# this is the entry point for the drone control software
if __name__ == "__main__":
    # TODO: determine if this should just be removed and make zmq the entrypoint
    zmqManager = ZMQManager()
