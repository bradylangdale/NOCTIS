import zmq
from threading import Thread

class ZMQManager:

    def __init__(self, drone):
        self.drone = drone
        self.running = True

        self.zmq_thread = Thread(target=self._zmqManager)
        self.zmq_thread.run()

    def _zmqManager(self):
        print("Connecting to the Web UI…")
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")

        while(self.running):
            message = socket.recv().decode("utf-8")

            #print(message)

            if message == 'Hello!':
                socket.send_string('Server Says Hi!')

                self.drone.stop()
                self.drone.start()
                t = Thread(target=self.drone.fly)
                t.start()
            elif message == 'GetCurrentFrame':
                #try:
                    #print(drone.current_frame)
                if self.drone.current_frame != None:
                    socket.send_string(self.drone.current_frame.decode('iso-8859-1'))
                else:
                    socket.send_string('Not Ready')
            else:
                socket.send_string('Acknowledged')

        socket.close()
