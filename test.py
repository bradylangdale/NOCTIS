import olympe
import os
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

DRONE_IP = os.environ.get("DRONE_IP", "10.202.0.1")


def test_moveby():
    drone = olympe.Drone(DRONE_IP)
    drone.connect()

    assert drone(
        TakeOff()
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    assert drone(
        moveBy(2, 0, 0, 0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    assert drone(
        moveBy(0, 0, 0, 3.14)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    assert drone(
        moveBy(2, 0, 0, 0)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    assert drone(
        moveBy(0, 0, 0, 3.14)
        >> FlyingStateChanged(state="hovering", _timeout=5)
    ).wait().success()

    assert drone(Landing()).wait().success()
    drone.disconnect()


if __name__ == "__main__":
    test_moveby()
