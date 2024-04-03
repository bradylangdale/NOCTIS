import olympe
import olympe_deps as od
from olympe.messages.skyctrl.CoPiloting import setPilotingSource

if __name__ == "__main__":
    skyctrl = olympe.Drone("192.168.53.1", drone_type=od.ARSDK_DEVICE_TYPE_ANAFI_THERMAL)
    skyctrl.connect()
    skyctrl(setPilotingSource(source="Controller")).wait()
    skyctrl.disconnect()