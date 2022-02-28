from src.evo import Evo
from src.vehicle_finder import VehicleFinder
import serial
import serial.tools.list_ports
import threading
import sys

# rangefinder
def tof_thread():
    # main thread code
    evo_obj = Evo()

    print("Starting Evo data streaming")
    # Get the port the evo has been connected to
    port = evo_obj.findEvo()

    if port is None:
        print("ERROR: Couldn't find the Evo. Exiting.")
        sys.exit()
    else:
        evo = evo_obj.openEvo(port)

    # main loop
    while True:
        try:
            dist = evo_obj.get_evo_range(evo)
            print(f"{dist} m")
        except serial.serialutil.SerialException:
            print("ERROR: Device disconnected (or multiple access on port). Exiting...")


print("-----------STARTING---------------")
arduino = serial.Serial(port="COM4", baudrate=115200, timeout=0.1)

vehicle_finder = VehicleFinder()
cvThread = threading.Thread(target=vehicle_finder.cv_thread)
serialThread = threading.Thread(
    target=vehicle_finder.serial_thread,
    args=(vehicle_finder.pq, vehicle_finder.yq, arduino),
)
evoThread = threading.Thread(target=tof_thread, args=())

cvThread.start()
serialThread.start()
evoThread.start()
