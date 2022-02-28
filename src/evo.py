from socket import timeout
import serial
import serial.tools.list_ports
import crcmod.predefined

# default evo functions
class Evo:
    # error code
    MAX_LIMIT_ERR_CODE = 65535  # Sensor measuring above its maximum limit
    UNABLE_TO_MEASURE_ERR_CODE = 1  # Sensor not able to measure
    BELOW_MIN_RANGE_ERR_CODE = 0  # Sensor detecting object below minimum range

    # fixed rate
    BAUD_RATE = 115200
    TIME_OUT = 2
    SET_TO_BINARY = (0x00, 0x11, 0x02, 0x4C)  # Send the command "Binary mode"

    def __init__(self):
        self.ports = list(serial.tools.list_ports.comports())

    def findEvo(self):
        # Find Live Ports, return port name if found, NULL if not
        print("Scanning all live ports on this PC")
        for p in self.ports:
            # print p # This causes each port's information to be printed out.
            if "5740" in p[2]:
                print("Evo found on port " + p[0])
                return p[0]

        print("NULL")
        return None

    def openEvo(self, portname) -> serial.Serial:
        print("Attempting to open port {}".format(portname))
        # Open the Evo and catch any exceptions thrown by the OS
        evo = serial.Serial(portname, baudrate=self.BAUD_RATE, timeout=self.TIME_OUT)

        # Flush in the buffer
        evo.flushInput()

        # Write the binary command to the Evo
        evo.write(self.SET_TO_BINARY)

        # Flush out the buffer
        evo.flushOutput()

        print("--- Serial port opened ---")
        return evo

    def get_evo_range(self, evo_serial):
        crc8_fn = crcmod.predefined.mkPredefinedCrcFun("crc-8")
        # Read one byte
        data = evo_serial.read(1)
        if data == b"T":
            # After T read 3 bytes
            frame = data + evo_serial.read(3)
            if frame[3] == crc8_fn(frame[0:3]):
                # Convert binary frame to decimal in shifting by 8 the frame
                rng = frame[1] << 8
                rng = rng | (frame[2] & 0xFF)
            else:
                print(
                    "CRC mismatch. Check connection or make sure only one progam access the sensor port."
                )
                return
        # Check special cases (limit values)
        else:
            print("Wating for frame header")
            return

        # Checking error codes
        if rng == self.MAX_LIMIT_ERR_CODE:
            return float("inf")
        elif rng == self.UNABLE_TO_MEASURE_ERR_CODE:
            return float("nan")
        elif rng == self.BELOW_MIN_RANGE_ERR_CODE:
            return -float("inf")
        else:
            # Convert frame in meters
            return rng / 1000.0
