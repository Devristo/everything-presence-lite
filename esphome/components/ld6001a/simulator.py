import serial
import time
import re

DEFAULTS = {
    "started": False,
    "debug": 0,
    "baud": 115200,
    "heatime": 60,
    "range": 450,
    "height": 300,
    "dpkth": 4,
    "XPosi": 450,
    "XNega": -450,
    "YPosi": 450,
    "YNega": -450,
    "Moving": 110,
    "Static": 100,
    "Exit": 5,
}

class SensorSimulator:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.ser = serial.Serial(port, baudrate=baud, timeout=1)
        self.state = DEFAULTS.copy()

    def write_sensor_data(self):
        # Example of a complete data package:
        # 55 AA 0A 04 00 00 00 00 00 0E
        # 55 AA : Frame header
        # 0A : Byte number 04 :
        # type=0x04 People
        # counting 00 00 :
        # Reserved
        # 00 00 : Reserved
        # 00 : Number of people counted
        # 0E : XOR check ( 0A 04 00 00 00 00 00 XOR calculation )

        if not self.state["started"]:
            return;
    
        if self.state["debug"] == 1:
            self.ser.write([
                0x55, 0xAA,
                0x0A,  # Byte number
                0x04,  # Type (0x04 for people counting)
                0x00, 0x00,  # Reserved
                0x00, 0x00,  # Reserved
                0x00,  # Number of people counted
                0x0E,  # XOR check
            ])
            return
        
        if self.state["debug"] == 2:
            self.ser.write([
                [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08], # Frame header,
                + [0x40, 0x00, 0x0, 0x0],                 # length
                + [0xA3, 0x01, 0x0, 0x0],              # Frame rate ?
                + [0x01, 0x00, 0x0, 0x0],                 # TLV1
                + [0x00, 0x00, 0x0, 0x0],                    # Point cloud length (always 0)
                + [0x02, 0x00, 0x0, 0x0],                 # TLV2
                + [0x20, 0x00, 0x0, 0x0],                 # Track length (= people count * 32 bytes)
                
                # Per person
                + [0x00, 0x00, 0x00, 0x00],                 # Reserved
                + [0x00, 0x00, 0x00, 0x00],                 # Person ID
                + [0x00, 0x00, 0x00, 0x00],                 # X coordinate
                + [0x00, 0x00, 0x00, 0x00],                 # Y coordinate
                + [0x00, 0x00, 0x00, 0x00],                 # Z coordinate
                + [0x00, 0x00, 0x00, 0x00],                 # Vx coordinate
                + [0x00, 0x00, 0x00, 0x00],                 # Vy coordinate
                + [0x00, 0x00, 0x00, 0x00],                 # V coordinate (should be Vz?)

                # Checksum, XOR all bytes from frame length until the end of the frame
                + [0xCC] 



            ])
            return


    def handle_command(self, line):
        line = line.strip()
        print(f"Received: {line}")

        if line == "AT+START":
            self.state["started"] = True
            return "OK\r\n"
        elif line == "AT+STOP":
            self.state["started"] = False
            return "OK\r\n"
        elif line == "AT+RESET":
            self.state = DEFAULTS.copy()
            return "OK\r\n"
        elif line == "AT+READ":
            return "\r\n".join([f"{k}={v}" for k, v in self.state.items()]) + "\r\n"
        elif line == "AT+RESTORE":
            self.state = DEFAULTS.copy()
            return "OK\r\n"

        # Handle config commands
        patterns = {
            "AT\\+DPKTH=(\\d+)": ("dpkth", int),
            "AT\\+BAUD=(\\d+)": ("baud", int),
            "AT\\+HEATIME=(\\d+)": ("heatime", int),
            "AT\\+RANGE=(\\d+)": ("range", int),
            "AT\\+HEIGHTD=(\\d+)": ("height", int),
            "AT\\+DEBUG=(\\d+)": ("debug", int),
            "AT\\+XPosi=(\\d+)": ("XPosi", int),
            "AT\\+XNega=(-\\d+)": ("XNega", int),
            "AT\\+YPosi=(\\d+)": ("YPosi", int),
            "AT\\+YNega=(-\\d+)": ("YNega", int),
            "AT\\+Moving=(\\d+)": ("Moving", int),
            "AT\\+Static=(\\d+)": ("Static", int),
            "AT\\+Exit=(\\d+)": ("Exit", int),
        }

        for pattern, (key, caster) in patterns.items():
            match = re.match(pattern, line)
            if match:
                value = caster(match.group(1))
                self.state[key] = value

                print(f"Set {key} to {value}")

                return "OK\r\n"

        return "ERROR\r\n"

    def run(self):
        print("Sensor simulator started. Listening for AT commands...")
        buffer = ""
        while True:
            if self.ser.in_waiting:
                char = self.ser.read().decode(errors="ignore")
                buffer += char
                if char == "\n":
                    response = self.handle_command(buffer.strip())
                    self.ser.write(response.encode())
                    buffer = ""
            
            time.sleep(self.state['heatime'] / 1000)  # Simulate delay based on heatime

            self.write_sensor_data()



if __name__ == "__main__":
    sim = SensorSimulator(port="/tmp/ttySIM")  # change to your virtual serial port
    sim.run()
