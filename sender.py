import serial
import time

# serialPort = serial.Serial(
#     port="COM4", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
# )
serialPort = serial.Serial(
    port="/dev/ttyACM0", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)


def waitForReady():
  line = serialPort.readline().strip()
  while (line != b'ready'):
    print(line)
    time.sleep(0.005)
    line = serialPort.readline().strip()


waitForReady()

# serialPort.write(bytes('X50 Y0\n', 'utf-8'))

while True:
  serialPort.write(bytes(input().strip() + '\n', 'utf-8'))
  waitForReady()


# line = serialPort.readline().strip()
# while (line != b'rasdfadseady'):
#   print(line)
#   time.sleep(0.005)
#   line = serialPort.readline().strip()

# with open('demo_program.txt', 'r') as f:
#     for line in f:
#         line = line.strip()
#         print("Sending: " + line)
#         serialPort.write(bytes(line + '\n', 'utf-8'))
#         waitForReady()
