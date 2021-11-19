import serial
import time

serialPort = serial.Serial(
    port="COM4", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE
)


def waitForReady():
    line = serialPort.readline().strip()
    while (line != b'ready'):
        print(line)
        time.sleep(0.005)
        line = serialPort.readline().strip()


waitForReady()

with open('demo_program.txt', 'r') as f:
    for line in f:
        line = line.strip()
        print("Sending: " + line)
        serialPort.write(bytes(line + '\n', 'utf-8'))
        waitForReady()
