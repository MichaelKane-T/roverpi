import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2)

ser.write(b'PING\n')

while True:
    if ser.in_waiting:
        print(ser.readline().decode())
