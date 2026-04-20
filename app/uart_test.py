import serial
import time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

time.sleep(2)

print("Sending...")
ser.write(b'HELLO FROM PI\n')

while True:
    if ser.in_waiting:
        msg = ser.readline().decode(errors='ignore').strip()
        print("ESP32:", msg)
        break
