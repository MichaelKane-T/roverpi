import time
import smbus2 as smbus

ADDR = 0x40

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

PAN = 1
TILT = 0

bus = smbus.SMBus(1)

def write(reg, val):
    bus.write_byte_data(ADDR, reg, val)

def read(reg):
    return bus.read_byte_data(ADDR, reg)

def set_pwm(ch, on, off):
    reg = LED0_ON_L + 4 * ch
    bus.write_byte_data(ADDR, reg, on & 0xFF)
    bus.write_byte_data(ADDR, reg + 1, on >> 8)
    bus.write_byte_data(ADDR, reg + 2, off & 0xFF)
    bus.write_byte_data(ADDR, reg + 3, off >> 8)

def set_freq(freq_hz):
    prescale = int(25000000.0 / (4096 * freq_hz) - 1)

    oldmode = read(MODE1)
    sleepmode = (oldmode & 0x7F) | 0x10

    write(MODE1, sleepmode)
    time.sleep(0.005)

    write(PRESCALE, prescale)

    write(MODE1, oldmode)
    time.sleep(0.005)

    write(MODE1, oldmode | 0xA1)  # restart + auto-increment + allcall
    time.sleep(0.005)

def angle(ch, a):
    if a < 0:
        a = 0
    if a > 180:
        a = 180

    pulse = int(120 + (a / 180.0) * 500)
    set_pwm(ch, 0, pulse)

print("Initializing PCA9685...")
write(MODE1, 0x00)
write(MODE2, 0x04)
time.sleep(0.01)

set_freq(50)

print("Centering...")
angle(PAN, 90)
angle(TILT, 90)
time.sleep(1)

print("Testing pan...")
for a in [60, 90, 120, 90]:
    angle(PAN, a)
    time.sleep(0.7)

print("Testing tilt...")
for a in [60, 90, 120, 90]:
    angle(TILT, a)
    time.sleep(0.7)

print("Done.")
