'''
/*=========================================================================
pantilt.py
RoverPi pan-tilt servo control via PCA9685 PWM driver.
This module provides functions to control the pan and tilt angles of the ADUCAM 
servo using the PCA9685 PWM driver. It initializes the driver, sets the PWM frequency, 
and allows for adjusting the pan and tilt angles with simple functions.
Created on: Apr 27, 2026
Author: Michael Kane
=================================================================
'''

from  smbus2 import SMBus
import time

PCA9685_ADDR = 0x40
MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

PAN = 1
TILT = 0

PAN_CENTER = 90
TILT_CENTER = 90

bus = SMBus(1)

pan_angle = PAN_CENTER
tilt_angle = TILT_CENTER

def write(reg, value):
    bus.write_byte_data(PCA9685_ADDR, reg, value)

def set_pwm(channel, on, off):
    reg = LED0_ON_L + 4 * channel
    bus.write_byte_data(PCA9685_ADDR, reg, on & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, reg + 1, on >> 8)
    bus.write_byte_data(PCA9685_ADDR, reg + 2, off & 0xFF)
    bus.write_byte_data(PCA9685_ADDR, reg + 3, off >> 8)

def set_pwm_freq(freq):
    prescale = int(25000000.0 / (4096 * freq) - 1)
    oldmode = bus.read_byte_data(PCA9685_ADDR, MODE1)
    newmode = (oldmode & 0x7F) | 0x10
    write(MODE1, newmode)
    write(PRESCALE, prescale)
    write(MODE1, oldmode)
    time.sleep(0.005)
    write(MODE1, oldmode | 0x80)

def angle_to_pwm(angle):
    angle = max(0, min(180, angle))
    return int(120 + (angle / 180.0) * 500)

def set_angle(channel, angle):
    pwm = angle_to_pwm(angle)
    set_pwm(channel, 0, pwm)

def init():
    write(MODE1, 0x00)
    set_pwm_freq(50)
    center()

def center():
    global pan_angle, tilt_angle
    pan_angle = PAN_CENTER
    tilt_angle = TILT_CENTER
    set_angle(PAN, pan_angle)
    set_angle(TILT, tilt_angle)

def pan_left(step=5):
    global pan_angle
    pan_angle = max(0, pan_angle - step)
    set_angle(PAN, pan_angle)

def pan_right(step=5):
    global pan_angle
    pan_angle = min(180, pan_angle + step)
    set_angle(PAN, pan_angle)

def tilt_up(step=5):
    global tilt_angle
    tilt_angle = min(180, tilt_angle + step)
    set_angle(TILT, tilt_angle)

def tilt_down(step=5):
    global tilt_angle
    tilt_angle = max(0, tilt_angle - step)
    set_angle(TILT, tilt_angle)

if __name__ == "__main__":
    init()
    print("Centered.")
