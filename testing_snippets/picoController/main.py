import sys
from machine import Pin, PWM
import mpu6050 #if not working, upload the library to pico

'''
w(byte) <pin_number>(byte) <value>(byte string until \\n char)

'''

def add_pwm(pin) -> PWM:
    global pwm_pins
    pwm = PWM(Pin(pin))
    pwm.freq(50) #50Hz
    pwm_pins.update({pin: pwm})


def set_duty_cycle(pin, value):
    global pwm_pins
    # min = 0; max = 65535
    pwm = pwm_pins[pin]
    pwm.duty_u16(value)





pwm_pins = {}


while True:

    instruction = sys.stdin.read(1)

    pin = sys.stdin.read(1)
    pin = ord(pin)

    if instruction == 'w':
        if pin not in pwm_pins.keys(): #setup pwm for pin if not already done so
            add_pwm(pin)

        value = sys.stdin.readline().strip()
        value = int(value) # ms to ns
        set_duty_cycle(pin, value)