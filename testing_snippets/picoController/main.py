import sys
from machine import Pin, PWM

'''
w(byte) <pin_number>(byte) <value>(byte string until \\n char)

'''
VALID_INSTRUCTIONS = ['w']

def add_pwm(pin) -> PWM:
    #logf.write(f"added on pin {pin}\n") #TODO RM
    global pwm_pins
    pwm = PWM(Pin(pin))
    pwm.freq(50) #50Hz
    pwm_pins.update({pin: pwm})


def set_duty_cycle(pin, value):
    #logf.write(f"set pwm on pin {pin} to {value}\n") #TODO RM
    global pwm_pins
    # min = 0; max = 65535
    pwm = pwm_pins[pin]
    pwm.duty_u16(value)




pwm_pins = {}

#logf = open("log.txt", 'a')

while True:
    #logf.flush()

    instruction = sys.stdin.read(1)

    #logf.write(f"instruction: {instruction}\n") #TODO: RM

    if instruction not in VALID_INSTRUCTIONS:
        continue

    # pin = sys.stdin.read(1)
    # pin = ord(pin)
    pin = sys.stdin.readline().strip()
    pin = int(pin)

    if instruction == 'w':
        if pin not in pwm_pins.keys(): #setup pwm for pin if not already done so
            add_pwm(pin)

        value = sys.stdin.readline().strip()
        value = int(value)
        set_duty_cycle(pin, value)