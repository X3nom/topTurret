"""
handle communication with pico (external pwm driver)
"""

import serial

class pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)
    
    def new_pwm(self, pin):
        p = chr(pin)
        s = f"n{p}".encode()
        self.port.write(s)

    def set_duty_cycle(self, pin, duty):
        p = chr(pin)
        s = f"w{p}{duty}\n".encode()
        self.port.write(s)


p = pico_serial_pwm("/dev/ttyACM0")

p.new_pwm(9)
p.set_duty_cycle(9, 2000)