"""
handle communication with pico (external pwm driver)
"""

import serial

class pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)

    def new_pwm(self, pin):
        p = chr(pin)
        self.port.write(f"n{p}")

    def set_duty_cycle(self, pin, duty):
        p = chr(pin)
        self.port.write(f"w{p}{duty}\n")
