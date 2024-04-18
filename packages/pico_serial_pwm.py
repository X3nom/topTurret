"""
handle communication with pico (external pwm driver)
"""

import serial

class Pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)
    
    def set_duty_cycle(self, pin, duty):
        p = chr(pin)
        s = f"w{p}{duty}\n".encode()
        self.port.write(s)
