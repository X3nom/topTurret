"""
handle communication with pico (external pwm driver)
"""

import serial, time

class Pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)
        self.port.flush()
    
    def set_duty_cycle(self, pin, duty):
        s = f"w{pin}\r{duty}\r".encode('utf-8')
        self.port.write(s)
        

pwm = Pico_serial_pwm('/dev/ttyACM0')
pwm.set_duty_cycle(0,2000)
time.sleep()
pwm.set_duty_cycle