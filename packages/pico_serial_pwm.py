"""
handle communication with pico (external pwm driver)

servo boundaries:
mid low - 4700 //right
mid high - 5070 //left
"""
import serial

class Pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)
    
    def set_duty_cycle(self, pin, duty):
        s = f"w{pin}\r{duty}\r".encode()
        self.port.write(s)
        self.port.flush()

