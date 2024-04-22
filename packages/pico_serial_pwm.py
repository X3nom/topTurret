"""
handle communication with pico (external pwm driver)

servo boundaries:
mid low - 4700 //right
mid high - 5070 //left
"""
import serial, queue, threading

class Pico_serial_pwm():
    def __init__(self, port="COM6" ) -> None:
        self.port = serial.Serial(port)
    

    def set_duty_cycle(self, pin, duty):
        s = f"w{pin}\r{duty}\r".encode()
        self.port.write(s)
        self.port.flush()
        

    def init_threaded(self):
        self.pwm_queue = queue.Queue()
        self.scheduled = []
        self.thread = threading.Thread(target=self._check_thread)


    def set_duty_cycle_threaded(self, pin, duty, time=-1):
        self.pwm_queue.put((pin, duty, time))

    def _check_thread(self):
        while True:
            pwm = self.pwm_queue.get()

            self.set_duty_cycle(pwm[0], pwm[1])