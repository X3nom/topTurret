import sys

sys.path.append('../topTurret') #import from parent dir
from packages import pico_serial_pwm

pwm = pico_serial_pwm.Pico_serial_pwm('/dev/ttyACM0')

while True:
    inp = int(input("duty: "))
    pwm.set_duty_cycle(0, inp)