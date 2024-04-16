import sys, time
#import mpu6050 as mpu
from machine import Pin, PWM


pwm = PWM(Pin(0))
pwm.freq(50)

for i in range(1000, 9000, 10):
    print(i)
    pwm.duty_u16(i)
    time.sleep(0.1)