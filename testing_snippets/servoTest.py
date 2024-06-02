import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import servoController_external

controller = servoController_external.Controller([1000,1000], 0, 1, 2)


'''
for i in range(5):
    # controller.xServo.rotateDeg(10)
    controller.yServo.rotateDeg(0, True)
    time.sleep(2)
    # controller.xServo.rotateDeg(-10)
    controller.yServo.rotateDeg(30, True)
    time.sleep(2)
'''

while True:
    deg = int(input("deg: "))
    controller.yServo.rotateDeg(deg)

# controller.xServo.stop()

print("end")