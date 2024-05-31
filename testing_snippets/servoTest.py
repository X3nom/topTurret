import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import servoController_external

controller = servoController_external.Controller([1000,1000], 0, 1, 2)

for i in range(1):
    controller.xServo.rotateDeg(10)
    # controller.yServo.rotateDeg(10)
    time.sleep(1)
    controller.xServo.rotateDeg(-10)
    # controller.yServo.rotateDeg(-10)
    time.sleep(1)


# controller.xServo.stop()

print("end")