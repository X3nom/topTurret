import sys, os, time
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import gyro, servoController

servo = servoController.Servo180(18)

mpu = gyro.mpu(0x68)
mpu.zeroGyro(0.3)

posX = 0
startT = time.time()
while True:
    data = mpu.getGyroZData()
    posX += data[2]*(time.time()-startT) #deg/sec * deltaTime
    startT = time.time()
    os.system("clear")
    print(data[2])
    print(posX)
