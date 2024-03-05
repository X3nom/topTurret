import sys, os
sys.path.append('../topTurret') #import from parent dir
from packages.rpiControll import gyro

mpu = gyro.mpu()
mpu.zeroGyro()
posX = 0
while True:
    data = mpu.getGyroZData()
    posX += data[0]
    os.system("clear")
    print(posX)