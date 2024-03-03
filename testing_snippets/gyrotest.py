from mpu6050 import mpu6050
import os, time, re
mpu = mpu6050(0x68)

while True:
    gyroData = mpu.get_gyro_data()
    gyro = [gyroData[x] for x in ['x','y','z']]

    os.system("clear")
    for x in gyro:
        print(re.findall(r"[0-9]*\.[0-9]{,3}",str(x))[0])