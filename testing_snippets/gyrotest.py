from mpu6050 import mpu6050
import os, time, re
class zeroedGyro():
    def __init__(self, mpu):
        self.mpu = mpu
        self.zero = [0,0,0]
        self.reZero(0.3)

    def reZero(self, t):
        self.zero = [0,0,0]

        startT = time.time()
        j = 0
        while time.time()-startT < t:
            gyroData = self.mpu.get_gyro_data()
            gyro = [gyroData[x] for x in ['x','y','z']]
            self.zero = [self.zero[i] + gyro[i] for i in range(3)]
            j += 1

        self.zero = [self.zero[i]/j for i in range(3)]
    
    def getGyroData(self):
        gyroData = self.mpu.get_gyro_data()
        gyro = [gyroData[x] for x in ['x','y','z']]
        gyro = [gyro[i]-self.zero[i] for i in range(3)]
        return gyro

mpu = mpu6050(0x68)
zero = zeroedGyro(mpu)

while True:
    #gyroData = mpu.get_gyro_data()
    #gyro = [gyroData[x] for x in ['x','y','z']]

    gyro = zero.getGyroData()

    os.system("clear")
    for x in gyro:
        print(re.findall(r"[0-9]*\.[0-9]{,3}",str(x))[0])