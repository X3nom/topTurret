from mpu6050 import mpu6050
import time

class mpu(mpu6050): #able to zero itself
    def __init__(self, address=0x68, bus=1):
        super.__init__(self, address, bus)
        self.gZero = [0,0,0]
        self.zeroGyro(0.1)


    def zeroGyro(self,t=0.1): # re zero self
        self.gZero = [0,0,0]

        startT = time.time()
        j = 0
        while time.time()-startT < t:
            gyroDataDict = self.get_gyro_data()
            gyroData = [gyroDataDict[x] for x in ['x','y','z']]
            self.gZero = [self.gZero[i] + gyroData[i] for i in range(3)]
            j += 1

        self.gZero = [self.gZero[i]/j for i in range(3)]
    

    def getGyroZData(self): # Get zeroed data from gyroscope as array [x, y, z]
        gyroDataDict = self.get_gyro_data()
        gyroData = [gyroDataDict[x] for x in ['x','y','z']]

        gyroData = [gyroData[i]-self.gZero[i] for i in range(3)]
        return gyroData

