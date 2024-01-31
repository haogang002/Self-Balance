from imu import MPU6050
from utime import sleep, ticks_ms
from time import sleep
from machine import Pin, I2C
from math import atan2, degrees, pi

class PID():
    def __init__(self, p, i, d, target, minOut, maxOut):
        self.targetBias = 0
        self.target = target
        self.pK = p
        self.iK = i
        self.dK = d
        self.error = 0
        self.sumError = 0
        self.lastError = 0
        self.minOut = minOut
        self.maxOut = maxOut
        
    def update(self, now):
        self.error = (self.target + self.targetBias) - now
        self.sumError += self.error
        out = (self.error * self.pK) + (self.sumError * self.iK) + ((self.error - self.lastError) * self.dK)
        self.lastError = self.error
        
        out = self.minOut if out < self.minOut else out
        out = self.maxOut if out > self.maxOut else out
        return(out)
    
def readAngle(angles):
    global lastTime
    now = ticks_ms()
    gyro[0] = mpu.gyro.x
    gyro[1] = mpu.gyro.y
    gyro[2] = mpu.gyro.z
    acc = mpu.accel
#     angles[0] += (gyro[0] - gyroBias[0])*100000 /16384
    angles[1] -= (gyro[1] - gyroBias[1])*pi
    angles[2] += (gyro[2] - gyroBias[2])*pi
    
    #angles[0] += (degrees(atan2(acc[1], acc[2])) - angles[0]) * accK
    angles[1] += (degrees(atan2(acc[0], acc[2])) - angles[1]) * accK
    angles[2] += (degrees(atan2(acc[0], acc[1])) - angles[2]) * accK
    
    angles[0] = round(angles[0], 2)
    angles[1] = round(angles[1], 2)
    angles[2] = round(angles[2], 2)
    
    lastTime = now
    return(angles)


MPU6050_vcc = Pin(3, Pin.OUT)
MPU6050_vcc.value(1)

gyroBias = [-0.0235664, 0.003460463, 0.01118344]
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
mpu = MPU6050(i2c)

biasAngle = 0
targetAngle = 89
tagretRPM = 0

speed = 40
speedBias = 0
gyroPID = PID(2, 0.3, 0, targetAngle, -100, 100)
rpmPID = PID(.1, 0.0001, .00, tagretRPM, -12, 12)

moveDir = [0, 0]
angles = [0, 90, 90]
for i in range(20):
    angles = readAngle(angles)
errorSumA = 0
errorOldA = 0
errorSumR = 0
errorOldR = 0
filteredAngles = angles.copy()
f = 0.5
oldTick = ticks_ms()

filteredAngles = angles.copy()

while True:
    angles = readAngle(angles)
    nowTick = ticks_ms()
    if(nowTick - oldTick > 20):
        print(rpmPID.target)
        oldTick = nowTick
    filteredAngles[0] = (angles[0] * (1 - f)) + filteredAngles[0] * f
    filteredAngles[1] = (angles[1] * (1 - f)) + filteredAngles[1] * f
    filteredAngles[2] = (angles[2] * (1 - f)) + filteredAngles[2] * f
    
    if(40 > abs(filteredAngles[1] - targetAngle) > 0 and 1):
        gyroPID.targetBias = rpmPID.update(moveDir[0]) + speedBias
        moveDir[0] = -gyroPID.update(filteredAngles[1])
        print(moveDir[0])
        print(moveDir)
        motorL.setRPM(moveDir[0] + moveDir[1])
        motorR.setRPM(moveDir[0] - moveDir[1])
    else:
        motorR.stop()
        motorL.stop()
    sleep(0.1)