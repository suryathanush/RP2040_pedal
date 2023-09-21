from machine import I2C, Pin
import utime as time
import math
from imu import MPU6050

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

RateRoll, RatePitch, RateYaw = 0, 0, 0
RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw = 0, 0, 0
RateCalibrationNumber = 2000
AccX, AccY, AccZ = 0, 0, 0
AngleRoll, AnglePitch = 0, 0
LoopTimer = 0
KalmanAngleRoll, KalmanUncertaintyAngleRoll = 0, 2*2
KalmanAnglePitch, KalmanUncertaintyAnglePitch = 0, 2*2
Kalman1DOutput = [0, 0]

def kalman_1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement):
    global Kalman1DOutput
    KalmanState += 0.004 * KalmanInput
    KalmanUncertainty += 0.004 * 0.004 * 4 * 4
    KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3)
    KalmanState += KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    Kalman1DOutput[0] = KalmanState
    Kalman1DOutput[1] = KalmanUncertainty

def gyro_signals():
    global RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ, AngleRoll, AnglePitch
    RateRoll = imu.gyro.x
    RatePitch = imu.gyro.y
    RateYaw = imu.gyro.z
    AccX = imu.accel.x
    AccY = imu.accel.y
    AccZ = imu.accel.z
    AngleRoll = math.atan(AccY / math.sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.142)
    AnglePitch = -math.atan(AccX / math.sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.142)

def setup():
    global LoopTimer, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYawz
    print("Initializing...")
    LoopTimer = time.ticks_us()
    for _ in range(RateCalibrationNumber):
        gyro_signals()
        RateCalibrationRoll += RateRoll
        RateCalibrationPitch += RatePitch
        RateCalibrationYaw += RateYaw
        time.sleep_us(1000)
    RateCalibrationRoll /= RateCalibrationNumber
    RateCalibrationPitch /= RateCalibrationNumber
    RateCalibrationYaw /= RateCalibrationNumber

def loop():
    global RateRoll, RatePitch, RateYaw, KalmanAngleRoll, KalmanUncertaintyAngleRoll, AngleRoll, KalmanAnglePitch, KalmanUncertaintyAnglePitch, AnglePitch, LoopTimer
    gyro_signals()
    RateRoll -= RateCalibrationRoll
    RatePitch -= RateCalibrationPitch
    RateYaw -= RateCalibrationYaw
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll)
    KalmanAngleRoll = Kalman1DOutput[0]
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1]
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch)
    KalmanAnglePitch = Kalman1DOutput[0]
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1]
    print("Roll Angle [°]:", KalmanAngleRoll, "Pitch Angle [°]:", KalmanAnglePitch)
    while time.ticks_diff(time.ticks_us(), LoopTimer) < 4000:
        pass
    LoopTimer = time.ticks_us()

setup()
while True:
    loop()
