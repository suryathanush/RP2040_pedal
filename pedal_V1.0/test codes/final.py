from machine import Pin, I2C,RTC, Timer, SPI
import utime
import ustruct as struct
import math
import lowpower
from imu import MPU6050
from neopixel import Neopixel
import time
import nrf24l01
import uasyncio as asyncio
from aswitch import Pushbutton
import _thread

numpix = 1
pixels = Neopixel(numpix, 0, 16, "GRB")
 
yellow = (255, 100, 0)
orange = (255, 50, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
red = (255, 0, 0)
black = (0,0,0)
 
pixels.brightness(50)
pixels.set_pixel(0, red)
pixels.show()

#configuration registers used
PWR_MGMT_1 = 0x6B
SIGNAL_PATH_RESET = 0x68
I2C_SLV0_ADDR = 0x37
ACCEL_CONFIG = 0x1C
MOT_THR = 0x1F # Motion detection threshold bits [7:0]
MOT_DUR = 0x20 # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
MOT_DETECT_CTRL = 0x69
INT_ENABLE = 0x38
WHO_AM_I_MPU6050 = 0x75 # Should return 0x68
INT_STATUS = 0x3A

IMU_PIN = 3
BUTTON = 2

int_pin = machine.Pin(IMU_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
button = Pin(BUTTON, Pin.IN, Pin.PULL_UP)

#device adress
MPU6050_ADDRESS = 0x69

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

RateRoll, RatePitch, RateYaw = 0, 0, 0
RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw = 0, 0, 0
RateCalibrationNumber = 1000
AccX, AccY, AccZ = 0, 0, 0
AngleRoll, AnglePitch = 0, 0
PedalAngle, RPM = 0, 0
LoopTimer = 0
prev_RateYaw, prev_RateRoll, prev_RatePitch = 0,0,0

calib_flag = False

# Slave pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Slave pauses an additional _SLAVE_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) master time to get into receive mode. The
# master may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_SLAVE_SEND_DELAY = const(10)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 5, "ce": 8}

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

def NRF_init():
    global nrf
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=16)
    auto_ack(nrf)
    nrf.set_power_speed(nrf24l01.POWER_3, nrf24l01.SPEED_1M)
    nrf.set_crc(2)
    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.stop_listening()
    
def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
 
async def btn_app():
    while(1):
        await asyncio.sleep(1)
   
def long_press():
    print("button long pressed")

def double_press():
    print("button double pressed")
    imu_calibrate()

def single_press():
    print("button single pressed")

# Creating the pushbutton instance
pb = Pushbutton(button, suppress=True)

# Pushbutton actions
pb.double_func(double_press)
pb.long_func(long_press)
pb.release_func(single_press)

def btn_thread():
    # Running the main loop
    loop = asyncio.get_event_loop()
    loop.run_until_complete(btn_app())


def gyro_fetch():
    global RateRoll, RatePitch, RateYaw, AccX, AccY, AccZ, AngleRoll, AnglePitch
    try:
        RateRoll = imu.gyro.x
        RatePitch = imu.gyro.y
        RateYaw = imu.gyro.z
        AccX = imu.accel.x
        AccY = imu.accel.y
        AccZ = imu.accel.z
    except OSError:
            print("OSError exception")
    AngleRoll = math.atan(AccY / math.sqrt(AccX * AccX + AccZ * AccZ)) * (180 / 3.142)
    AnglePitch = -math.atan(AccX / math.sqrt(AccY * AccY + AccZ * AccZ)) * (180 / 3.142)

def imu_calibrate():
    global LoopTimer, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw, calib_flag, pixels
    calib_flag = True
    print("calibrating...")
    #LoopTimer = time.ticks_us()
    timer_update()
    for _ in range(RateCalibrationNumber):
        gyro_fetch()
        RateCalibrationRoll += RateRoll
        RateCalibrationPitch += RatePitch
        RateCalibrationYaw += RateYaw
        time.sleep_us(1000)
    RateCalibrationRoll /= RateCalibrationNumber
    RateCalibrationPitch /= RateCalibrationNumber
    RateCalibrationYaw /= RateCalibrationNumber
    calib_flag = False
    print("done calibrating")


def imu_loop():
    global RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch, RPM, PedalAngle
    gyro_fetch()
    RateRoll -= RateCalibrationRoll
    RatePitch -= RateCalibrationPitch
    RateYaw -= RateCalibrationYaw
    
    PedalAngle = abs(AngleRoll)+abs(AnglePitch)
    RPM = RateYaw*0.166667
    #print("Roll Angle [°]:", KalmanAngleRoll, "Pitch Angle [°]:", KalmanAnglePitch)
    print("Roll Angle [°]:", AngleRoll, "Pitch Angle [°]:", AnglePitch)
    print("Angle :", PedalAngle, " RPM :", RPM)
    #time.sleep(0.1)
    while time.ticks_diff(time.ticks_us(), LoopTimer) < 4000:
        pass
    timer_update()
    
def timer_update():
    global LoopTimer
    LoopTimer = time.ticks_us()


def interrupt_callback(pin):
    print("interrputed")
    print(i2c.readfrom_mem(MPU6050_ADDRESS,0x3A ,  1))
    

int_pin.irq(trigger = machine.Pin.IRQ_FALLING, handler = interrupt_callback)

_thread.start_new_thread(btn_thread, ())

NRF_init()

while(1):
    if(not calib_flag):
        imu_loop()
        roll_mov = abs(prev_RateRoll)-abs(RateRoll)
        pitch_mov = abs(prev_RatePitch)-abs(RatePitch)
        yaw_mov = abs(prev_RateYaw)-abs(RateYaw)
        max_movement = max(abs(pitch_mov), abs(roll_mov), abs(yaw_mov))
        prev_RateRoll = RateRoll
        Prev_RatePitch = RatePitch
        prev_RateYaw = RateYaw
        
        try:
            pixels.fill(green)
            pixels.show()
            nrf.send(struct.pack("fff", time.ticks_ms(), PedalAngle, RPM))
            pixels.fill(blue)
            pixels.show()
            
        except OSError as e:
            print(e)
            pixels.fill(red)
            pixels.show()
            
        if(max_movement<2):
            print("momement less than 2")
            pixels.fill(black)
            pixels.show()
            lowpower.dormant_with_modes({
                IMU_PIN: lowpower.EDGE_LOW,
                BUTTON: lowpower.EDGE_LOW
                })
            print("woke up")
            print(i2c.readfrom_mem(MPU6050_ADDRESS,0x3A ,  1))
            pixels.fill(blue)
            pixels.show()
            NRF_init()
            timer_update()
    else:
        print("calibrating in progress")
        pixels.fill(red)
        pixels.show()
        time.sleep(0.5)
            
            
            
            
            
            