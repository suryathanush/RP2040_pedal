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
import uos

file_path = "rx_pair.bin"

pairing_pipe_Tx = b"\xe1\xf0\xf0\xf0\xf0"
pairing_pipe_Rx = b"\xe2\xf0\xf0\xf0\xf0"

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

sleep_flag = False

core2_status = True

bat_v = 0.0
prev_time = 0
prev_wake_time= 0
prev_pair_time = 0

color1 = (0,0,0)
color2 = (0,0,0)
blk_time = 0
blink = False

# Slave pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Slave pauses an additional _SLAVE_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) master time to get into receive mode. The
# master may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_SLAVE_SEND_DELAY = const(10)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 5, "ce": 8}

Tx_ID = b"\x00"
Rx_ID = b"\x00"

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
#pipes = (pairing_pipe_Tx, b"\xd2\xf0\xf0\xf0\xf0")

def check_paired():
    try:
        # Check if the file exists
        file_size = uos.stat(file_path)[6]
        if file_size > 0:
            print("File exists.")
            return True
        else:
            print("File is empty.")
            return False
    except OSError as e:
        if e.args[0] == 2:  # ENOENT: No such file or directory
            print("File does not exist.")
            return False
    
def Generate_TxID():
    global Tx_ID
    print("generating Tx_ID")
    UID = machine.unique_id()
    # Extract the last 5 elements from the hex array
    last_5_elements = UID[-5:]
    # Convert the last 5 elements to bytes
    Tx_ID = bytes(last_5_elements)
    print("Tx pipe ID:", Tx_ID)

def save_Rx(byte_array):
    with open(file_path, 'w') as f:
        # Write the byte array to the file
        f.write(byte_array)
    
    
def NRF_init():
    global nrf
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    if(Tx_ID==pairing_pipe_Tx):
        nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=5)
    else:
        nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=16)
    auto_ack(nrf)
    nrf.set_power_speed(nrf24l01.POWER_3, nrf24l01.SPEED_1M)
    nrf.set_crc(2)
    nrf.open_tx_pipe(Tx_ID)
    nrf.open_rx_pipe(1, Rx_ID)
    if(Tx_ID==pairing_pipe_Tx):
        print("pairing mode, waiting for message")
        nrf.start_listening()
    else:
        print("normal mode, sending data")
        nrf.stop_listening()
    
def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
 
def get_bat():
    global bat_v
    adc = machine.ADC(28)  # Using GPIO 26 as the ADC pin
    total_value = 0
    count = 10
    for _ in range(count):
        total_value += adc.read_u16()
        utime.sleep_ms(1)  # Delay between readings
    bat_v = (total_value / count)*2*3.4/65535

def Blink(Color1, Color2, Time):
    global blink, color1, color2, blk_time
    if(Time>0):
        blink = True
    else:
        blink = False
    color1 = Color1
    color2 = Color2
    blk_time = Time
    
async def btn_app():
    core2_status = True
    while(not sleep_flag):
        while(1):
            if(blink):
                print("**************************")
                pixels.fill(color1)
                pixels.show()
                time.sleep_ms(blk_time)
                pixels.fill(color2)
                pixels.show()
                time.sleep_ms(blk_time)
            await asyncio.sleep_ms(100)
    core2_status = False
    
def long_press():
    global Tx_ID, prev_pair_time
    prev_pair_time = time.ticks_ms()
    print("button long pressed")
    Tx_ID=pairing_pipe_Tx
    NRF_init()

def double_press():
    print("button double pressed")
    imu_calibrate()

def single_press():
    print("button single pressed")


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
    global calib_flag, RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw, calib_flag, pixels
    calib_flag = True
    print("calibrating...")
    #LoopTimer = time.ticks_us()
    timer_update()
    for _ in range(RateCalibrationNumber):
        gyro_fetch()
        RateCalibrationRoll += RateRoll
        RateCalibrationPitch += RatePitch
        RateCalibrationYaw += RateYaw
        time.sleep_us(100)
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
    

def NRF_send_barray(barray):
    nrf.stop_listening()
    try:
        nrf.send(barray)
        print("sent response: ", barray)
        Tx_ID = pairing_pipe_Tx
    except OSError:
        pass
    
def setup():
    global Tx_ID, Rx_ID, prev_time, prev_wake_time, prev_pair_time, pb
    
    Rx_ID = pairing_pipe_Rx
    if(not check_paired()):
        print("no pairing data found")
        Tx_ID = pairing_pipe_Tx
    elif(check_paired):
        Generate_TxID()
    print(Tx_ID)
    print(Rx_ID)
    NRF_init()
    
    # Creating the pushbutton instance
    pb = Pushbutton(button, suppress=True)
    # Pushbutton actions
    pb.double_func(double_press)
    pb.long_func(long_press)
    pb.release_func(single_press)
    
    int_pin.irq(trigger = machine.Pin.IRQ_FALLING, handler = interrupt_callback)

    _thread.start_new_thread(btn_thread, ())
    
    timer_update()
    
    prev_time = time.ticks_ms()
    prev_wake_time = time.ticks_ms()
    prev_pair_time = time.ticks_ms()

setup()

while(1):
    if(time.ticks_diff(time.ticks_ms(), prev_time) > 60000):
        prev_time = time.ticks_ms()
        get_bat()
        print("bat_value:", bat_v)
    
    if(Tx_ID != pairing_pipe_Tx):
        if(not calib_flag):
            imu_loop()
            roll_mov = abs(prev_RateRoll)-abs(RateRoll)
            pitch_mov = abs(prev_RatePitch)-abs(RatePitch)
            yaw_mov = abs(prev_RateYaw)-abs(RateYaw)
            max_movement = max(abs(pitch_mov), abs(roll_mov), abs(yaw_mov))
            prev_RateRoll = RateRoll
            Prev_RatePitch = RatePitch
            prev_RateYaw = RateYaw
            
            if((max_movement<2) and (not calib_flag) and (Tx_ID != pairing_pipe_Tx)):
                if(time.ticks_diff(time.ticks_ms(), prev_wake_time) > 5000):
                    prev_wake_time = time.ticks_ms()
                    sleep_flag = True
                    imu.cycle_sleep()
                    time.sleep(0.01)
                    print("movement less than 2")
                    pixels.fill(black)
                    pixels.show()
                    lowpower.dormant_with_modes({
                        IMU_PIN: lowpower.EDGE_LOW,
                        BUTTON: lowpower.EDGE_LOW
                        })
                    print("woke up")
                    imu.wake()
                    sleep_flag = False
                    get_bat()
                    print("battery voltage:", bat_v)
                    print(i2c.readfrom_mem(MPU6050_ADDRESS,0x3A ,  1))
                    pixels.fill(blue)
                    pixels.show()
                    if(not core2_status):
                        pass
                        _thread.start_new_thread(btn_thread, ())
                    NRF_init()
                    timer_update()
            
            else:
                try:
                    pixels.fill(green)
                    pixels.show()
                    nrf.send(struct.pack("ffff", time.ticks_ms(), PedalAngle, RPM, bat_v))
                    pixels.fill(blue)
                    pixels.show()
                    
                except OSError as e:
                    print(e)
                    pixels.fill(red)
                    pixels.show()
                
        elif(calib_flag):
            print("calibrating in progress")
            pixels.fill(red)
            pixels.show()
            time.sleep(0.5)
            
    elif(Tx_ID == pairing_pipe_Tx):
        if(time.ticks_diff(time.ticks_ms(), prev_pair_time) < 10000):
            Blink(orange,black,100)
            if nrf.any():
                while nrf.any():
                    buf = nrf.recv()
                    print("received:", buf)
                    if(buf != b'\xf0\xf0\xf0\xf0\xf0'):
                        save_Rx(buf)
                    else:
                        Generate_TxID()
                        Blink(orange,black,0)
                        pixels.fill(green)
                        pixels.show()
                        time.sleep_ms(500)
                        print("paired, entering normal mode")
                        NRF_init()
                    utime.sleep_ms(_RX_POLL_DELAY)
                
                if(Tx_ID == pairing_pipe_Tx):
                    utime.sleep_ms(_SLAVE_SEND_DELAY)
                    Generate_TxID()
                    nrf.stop_listening()
                    NRF_send_barray(Tx_ID)
                    nrf.start_listening()
        else:
            prev_pair_time = time.ticks_ms()
            Generate_TxID()
            Blink(orange,black,0)
            print("timeout, exiting pairing mode")

