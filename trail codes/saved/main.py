#import nrf24l01test
import usys
import ustruct as struct
import utime
from machine import Pin, SPI, I2C
import nrf24l01
from micropython import const
from imu import MPU6050
import math
import array, time
import rp2

buz=Pin(10,Pin.OUT)

# Configure the number of WS2812 LEDs.
NUM_LEDS = 1
PIN_NUM = 16
brightness = 0.2

@rp2.asm_pio(sideset_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=24)
def ws2812():
    T1 = 2
    T2 = 5
    T3 = 3
    wrap_target()
    label("bitloop")
    out(x, 1)               .side(0)    [T3 - 1]
    jmp(not_x, "do_zero")   .side(1)    [T1 - 1]
    jmp("bitloop")          .side(1)    [T2 - 1]
    label("do_zero")
    nop()                   .side(0)    [T2 - 1]
    wrap()


# Create the StateMachine with the ws2812 program, outputting on pin
sm = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))

# Start the StateMachine, it will wait for data on its FIFO.
sm.active(1)

# Display a pattern on the LEDs via an array of LED RGB values.
ar = array.array("I", [0 for _ in range(NUM_LEDS)])

##########################################################################
def pixels_show():
    dimmer_ar = array.array("I", [0 for _ in range(NUM_LEDS)])
    for i,c in enumerate(ar):
        r = int(((c >> 8) & 0xFF) * brightness)
        g = int(((c >> 16) & 0xFF) * brightness)
        b = int((c & 0xFF) * brightness)
        dimmer_ar[i] = (g<<16) + (r<<8) + b
    sm.put(dimmer_ar, 8)
    time.sleep_ms(10)

def pixels_set(i, color):
    ar[i] = (color[1]<<16) + (color[0]<<8) + color[2]

def pixels_fill(color):
    for i in range(len(ar)):
        pixels_set(i, color)

BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
COLORS = (BLACK, RED, YELLOW, GREEN, CYAN, BLUE, PURPLE, WHITE)


i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

accCoef = 0.02
gyroCoef = 0.98
angleGyroX = 0
angleGyroY = 0
angleGyroZ = 0
angleX = 0
angleZ = 0
angleY = 0
gyroXoffset = 0
gyroYoffset = 0
gyroZoffset = 0

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 57.295779513082 # 1 rad/s is 57.295779578552 deg/s

# Slave pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Slave pauses an additional _SLAVE_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) master time to get into receive mode. The
# master may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_SLAVE_SEND_DELAY = const(10)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 5, "ce": 8}
#cfg = {"spi": 0, "miso": 4, "mosi": 7, "sck": 6, "csn": 5, "ce": 8}

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

analog_value = machine.ADC(28)

def NRF_Master():
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=16)
    auto_ack(nrf)
    nrf.set_power_speed(nrf24l01.POWER_3, nrf24l01.SPEED_2M)
    nrf.set_crc(2)
    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.start_listening()
    
    led_state = 0
    print("NRF24L01 master mode, sending packets...")
    while(1):
        nrf.stop_listening()
        millis = utime.ticks_ms()
        #reading = analog_value.read_u16()
        ax=round(imu.accel.x,2)
        ay=round(imu.accel.y,2)
        az=round(imu.accel.z,2)
        gx=round(imu.gyro.x)
        gy=round(imu.gyro.y)
        gz=round(imu.gyro.z)
        angle = gz*0.1666667
        tem=round(imu.temperature,2)
        y= round(math.degrees(math.atan2(ax, (az+abs(ay)))+math.pi),2)
        x = round(math.degrees(math.atan2(ay, az + abs(ax))+math.pi),2)
        anglex = round(math.degrees(math.atan2(ay,math.sqrt(math.pow(ax,2)+math.pow(az,2)))))
        angley = round(math.degrees(math.atan2(-1*ax,math.sqrt(math.pow(ay,2)+math.pow(az,2)))))
        #reading = 10
        #led_state = max(1, (led_state << 1) & 0x0F)
        print(" inclination : ",anglex,angley, y, x)
        try:
            nrf.send(struct.pack("fff", angle, tem, x))
            pixels_fill(RED)
            pixels_show()
            buz.value(1)
            #nrf.send(struct.pack("ii", millis,int(data['gyro']['x']) ))
        except OSError:
            print("OSError exception")
         
         # start listening again
        nrf.start_listening()

        # wait for response, with 250ms timeout
        start_time = utime.ticks_ms()
        timeout = False
        while not nrf.any() and not timeout:
            if utime.ticks_diff(utime.ticks_ms(), start_time) > 10:
                timeout = True

        if timeout:
            pass
            #print("failed, response timed out")

        else:
            # recv packet
            (got_millis,) = struct.unpack("i", nrf.recv())

            # print response and round-trip delay
            print(
                "got response:",
                got_millis,
                "(delay",
                utime.ticks_diff(utime.ticks_ms(), got_millis),
                "ms)",
            )
            
        # delay then loop
        pixels_fill(BLACK)
        pixels_show()
        buz.value(0)
        utime.sleep_ms(10)

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
 
NRF_Master()
#nrf24l01test.master()

