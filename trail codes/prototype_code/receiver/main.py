import usys
import ustruct as struct
import utime
from machine import Pin, SPI
import nrf24l01
from micropython import const
from rp2 import PIO, StateMachine, asm_pio
from time import sleep

# Slave pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Slave pauses an additional _SLAVE_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) master time to get into receive mode. The
# master may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_SLAVE_SEND_DELAY = const(10)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 5, "ce": 8}
#cfg = {"spi": 0, "miso": 4, "mosi": 7, "sck": 6, "csn": 5, "ce": 8}

pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")


@asm_pio(out_init=(rp2.PIO.OUT_HIGH,) * 6, out_shiftdir=PIO.SHIFT_RIGHT, 
 autopull=True, pull_thresh=16)
def paral_prog():
    pull()  
    out(pins, 6)  

DAC = StateMachine(0, paral_prog, freq=10000000, out_base=Pin(16))
DAC.active(1)

def BitConvert(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
 
def NRF_Slave():
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    nrf = nrf24l01.NRF24L01(spi, csn, ce, payload_size=16)
    
    auto_ack(nrf)
    nrf.set_power_speed(nrf24l01.POWER_3, nrf24l01.SPEED_2M)
    nrf.set_crc(2)
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, pipes[0])
    nrf.start_listening()
    print("NRF24L01 slave mode, waiting for packets... (ctrl-C to stop)")
    
    while True:
        if nrf.any():
            while nrf.any():
                buf = nrf.recv()
                rpm, temp, Angle= struct.unpack("fff", buf)
                print("received:", "RPM : ", rpm, "Temperature : ", temp, "Angle : ", Angle)
                #dac_val = BitConvert(dac_val, 0, 65535, 0, 64);
                #DAC.put(int(dac_val))
                utime.sleep_ms(_RX_POLL_DELAY)

            # Give master time to get into receive mode.
            utime.sleep_ms(_SLAVE_SEND_DELAY)
            nrf.stop_listening()
            try:
                millis = utime.ticks_ms()
                nrf.send(struct.pack("ff", millis))
            except OSError:
                pass
            print("sent response")
            nrf.start_listening()
    
NRF_Slave()


