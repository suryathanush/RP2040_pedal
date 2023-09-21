"""Test for nrf24l01 module.  Portable between MicroPython targets."""

import usys
import ustruct as struct
import utime
from machine import Pin, SPI
from nrf24l01 import *
from micropython import const

# Slave pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Slave pauses an additional _SLAVE_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) master time to get into receive mode. The
# master may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_SLAVE_SEND_DELAY = const(10)

# if usys.platform == "pyboard":
#     cfg = {"spi": 2, "miso": "Y7", "mosi": "Y8", "sck": "Y6", "csn": "Y5", "ce": "Y4"}
# elif usys.platform == "esp8266":  # Hardware SPI
#     cfg = {"spi": 1, "miso": 12, "mosi": 13, "sck": 14, "csn": 4, "ce": 5}
# elif usys.platform == "esp32":  # Software SPI
#     cfg = {"spi": -1, "miso": 32, "mosi": 33, "sck": 25, "csn": 26, "ce": 27}
# elif usys.platform == "rp2":  # PI PICO
#     cfg = {"spi": 0, "miso": 4, "mosi": 7, "sck": 6, "csn": 5, "ce": 8}
# else:
#     raise ValueError("Unsupported platform {}".format(usys.platform))

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 5, "ce": 8}

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xe2\xf0\xf0\xf0\xf0")

Rx_ID = pipes[1]
Tx_ID = b'\x00'

def Generate_RxID():
    global Rx_ID
    print("generating Rx_ID")
    UID = machine.unique_id()
    # Extract the last 5 elements from the hex array
    last_5_elements = UID[-5:]
    # Convert the last 5 elements to bytes
    Rx_ID = bytes(last_5_elements)
    print("Tx pipe ID:", Rx_ID)

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
 
def send_Rx_ID(nrf):
    # stop listening and send packet
    nrf.stop_listening()
    millis = utime.ticks_ms()
    try:
        print("sending")
        nrf.send(Rx_ID)
    except OSError:
        pass

def send_conf(nrf):
    try:
        print("sending confirmation")
        nrf.send(b"\xf0\xf0\xf0\xf0\xf0")
    except OSError:
        pass
    
def Pair():
    global Tx_ID
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, payload_size=5)
    nrf.set_power_speed(POWER_3, SPEED_1M)
    auto_ack(nrf)
    nrf.set_crc(2)
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, pipes[0])
    nrf.start_listening()
    Generate_RxID()
    print("NRF24L01 pairing mode, sending ", )

    while(1):
        if(Tx_ID == b'\x00'):
            # stop listening and send packet
            nrf.stop_listening()
            millis = utime.ticks_ms()
            send_Rx_ID(nrf)

            # start listening again
            nrf.start_listening()

        # wait for response, with 250ms timeout
        start_time = utime.ticks_ms()
        timeout = False
        while not nrf.any() and not timeout:
            if utime.ticks_diff(utime.ticks_ms(), start_time) > 250:
                timeout = True

        if timeout:
            print("failed, response timed out")

        else:
            recv = nrf.recv()
            Tx_ID = recv
            print("got response:",Tx_ID)
            utime.sleep_ms(_SLAVE_SEND_DELAY)
            nrf.stop_listening()
            millis = utime.ticks_ms()
            for _ in range(10):
                send_conf(nrf)
            break            


        # delay then loop
        #utime.sleep_ms(250)

    print("Pairing done")
    slave()

def slave():
    print("entered recieving mode")
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    nrf = NRF24L01(spi, csn, ce, payload_size=16)
    nrf.set_power_speed(POWER_3, SPEED_1M)
    auto_ack(nrf)
    nrf.set_crc(2)
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, Tx_ID)
    nrf.start_listening()

    print("NRF24L01 slave mode, waiting for packets... (ctrl-C to stop)")

    while True:
        if nrf.any():
            while nrf.any():
                buf = nrf.recv()
                millis, PedalAngle, RPM, bat_v = struct.unpack("ffff", buf)
                print("received:",millis,
                      "PedalAngle: ",PedalAngle,
                      "RPM: ",RPM,
                      "bat_v:",bat_v)
                #utime.sleep_ms(_RX_POLL_DELAY)


print("NRF24L01 reciever loaded")
Pair()
