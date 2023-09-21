from machine import Pin, SPI
import struct

from nrf24l01 import NRF24L01

csn = Pin(5, mode=Pin.OUT, value=1)  # chip select not
ce  = Pin(8, mode=Pin.OUT, value=0)  # chip enable

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2 - swap these on the other Pico!
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

def setup():
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=4)
    
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, pipes[0])
    nrf.start_listening()
    return nrf

def demo(nrf):
    count = 0
    while True:
        count = count+1
        if nrf.any():
            buf = nrf.recv()
            got = struct.unpack("i", buf)[0]
            print("rx", got)
            
        nrf.stop_listening()
        try:
            nrf.send(struct.pack("i", 10))
            print("sending", count)
        except OSError:
            print('message lost')
        nrf.start_listening()
            

def auto_ack(nrf):
    nrf.reg_write(0x01, 0b11111000)  # enable auto-ack on all pipes
    

nrf = setup()
auto_ack(nrf)
demo(nrf)