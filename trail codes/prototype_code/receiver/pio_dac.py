  
# Example of using PIO writing a parallel byte from data
# for a more wrapped-up examples, see https://github.com/raspberrypi/pico-micropython-examples/blob/master/pio/pio_pwm.py

from machine import Pin
from rp2 import PIO, StateMachine, asm_pio
from time import sleep

@asm_pio(out_init=(rp2.PIO.OUT_HIGH,) * 8, out_shiftdir=PIO.SHIFT_RIGHT, 
 autopull=True, pull_thresh=16)
def paral_prog():
    pull()  
    out(pins, 8)  

paral_sm = StateMachine(0, paral_prog, freq=10000000, out_base=Pin(16))
paral_sm.active(1)

while True:
    for i in range(0, pow(2,8)):
        paral_sm.put(i)
        print(i)
        sleep(0.5)
