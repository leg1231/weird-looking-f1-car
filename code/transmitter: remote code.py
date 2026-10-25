from machine import Pin, SPI, ADC
import time
from lora import LoRa
import utime

"""lora pins"""
spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs   = Pin(17, Pin.OUT)
rst  = Pin(20, Pin.OUT)
dio0 = Pin(21, Pin.IN)
lora = LoRa(spi, cs, rst, dio0, freq=915e6, pa_boost=True, tx_power=17)
print("LoRa ready")

"""joystick pins"""
s0 = Pin(0, Pin.OUT)
s1 = Pin(1, Pin.OUT)
s2 = Pin(2, Pin.OUT)
adc = ADC(Pin(26))


def select_channel(ch):
    s0.value(ch & 1)
    s1.value((ch >> 1) & 1)
    s2.value((ch >> 2) & 1)
    
def read_channel(ch):
    select_channel(ch)
    time.sleep_us(5000)
    rawValue=adc.read_u16()
    return rawValue

while True:
    x=read_channel(0)
    y=read_channel(1)
    print (f"X: {x}  Y: {y}")
    
    msg="{},{}".format(x, y)
    lora.send(msg)
    
    time.sleep_ms(50)
    

