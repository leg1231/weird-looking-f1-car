from machine import Pin, SPI, PWM
import time
from lora import LoRa
import utime

# lora comamands
spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(16))
cs   = Pin(21, Pin.OUT)
rst  = Pin(22, Pin.OUT)
dio0 = Pin(17, Pin.IN)
lora = LoRa(spi, cs, rst, dio0, freq=915e6, pa_boost=True, tx_power=17)
print("LoRa ready - listening...")

# motor direction, power, and brake
dirL_pin   = Pin(28, Pin.OUT)
dirR_pin   = Pin(0, Pin.OUT)

driveL_pwm = PWM(Pin(27))
driveL_pwm.freq(20000)
driveL_pwm.duty_u16(0)
driveR_pwm = PWM(Pin(1))
driveR_pwm.freq(20000)
driveR_pwm.duty_u16(0)

# Initialize PWM pins for servos (moved off GP16/17 to avoid SPI conflict)
servoL = PWM(Pin(20))
servoR = PWM(Pin(15))
servoL.freq(50)  # Pulse every 20ms
servoR.freq(50)

receving_led = Pin(3, Pin.OUT)
receving_led.off()

def set_angleL(angleL, min_pulse_us=500, max_pulse_us=2500):
    us_per_degree = (max_pulse_us - min_pulse_us) / 270
    pulse_us = us_per_degree * angleL + min_pulse_us
    duty = int(pulse_us / 0.305)
    servoL.duty_u16(duty)

def set_angleR(angleR, min_pulse_us=500, max_pulse_us=2500):
    us_per_degree = (max_pulse_us - min_pulse_us) / 270
    pulse_us = us_per_degree * angleR + min_pulse_us
    duty = int(pulse_us / 0.305)
    servoR.duty_u16(duty)

# Define resting (center) angles
center_angleL = 122.1
center_angleR = 142.4

# Define min and max angle limits based on your original button logic
min_angleL, max_angleL = 94.6, 147.4
min_angleR, max_angleR = 109.6, 167.5

# Initialize a variable to track when we last saw a signal
last_received_time = time.ticks_ms()
signal_timeout = 300  # Milliseconds to wait before turning off the LED

while True:
    payload, rssi = lora.receive()
    
    if payload is not None:
        # We got a packet! Reset our timer and turn the LED on
        last_received_time = time.ticks_ms()
        receving_led.on()
        
        try:
            if isinstance(payload, bytes):
                payload = payload.decode('utf-8')

            parts = payload.split(",")
            y = int(parts[0])
            x = int(parts[1])
            print("x: {}  y: {}  RSSI: {}dBm".format(x, y, rssi))
        except Exception as e:
            print("Parse error:", e, "raw:", payload)
            continue

        # Apply command - X axis servo control
        if 32700 <= x <= 33300:
            angleL = center_angleL
            angleR = center_angleR
        elif x < 32700:
            scale = x / 32699
            angleL = min_angleL + scale * (center_angleL - min_angleL)
            angleR = min_angleR + scale * (center_angleR - min_angleR)
        else:
            scale = (x - 33301) / (65535 - 33301)
            angleL = center_angleL + scale * (max_angleL - center_angleL)
            angleR = center_angleR + scale * (max_angleR - center_angleR)
            
        set_angleL(angleL)
        set_angleR(angleR)
            
        # Apply command - Y axis motor drive control
        if y > 36000:
            dirR_pin.off()
            dirL_pin.on()
            driveR_pwm.duty_u16(int((y - 36000) / (65535 - 36000) * 65535))
            driveL_pwm.duty_u16(int((y - 36000) / (65535 - 36000) * 65535))
        elif y < 29000:
            dirR_pin.on()
            dirL_pin.off()
            driveR_pwm.duty_u16(int((29000 - y) / 29000 * 65535))
            driveL_pwm.duty_u16(int((29000 - y) / 29000 * 65535))
        else:
            driveR_pwm.duty_u16(0)
            driveL_pwm.duty_u16(0)
            
    else:
        # No packet this loop cycle. Check if we've exceeded our timeout window.
        if time.ticks_diff(time.ticks_ms(), last_received_time) > signal_timeout:
            receving_led.off()
            dirR_pin.on()
            dirL_pin.off()
            driveR_pwm.duty_u16(100)
            driveL_pwm.duty_u16(100)
            set_angleL(122.1)
            set_angleR(142.4)
        
    time.sleep_ms(10)
