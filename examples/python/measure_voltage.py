# Чтение напряжения с canhat
# Доустановите несколько библиотек, для этого в терминале введите:

# pip3 install adafruit-circuitpython-ads1x15 --break-system-packages
# sudo apt update
# sudo apt install python3-lgpio -y 

import time
import board
import busio

import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# I2C
i2c = busio.I2C(board.SCL, board.SDA)

# ADS1115
ads = ADS.ADS1115(i2c, address=0x48)

# возьмем выход AIN0
chan = AnalogIn(ads, 0)
while True:
    print(f"Voltage: {chan.voltage:.3f} V")
    time.sleep(0.5)
