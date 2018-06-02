from Adafruit_BBIO.SPI import SPI
from adafruit_max31865 import MAX31865
import time


spi = SPI(2, 1)
spi.msh = 500000
spi.cshigh = False
spi.mode = 3

sensor = MAX31865(spi)

while True:
    temperature = sensor.get_temperature()
    print('Temperature: {0:0.3f}C'.format(temperature))
    time.sleep(1)
