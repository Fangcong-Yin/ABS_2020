import time
import board
import busio
import adafruit_adxl34x

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_adxl34x.ADXL345(i2c)

sensor.range = adafruit_adxl34x.Range.RANGE_16_G

while True:
    print('%f %f %f'%sensor.acceleration)

    time.sleep(.02)
