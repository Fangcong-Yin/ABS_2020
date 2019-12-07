#Import libraries here
import time,sys
import board,busio
import glob

import adafruit_mpl3115a2
import adafruit_bno055
import adafruit_adxl34x
from gpiozero import LED

#Constants
delay_time = .001

#I2C Initialization
i2c = busio.I2C(board.SCL, board.SDA)

#BNO Initialization
bno = adafruit_bno055.BNO055(i2c)
#bno.use_external_crystal = True

#Initialize MPL
mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
mpl._ctrl_reg1 = adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_OS1 |adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_ALT
time.sleep(1)

led = LED(21)

total = 0
for i in range(200):
    #print(i)
    total += mpl.pressure
    time.sleep(.005)
mpl.sealevel_pressure = int(total / 200)

#Initialize ADXL
adxl = adafruit_adxl34x.ADXL345(i2c)
adxl.range = adafruit_adxl34x.Range.RANGE_16_G
adxl.data_rate = adafruit_adxl34x.DataRate.RATE_100_HZ

#Figure out what to name the file
files = glob.glob('./data*')
x = 0
found = False
while not found:
    found = True
    for file in files:
        num = int(''.join([i for i in file if i in '1234567890']))
        if num == x:
            found = False

    if not found:
        x += 1
filename = 'data'+str(x) + '.csv'            

#Initialize File and Write Header
with open(filename,'w') as f:
    f.write("Time ms,")
    f.write("BNO X Acceleration m/s^2,")
    f.write("BNO Y Acceleration m/s^2,")
    f.write("BNO Z Acceleration m/s^2,")
    f.write("BNO Gyro X rad/s,")    
    f.write("BNO Gyro Y rad/s,")    
    f.write("BNO Gyro Z rad/s,")    
    f.write("BNO Euler Angle X,")    
    f.write("BNO Euler Angle Y,")    
    f.write("BNO Euler Angle Z,")    
    f.write("BNO Magnetometer X,")    
    f.write("BNO Magnetometer Y,")    
    f.write("BNO Magnetometer Z,")    
    f.write("BNO Gravity X,")    
    f.write("BNO Gravity Y,")    
    f.write("BNO Gravity Z,")    
    f.write("Altitude m,")    
    f.write("ADXL X Acceleration,")    
    f.write("ADXL Y Acceleration,")    
    f.write("ADXL Z Acceleration,")    
    f.write("\n") 



#Main Loop
while True:
    led.on()
    #Read MPL data
    #mpl_pressure = mpl.pressure
    mpl_altitude = mpl.altitude
    mpl_altitude = mpl_altitude if mpl_altitude is not None else 0
    #mpl_temp = mpl.temperature

    #Read BNO data
    #bno_temp = bno.temperature
    bno_accel = bno.acceleration
    #print(bno_accel)
    bno_accel = tuple([i if i is not None else 0 for i in bno_accel])
            
    bno_mag = bno.magnetic
    #print(bno_mag)
    bno_mag = tuple([i if i is not None else 0 for i in bno_mag])
    bno_gyro = bno.gyro
    bno_gyro = tuple([i if i is not None else 0 for i in bno_gyro])
    bno_euler = bno.euler
    #print(bno_euler)
    bno_euler = tuple([i if i is not None else 0 for i in bno_euler])
    #bno_quaternion = bno.quaternion
    #bno_linac = bno.linear_acceleration
    #bno_gravy = bno.gravity
    #bno_gravy = tuple([i if i is not None else 0 for i in bno_gravy])

    #Read ADXL data
    adxl_accel = adxl.acceleration
    adxl_accel = tuple([i if i is not None else 0 for i in adxl_accel])

    led.off()

    with open(filename,'a') as f:
        f.write(str(time.time())+', ')
        f.write('%f, %f, %f,'%bno_accel)
        f.write('%f, %f, %f,'%bno_gyro)
        f.write('%f, %f, %f,'%bno_euler)
        f.write('%f, %f, %f,'%bno_mag)
        #f.write('%f, %f, %f,'%bno_gravy)
        f.write(str(mpl_altitude))
        f.write(', %f, %f, %f'%adxl_accel)
        f.write('\n')

        


    #print(f'MPL Altitude: {mpl_altitude}')
