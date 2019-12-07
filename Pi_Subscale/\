#Import libraries here
import time,sys
import board,busio

import adafruit_mpl3115a2
import adafruit_bno055
import adafruit_adxl34x


#Constants
delay_time = .001

#I2C Initialization
i2c = busio.I2C(board.SCL, board.SDA)

#BNO Initialization
bno = adafruit_bno055.BNO055(i2c)
bno.use_external_crystal = True

#Initialize MPL
mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
mpl._ctrl_reg1 = adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_OS1 |adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_ALT
time.sleep(1)

total = 0
for i in range(200):
    print(i)
    total += mpl.pressure
    time.sleep(.005))
mpl.sealevel_pressure = int(total / 200)

#Initialize ADXL
adxl = adafruit_adxl34x.ADXL345(i2c)
adxl.range = adafruit_adxl34x.Range.RANGE_16_G
adxl.data_rate = adafruit_adxl34x.DataRate.RATE_100_HZ

#Initialize File and Write Header
filename = 'data.txt'
with open(filename,'w') as f:
    f.write("Flight State,")
    f.write("Time ms,")
    f.write("BNO Temperature C,")
    f.write("MPL Temperature C,")
    f.write("BNO X Acceleration m/s^2,")
    f.write("BNO Y Acceleration m/s^2,")
    f.write("BNO Z Acceleration m/s^2,")
    f.write("BNO X Linear Acceleration m/s^2,")
    f.write("BNO Y Linear Acceleration m/s^2,")
    f.write("BNO Z Linear Acceleration m/s^2,")
    f.write("BNO Gyro X rad/s,")    
    f.write("BNO Gyro Y rad/s,")    
    f.write("BNO Gyro Z rad/s,")    
    f.write("BNO Euler Angle X,")    
    f.write("BNO Euler Angle Y,")    
    f.write("BNO Euler Angle Z,")    
    f.write("BNO Magnetometer X,")    
    f.write("BNO Magnetometer Y,")    
    f.write("BNO Magnetometer Z,")    
    f.write("BNO Quaternion W,")    
    f.write("BNO Quaternion X,")    
    f.write("BNO Quaternion Y,")    
    f.write("BNO Quaternion Z,")    
    f.write("Altitude m,")    
    f.write("Pressure Pa,")    
    f.write("ADXL X Acceleration,")    
    f.write("ADXL Y Acceleration,")    
    f.write("ADXL Z Acceleration,")    
    f.write("\n") 



#Main Loop
while True:
    #Read MPL data
    mpl_pressure = mpl.pressure
    mpl_altitude = mpl.altitude
    mpl_temp = mpl.temperature

    #Read BNO data
    bno_accel = bno.acceleration



    #Read ADXL data
    adxl_accel = adxl.acceleration


    print(f'MPL Altitude: {mpl_altitude}')
    time.sleep(.005)
