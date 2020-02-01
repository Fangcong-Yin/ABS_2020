"""
Usage: automatically initializes sensors upon startup
Methods:
zero_mpl-should call periodically before launch
read_data-queries all the sensor data
write_data-writes current data to file
get_data-returns the data in the following format:
(timestamp, euler, altitude, acceleration)

"""


#Import libraries here
import time,sys
import board,busio
import glob

import adafruit_mpl3115a2
import adafruit_bno055
import adafruit_adxl34x
from gpiozero import LED



class DataLogger():
    def __init__(self):
        #Constants
        self.delay_time = .001

        #I2C Initialization
        self.i2c = busio.I2C(board.SCL, board.SDA)

        #BNO Initialization
        self.bno = adafruit_bno055.BNO055(i2c)
        #self.bno.use_external_crystal = True

        #MPL Initialization
        self.mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
        self.mpl._ctrl_reg1 = adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_OS1 |adafruit_mpl3115a2._MPL3115A2_CTRL_REG1_ALT
        time.sleep(1)
        self.zero_mpl()

        self.led = LED(21)

        #Initialize ADXL
        self.adxl = adafruit_adxl34x.ADXL345(i2c)
        self.adxl.range = adafruit_adxl34x.Range.RANGE_16_G
        self.adxl.data_rate = adafruit_adxl34x.DataRate.RATE_100_HZ

        #Initialize file and write header
        self.filename = self.gen_filename()
        with open(filename,'w') as f:
            f.write("Time ms,")
            #f.write("BNO X Acceleration m/s^2,")
            #f.write("BNO Y Acceleration m/s^2,")
            #f.write("BNO Z Acceleration m/s^2,")
            #f.write("BNO Gyro X rad/s,")    
            #f.write("BNO Gyro Y rad/s,")    
            #f.write("BNO Gyro Z rad/s,")    
            f.write("BNO Euler Angle X,")    
            f.write("BNO Euler Angle Y,")    
            f.write("BNO Euler Angle Z,")    
            #f.write("BNO Magnetometer X,")    
            #f.write("BNO Magnetometer Y,")    
            #f.write("BNO Magnetometer Z,")    
            #f.write("BNO Gravity X,")    
            #f.write("BNO Gravity Y,")    
            #f.write("BNO Gravity Z,")    
            f.write("Altitude m,")    
            f.write("ADXL X Acceleration,")    
            f.write("ADXL Y Acceleration,")    
            f.write("ADXL Z Acceleration,")    
            f.write("\n") 

    #Take a running sample and create a new reference for the MPL sensor
    def zero_mpl(self):
        total = 0
        for i in range(200):
            #print(i)
            total += self.mpl.pressure
            time.sleep(.005)
        mpl.sealevel_pressure = int(total / 200)

    #Figure out what to name the file
    def gen_filename(self):
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

        return filename         

    #Read sensor data
    def read_data(self):
        self.led.on()

        self.timestamp = time.time()

        #Read MPL data
        self.mpl_altitude = self.mpl.altitude
        self.mpl_altitude = self.mpl_altitude if self.mpl_altitude is not None else 0

        #Read BNO data
        #bno_accel = bno.acceleration
        #bno_accel = tuple([i if i is not None else 0 for i in bno_accel])
                
        #bno_mag = bno.magnetic
        #print(bno_mag)
        #bno_mag = tuple([i if i is not None else 0 for i in bno_mag])
        #bno_gyro = bno.gyro
        #bno_gyro = tuple([i if i is not None else 0 for i in bno_gyro])
        self.bno_euler = self.bno.euler
        #print(bno_euler)
        self.bno_euler = tuple([i if i is not None else 0 for i in self.bno_euler])
        #bno_quaternion = bno.quaternion
        #bno_linac = bno.linear_acceleration
        #bno_gravy = bno.gravity
        #bno_gravy = tuple([i if i is not None else 0 for i in bno_gravy])

        #Read ADXL data
        self.adxl_accel = self.adxl.acceleration
        self.adxl_accel = tuple([i if i is not None else 0 for i in self.adxl_accel])

        self.led.off()

    #Write current sensor data to a file
    def write_data(self):
        with open(self.filename,'a') as f:
            f.write(str(self.timestamp)+', ')
            #f.write('%f, %f, %f,'%bno_accel)
            #f.write('%f, %f, %f,'%bno_gyro)
            f.write('%f, %f, %f,'%self.bno_euler)
            #f.write('%f, %f, %f,'%bno_mag)
            #f.write('%f, %f, %f,'%bno_gravy)
            f.write(str(self.mpl_altitude))
            f.write(', %f, %f, %f'%self.adxl_accel)
            f.write('\n')

    #Outputs the current data
    def get_data(self):
        return self.timestamp,self.bno_euler,self.mpl_altitude,self.adxl_accel
