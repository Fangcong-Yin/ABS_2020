#Import libraries here
import time,sys
import board,busio

import adafruit_mpl3115a2
from Adafruit_BNO055 import BNO055




#BNO Initialization
#TODO: figure out how to distinguish between the BNOs
#TODO: figure out how to set one BNO into 16g mode-may need to tweak library
bno1 = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
bno2 = BNO055.BNO055(serial_port='/dev/serial0', rst=18)

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

#Initialize both, stop if something goes wrong
if not (bno1.begin() and bno2.begin()):
    raise RuntimeError("Failed to initialize BNO")


#Initialize MPL
i2c = busio.I2C(board.SCL, board.SDA)
mpl = adafruit_mpl3115a2.MPL3115A2(i2c)
mpl.sealevel_pressure = 102250 #value in pascals


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

    #Read BNO 1-Orientation Data
    bno_euler_x,bno_euler_y,bno_euler_z = bno1.read_euler()
    bno_quat_x,bno_quat_y,bno_quat_z,bno_quat_w = bno1.read_quaternion()
    bno1_temp = bno1.read_temp()
    bno1_mag_x,bno1_mag_y,bno1_mag_z = bno1.read_magnetometer() #microteslas
    bno1_gyro_x,bno1_gyro_y,bno1_gyro_z = bno1.read_gyroscope() #deg/s
    bno1_accel_x,bno1_accel_y,bno1_accel_z = bno1.read_accelerometer()
    bno1_linac_x,bno1_linac_y,bno1_linac_z = bno1.read_linear_acceleration()
    
    #Read BNO 2-Acceleration Data
    bno2_mag_x,bno2_mag_y,bno2_mag_z = bno2.read_magnetometer() #microteslas
    bno2_gyro_x,bno2_gyro_y,bno2_gyro_z = bno2.read_gyroscope() #deg/s
    bno2_accel_x,bno2_accel_y,bno2_accel_z = bno2.read_accelerometer()

    #TODO: Add Data Analysis
    #TODO: Add printing
