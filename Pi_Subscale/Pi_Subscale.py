#Import libraries here






#Initialize Sensors




#Initialize File
filename = 'data.txt'

with open(filename,'w') as f:
    dataFile.print("Flight State,")
    dataFile.print("Time ms,")
    dataFile.print("BNO Temperature C,")
    dataFile.print("MPL Temperature C,")
    dataFile.print("BNO X Acceleration m/s^2,")
    dataFile.print("BNO Y Acceleration m/s^2,")
    dataFile.print("BNO Z Acceleration m/s^2,")
    dataFile.print("BNO X Linear Acceleration m/s^2,")
    dataFile.print("BNO Y Linear Acceleration m/s^2,")
    dataFile.print("BNO Z Linear Acceleration m/s^2,")
    dataFile.print("BNO Gyro X rad/s,"); dataFile.flush();
    dataFile.print("BNO Gyro Y rad/s,"); dataFile.flush();
    dataFile.print("BNO Gyro Z rad/s,"); dataFile.flush();
    dataFile.print("BNO Euler Angle X,"); dataFile.flush();
    dataFile.print("BNO Euler Angle Y,"); dataFile.flush();
    dataFile.print("BNO Euler Angle Z,"); dataFile.flush();
    dataFile.print("BNO Magnetometer X,"); dataFile.flush();
    dataFile.print("BNO Magnetometer Y,"); dataFile.flush();
    dataFile.print("BNO Magnetometer Z,"); dataFile.flush();
    dataFile.print("BNO Quaternion W,"); dataFile.flush();
    dataFile.print("BNO Quaternion X,"); dataFile.flush();
    dataFile.print("BNO Quaternion Y,"); dataFile.flush();
    dataFile.print("BNO Quaternion Z,"); dataFile.flush();
    dataFile.print("Altitude m,"); dataFile.flush();
    dataFile.print("Pressure Pa,"); dataFile.flush();
    dataFile.print("ADXL X Acceleration,"); dataFile.flush();
    dataFile.print("ADXL Y Acceleration,"); dataFile.flush();
    dataFile.print("ADXL Z Acceleration,"); dataFile.flush();
    dataFile.print("\n");
    






#Main Loop
while True:
    pass