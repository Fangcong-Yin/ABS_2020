/* BEFORE FLIGHT CHECKLIST
 *    make sure R/W on SD card
 *    comment out:
 *    - while(!//Serial)
 *    - all //Serial.print statements
 *    - all //Serial.println statements
 *    - all Matrix.print statements
 *    update:
 *    - pAir
 *    - aRocket
 *    - mRocket
 *    - DELAY_TIME (if we find a lower value that the sensors can still handle)
 *    
 * AFTER FLIGHT CHECKLIST
 * - inspect SD card for data
 * - copy data file to computer
 * - update data file name on SD card
 * - fire handguns wildly into the sky in celebratory gesture
 * 
 */

// libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "SparkFunMPL3115A2.h"
//#include <Adafruit_MPL3115A2.h>
//#include <Adafruit_L3GD20_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//#include <SD.h>
#include <SdFat.h>
SdFat SD;
#include <SPI.h>
#include <MatrixMath.h>
#include <Adafruit_ADXL345_U.h>

#define DELAY_TIME 10
#define ARMED 0
#define LAUNCHED 1
#define BURNOUT 2
#define APOGEE 3
#define LANDED 4

float lastT, dT;
float k;
const float Cd = 0.25; // UPDATE THESE VALUES!!
const float pAir = 1.225;
const float aRocket = 0.1;
const float mRocket = 10;
// ^^^ UPDATE THESE VALUES!!


// matrices for kalman filter, continually updated
mtx_type x[3][1] = {{0}, {0}, {0}}; // State (position, velocity, accel), varies with time
mtx_type P[3][3] = {{0.005, 0, 0}, {0, 0.0122, 0}, {0, 0, 0.0176}}; // Covariance, varies with time
mtx_type R[3][3] = {{0.1, 0, 0}, {0, 0.1, 0}, {0, 0, 0.4}};   // Distrust of sensors
mtx_type Theta[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}; // physics transformation matrix
mtx_type I3[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Identity (for maths)
mtx_type Q[3][3] = {{0, 0, 0}, {0, 0.001, 0}, {0, 0, 0.001}}; // Distrust of predictions


const int chipSelect = SDCARD_SS_PIN;

int flightstate = ARMED;
bool LEDINITIAL = false;
bool LEDWRITING = false;

bool BNOINIT = false;
bool L3GINIT = false;
bool SDINIT = false;
bool ADXLINIT = false;

const float accelLiftoffThreshold = 50; //m/s^2  50
const float baroLiftoffThreshold = 10; //m   10
const float accelBurnoutThreshold = -5; //m/s^2 -5
//const float baroApogeeThreshold = 5; //m    5
//const float baroLandedThreshold = 0; //m    5
//const float accelFreefallThreshold = 1; //m/s^2  30

float launchA;
float maxA = -100;
float altitude_offset;

File dataFile;

// BNO instantiation
Adafruit_BNO055 bno = Adafruit_BNO055();

// MPL instantiation
//Adafruit_MPL3115A2 MPLPressure;
MPL3115A2 MPLPressure;

// ADXL instantiation
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


//L3G instantiation?
/* Assign a unique ID to this sensor at the same time */
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

char filename[9] = "data.txt";

void setup() {

  //Serial.begin(9600);   // printing to screen // TESTING

  //while(!Serial) ;  // TESTING
  
  Wire.begin();        // Join i2c bus

  if (!SD.begin(chipSelect)) {
    ////Serial.println("The SD card has not exploded!!!");
    return;
  } else{
    SDINIT = true;
  }

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    ////Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // Record failure in SD card
    while(1);
  } else
  {
    //bno.set16GRange(); //do to disable on-chip filtering
    
    BNOINIT = true;
  }
   /* Enable auto-ranging */
  //gyro.enableAutoRange(true);
  //if(!gyro.begin())
  //{
  //  /* There was a problem detecting the L3GD20 ... check your connections */
  //  //Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
  //  while(1);
  //}
  //else
  //{
  //  L3GINIT = true;
  //}
  if (!accel.begin()) {
    Serial.println("ADXL not detected, check your wiring.");
  }

  ADXLINIT = true;
  accel.setRange(ADXL345_RANGE_16_G); //Set to 16_G range
  



  // check if that init returned something // MPLPressure.begin();
  MPLPressure.begin();

  //Zero out altitude
  float sum;
  for (int i=0; i<200; i++) {
    sum += MPLPressure.readAltitude();
    delay(40);
  }
  altitude_offset = sum / 200;
  
  //float pressure = MPLPressure.getPressure();
  ////Serial.print("Initial Pressure: "); //Serial.println(pressure);
  //MPLPressure.setSeaPressure(pressure);
  //delay(10);
  ////Serial.print("Pressure After Zeroing: "); //Serial.println(MPLPressure.getPressure());

  if ( BNOINIT && SDINIT && ADXLINIT )
  {
    LEDINITIAL = true;
  }
  
  bno.setExtCrystalUse(true);

  MPLPressure.setModeAltimeter(); // Measure altitude above sea level in meters (MPL)
  MPLPressure.setOversampleRate(1); // Set Oversample to the recommended 128
  MPLPressure.enableEventFlags(); // Enable all three pressure and temp event flags 
  
  k = -1*Cd*pAir*aRocket / (2*mRocket);
  
  x[0][0] = MPLPressure.readAltitude(); //Set initial altitude based on sensor reading
  //x[0][0] = MPLPressure.getAltitude();
  lastT = millis();

  launchA = x[0][0];

  //delay(DELAY_TIME); // Delay time for sensor sampling rate

  Print_Header();

  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(6, LEDINITIAL);
}

void loop() {
  //Serial.println(millis());
  ////Serial.println("We are looping");
  digitalWrite(6,true);
  // Temperature variables
  int8_t bno_temp = bno.getTemp();
  float mpl_temp = MPLPressure.readTemp();
  //float mpl_temp = MPLPressure.getTemperature();

  // vector creation
  // acceleration units in m/s^2
  // gyroscope units in rad/s
  imu::Vector<3> bno_accelerometer = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> bno_linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> bno_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> bno_euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> bno_mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu::Quaternion bno_quaternion = bno.getQuat();

  /* Get a new sensor event */ 
  //sensors_event_t L3G_event; 
  //gyro.getEvent(&L3G_event);

  sensors_event_t adxl_event;
  accel.getEvent(&adxl_event);

  //Get ADXL
  double adxl_accel_x = adxl_event.acceleration.x;
  double adxl_accel_y = adxl_event.acceleration.y;
  double adxl_accel_z = adxl_event.acceleration.z;

  //Get BNO
  double bno_accel_x = bno_accelerometer.x();
  double bno_accel_y = bno_accelerometer.y();
  double bno_accel_z = bno_accelerometer.z();
  double bno_lin_accel_x = bno_linearAccel.x();
  double bno_lin_accel_y = bno_linearAccel.y();
  double bno_lin_accel_z = bno_linearAccel.z();
  double bno_gyro_x = bno_gyro.x();
  double bno_gyro_y = bno_gyro.y();
  double bno_gyro_z = bno_gyro.z();
  double bno_euler_x = bno_euler.x();
  double bno_euler_y = bno_euler.y();
  double bno_euler_z = bno_euler.z();
  double bno_mag_x = bno_mag.x();
  double bno_mag_y = bno_mag.y();
  double bno_mag_z = bno_mag.z();
  double bno_quaternion_w = bno_quaternion.w();
  double bno_quaternion_x = bno_quaternion.w();
  double bno_quaternion_y = bno_quaternion.w();
  double bno_quaternion_z = bno_quaternion.w();
  

  //double l3g_gyro_x = L3G_event.gyro.x;
  //double l3g_gyro_y = L3G_event.gyro.y;
  //double l3g_gyro_z = L3G_event.gyro.z;

  // Altitude in m
  // Pressure in Pa
  float mpl_alt = MPLPressure.readAltitude();
  mpl_alt -= altitude_offset;
  float mpl_pres = MPLPressure.readPressure();
  //float mpl_alt = MPLPressure.getAltitude();
  //float mpl_pres = MPLPressure.getPressure();

  if (maxA < x[0][0]) {
    maxA = x[0][0];
  }

  //Kalman(mpl_alt, accel_z);

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    
    LEDWRITING = true;

    //Serial.println("WITNESS ME!!!");
    //Serial.println(flightstate);
    //Serial.print("accelx: ");
    //Serial.print(accel_x, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("accely: ");
    //Serial.print(accel_y, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("accelz: ");
    //Serial.print(accel_z, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("bnogyrox: ");
    //Serial.print(bno_gyro_x, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("bnogyroy: ");
    //Serial.print(bno_gyro_y, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("bnogyroz: ");
    //Serial.print(bno_gyro_z, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print("mpl_alt: ");
    //Serial.print(mpl_alt,8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    //Serial.print(adxl_accel_x, 8); Serial.print(",");
    //Serial.print(adxl_accel_y, 8); Serial.print(",");
    //Serial.print(adxl_accel_z, 8); Serial.println(" ");
    
    ////Serial.print("l3ggyrox");
    ////Serial.print(l3g_gyro_x, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    ////Serial.print("l3ggyroy");
    ////Serial.print(l3g_gyro_y, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" ");
    ////Serial.print("l3ggyroz");
    ////Serial.print(l3g_gyro_z, 8); //Serial.print(","); //Serial.flush(); //Serial.println(" "); //Serial.flush();

    //Things we want by Sensor:
    //BNO-temperature,accel,gyro,magnetometer,euler,quaternion,linear accel
    //MPL-temperature,pressure,altitude
    //ADXL-accel


    dataFile.print(flightstate); dataFile.print(","); //dataFile.flush();
    dataFile.print(millis()); dataFile.print(","); //dataFile.flush();

    dataFile.print(bno_temp,8); dataFile.print(","); //dataFile.flush();
    dataFile.print(mpl_temp, 8); dataFile.print(","); //dataFile.flush();

    
    dataFile.print(bno_accel_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_accel_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_accel_z, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_lin_accel_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_lin_accel_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_lin_accel_z, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_gyro_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_gyro_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_gyro_z, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_euler_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_euler_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_euler_z, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_mag_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_mag_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_mag_z, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_quaternion_w, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_quaternion_x, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_quaternion_y, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(bno_quaternion_z, 8); dataFile.print(","); //dataFile.flush();
  
    dataFile.print(mpl_alt, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(mpl_pres, 8); dataFile.print(","); //dataFile.flush();
    dataFile.print(adxl_accel_x,8); dataFile.print(",");
    dataFile.print(adxl_accel_y,8); dataFile.print(",");
    dataFile.print(adxl_accel_z,8); dataFile.print(",");
    dataFile.print("\n"); dataFile.flush();

    /*
    // state matrix begins
    dataFile.print(x[0][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(x[1][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(x[2][0], 8); dataFile.print(","); dataFile.flush();
    // covariance begins
    dataFile.print(P[0][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[0][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[0][2], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[1][2], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][0], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][1], 8); dataFile.print(","); dataFile.flush();
    dataFile.print(P[2][2], 8); dataFile.print("\n"); dataFile.flush();
    */
    
    dataFile.close();
  } else {
    LEDWRITING = false;
  }

  switch(flightstate){
    case ARMED:
      if ( x[2][0] > accelLiftoffThreshold || ( x[0][0] - launchA) > baroLiftoffThreshold){
        flightstate = LAUNCHED;
      }
    break;
    case LAUNCHED:
      if ( x[2][0] < accelBurnoutThreshold){
        flightstate = BURNOUT;
      }
    break;
    case BURNOUT:
      if ( maxA > x[0][0] ){
        flightstate = APOGEE;
      }
    break;
    case APOGEE:
      if ( x[0][0] < launchA + 10 && x[1][0] < 1 ){
        flightstate = LANDED;
      }
    break;
    case LANDED:
    break;
  }

  digitalWrite(5, LEDWRITING);

  //delay(DELAY_TIME);
}

void Print_Header() {

  dataFile = SD.open(filename, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    //Serial.println("The header opened");
    dataFile.print("Flight State,"); dataFile.flush();
    dataFile.print("Time ms,"); dataFile.flush();
    dataFile.print("BNO Temperature C,"); dataFile.flush();
    dataFile.print("MPL Temperature C,"); dataFile.flush();
    dataFile.print("BNO X Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO Y Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO Z Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO X Linear Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO Y Linear Acceleration m/s^2,"); dataFile.flush();
    dataFile.print("BNO Z Linear Acceleration m/s^2,"); dataFile.flush();
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
    /*
    dataFile.print("Kalman Altitude,"); dataFile.flush();
    dataFile.print("Kalman Velocity,"); dataFile.flush();
    dataFile.print("Kalman Z Acceleration,"); dataFile.flush();
    dataFile.print("Covariance (3x3 Matrix)\n"); dataFile.flush();
  */
    
    dataFile.close();
  } else {
    // if the file didn't open, print an error:
    ////Serial.println("this boi don;t open");
  }

}
/*
void Kalman(float altitude,float zAccel) {
  dT = (millis() - lastT)/1000;
  float kTabs = 0; // UPDATE THIS WITH ACTUAL TAB DRAG

  // Start Kalman filter code
  mtx_type z[3][1] = {{altitude}, {x[1][0]}, {zAccel}};
  Matrix.Print((mtx_type *)z, 3, 1, "Current readings:");
  mtx_type K[3][3];
  mtx_type tempM[3][3];
  mtx_type temp_2_M[3][3];
  mtx_type tempV[3][1];

  
  // Calculate Kalman gain
  Matrix.Add((mtx_type *)P, (mtx_type *)R, 3, 3, (mtx_type *)tempM);
  Matrix.Invert((mtx_type *)tempM, 3);
  Matrix.Multiply((mtx_type *)P, (mtx_type *)tempM, 3, 3, 3, (mtx_type *)K);

  // Update estimate
  Matrix.Subtract((mtx_type *)z, (mtx_type *)x, 3, 1, (mtx_type *)tempV);
  Matrix.Multiply((mtx_type *)K, (mtx_type *)tempV, 3, 3, 1, (mtx_type *)z);
  Matrix.Add((mtx_type *)x, (mtx_type *)z, 3, 1, (mtx_type *)x);

  // Update covariance
  Matrix.Copy((mtx_type *)P, 3, 3, (mtx_type *)temp_2_M);
  Matrix.Subtract((mtx_type *)I3, (mtx_type *)K, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)temp_2_M, 3, 3, 3, (mtx_type *)P);

  
  // Project into next time step
  mtx_type tempTheta[3][3];
  Theta[0][1] = dT;
  Theta[0][2] = 0.5*dT*dT;
  Theta[1][2] = dT;
  Theta[2][1] = k + kTabs;
  Matrix.Copy((mtx_type *)x, 3, 1, (mtx_type *)tempV);
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)tempV, 3, 3, 1, (mtx_type *)x);
  Matrix.Transpose((mtx_type *)Theta, 3, 3, (mtx_type *)tempTheta);
  Matrix.Multiply((mtx_type *)Theta, (mtx_type *)P, 3, 3, 3, (mtx_type *)tempM);
  Matrix.Multiply((mtx_type *)tempM, (mtx_type *)tempTheta, 3, 3, 3, (mtx_type *)temp_2_M);
  Matrix.Add((mtx_type *)temp_2_M, (mtx_type *)Q, 3, 3, (mtx_type *)P);

  lastT = millis();

  //Matrix.Print((mtx_type *)x, 3, 1, "State of the world:");
  //Serial.println("\n");
  
}
*/
