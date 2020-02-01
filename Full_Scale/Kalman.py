import numpy as np
import matplotlib.pyplot as plt
import time

"""
Variables:
x-state vector
phi-state transition matrix
H-state to sensor matrix


Kalman Filter-defined with three parameters: var_m, var_s, and var_a
Current default: 3,.1,5-will want to extensively test different values

Use these three to initialize the Kalman object
Can also set the state by passsing in a 3D vector (y,v_y,a_y)
Passing in measurements: send in a 2D NP array (y,a_y)

body_to_inertial: transforms roll, pitch, yaw of BNO into an angle W.R.T the vertical (or ground)
accel_trans: transforms the acceleration into the inertial frame

NOTE: May have to reconfigure sone signs to get the dimensions of the BNO to line up with the
dimensions of the ADXL 

DataFilter-abstracts all of this stuff
Feed in output from DataLogger, get out everything the PID controller needs

Usage: after initialization, call process_data() on the output from DataLogger. 
Output: timestamp, angle, height, vertical velocity, vertical acceleration

Good practice: run set_time() before you begin filtering
"""

#Implements the Kalman Filter-May later be replaced by the FilterPy Library
class Kalman():
    def __init__(self,var_m,var_s,var_a):
        self.T = 0

        self.x = np.array([0,0,0])
        self.gen_phi()
        self.H = np.array([[1,0,0],
                           [0,0,1]])

        self.Q = np.array([[0,0,0],
                           [0,0,0],
                           [0,0,var_m]])
        self.R = np.array([[var_s,0],
                           [0,var_a]])

        self.P = np.zeros((3,3))
        self.K = np.zeros((3,2))
        self.I = np.eye(3)

    #Generate the transition matrix from the delta t
    def gen_phi(self):
        self.phi = np.array([[1,self.T,self.T**2/2],
                            [0,1,self.T],
                            [0,0,1]])

    #Updates the state
    def update_state(self,in_z,delta_t): #in_z is input sensor readings
        self.T = delta_t
        self.gen_phi()

        xhatpre = self.phi@self.x
        xhatpost = xhatpre+self.K@(in_z-self.H@xhatpre)

        Pkpre = self.phi@self.P@self.phi.T+self.Q
        Pkpost = (self.I-self.K@self.H)@Pkpre

        Kk = Pkpre@self.H.T@np.linalg.inv(self.H@Pkpre@self.H.T+self.R)

        self.x = xhatpost
        self.P = Pkpost
        self.K = Kk

    #Outputs the current state
    def current_state(self):
        return self.x

    def set_state(self,x):
        self.x = x




#Abstracts the Kalman filter
class DataFilter():
    def __init__(self):
        var_m = 3
        var_s = .1
        var_a = 5 
        self.filter = Kalman(var_m,var_s,var_a)
        
        self.ref = np.array([0,1,0]).T

        self.timestamp = time.time()

    def start_time(self):
        self.timestamp = time.time()

    #Constructs a matrix to move from the body frame to the inertial frame
    def body_to_inertial(self,roll,pitch,yaw):
        c = lambda x: np.cos(x)
        s = lambda x: np.sin(x)

        out = np.zeros((3,3))
        out[0,0] = c(yaw)*c(pitch)
        out[0,1] = c(yaw)*s(roll)*s(pitch)-c(roll)*s(yaw)
        out[0,2] = s(roll)*s(yaw)+c(roll)*c(yaw)*s(pitch)
        out[1,0] = c(pitch)*s(yaw)
        out[1,1] = c(roll)*c(yaw)+s(roll)*s(yaw)*s(pitch)
        out[1,2] = c(roll)*s(yaw)*s(pitch)-c(yaw)*s(roll)
        out[2,0] = -s(pitch)
        out[2,1] = c(pitch)*s(roll)
        out[2,2] = c(roll)*c(pitch)

        return out


    #Processes data from Data_Logger-need to maintain formatting!
    def process_data(self,data):
        timestamp,euler,altitude,accel = data
        
        #convert euler angles to radians
        roll  = euler[0]*np.pi/180
        pitch = euler[1]*np.pi/180
        yaw   = euler[2]*np.pi/180

        #create vertical acceleration and angle with vertical
        rot = self.body_to_inertial(roll,pitch,yaw)
        theta = rot@self.ref #create theta from a rotation matrix
        accel = rot@accel    #transform accel to the inertial frame
        accel = accel[1]

        #handle time
        dt = timestamp-self.timestamp
        self.timestamp = timestamp

        #filter the data
        z = np.array([altitude,accel])
        self.filter.update_state(z,dt)

        #parse and output the results
        x = self.filter.current_state()

        return self.timestamp,theta,x[0],x[1],x[2]