import numpy as np
import matplotlib.pyplot as plt
import openpyxl

"""
Variables:
x-state vector
phi-state transition matrix
H-state to sensor matrix
"""

#Constructs a matrix to move from the body frame to the inertial frame
def body_to_inertial(roll,pitch,yaw):
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

#Transforms body acceleration to inertial acceleration
def accel_trans(a,roll,pitch,yaw):
    R = body_to_inertial(roll,pitch,yaw)
    #R = inertial_to_body(roll,pitch,yaw)
    a_trans = R@a

    return a_trans


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


#reads in a row of the sheet and creates a measurement
def create_measurement(row,sheet):
    #Read in BNO Absolute Orientation and convert to radians
    roll = sheet['H'+str(row)].value*np.pi/180
    pitch = sheet['I'+str(row)].value*np.pi/180
    yaw = sheet['J'+str(row)].value*np.pi/180

    #Read in altitude
    y = sheet['N'+str(row)].value

    #Read in ADXL Acceleration and translate to vertical
    ax = sheet['O'+str(row)].value
    ay = sheet['P'+str(row)].value
    az = sheet['Q'+str(row)].value
    a_trans = accel_trans(np.array([ax,ay,az]),roll,pitch,yaw)

    return np.array([y,a_trans[1]]) 

if __name__ == '__main__':
    wb = openpyxl.load_workbook('NoTab.xlsx')
    sheet = wb['Filtered Data']

    #Set the parameters of the variances
    var_m = 3
    var_s = .1
    var_a = 5 
    filty = Kalman(var_m,var_s,var_a)

    states = []
    x_rec = []
    a_rec = []

    prev_t = sheet['A2'].value
    for i in range(2,200):#len([i for i in sheet.rows])):
        z = create_measurement(i,sheet)

        t = sheet['A'+str(i)].value
        delta_t = t - prev_t
        prev_t = t

        filty.update_state(z,delta_t)
        states.append(filty.current_state())
        x_rec.append(z[0])
        a_rec.append(z[1])

    #Plot Output
    states = np.array(states)
    plt.clf()
    plt.subplot(131)
    plt.plot(x_rec,label='Sensor')
    plt.plot(states[:,0],label='Kalman')
    plt.xlabel('t (seconds)')
    plt.ylabel('y (meters)')
    plt.legend()
    plt.subplot(132)
    plt.plot(states[:,1],label='Kalman')
    plt.xlabel('t (seconds)')
    plt.ylabel('v (meters/second)')
    plt.legend()
    plt.subplot(133)
    plt.plot(a_rec,label='Sensor')
    plt.plot(states[:,2],label='Kalman')
    plt.xlabel('t (seconds)')
    plt.ylabel('a (meters/second^2)')
    plt.legend()
    plt.show()
