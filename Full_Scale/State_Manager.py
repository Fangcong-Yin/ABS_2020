#Manages state transitions for the ABS system


class StateManager():
    def __init__(self):
        self.state_list = ['Armed','Launched','Burnout','Apogee','Landed']
        self.current_state = 0
        self.threshold_of_liftoff = 0
        self.threshold_of_burnout = 0
        self.apogee = 4444
        self.threshold_of_landing = 0

    #Takes in input data from Kalman Filter and adjusts the current state if necessary
    def check_transition(self,position,velocity,acceleration):
        #Use the syntax "self.current_state" to get the number representing the current state
        #Use the syntax "self.get_state()" to get the name of the state as a string
        if(self.current_state ==0 and acceleration >= self.threshold_of_liftoff):
            self.current_state = 1
        elif(self.current_state ==1 and acceleration <= self.threshold_of_burnout):
            self.current_state = 2
        elif(self.current_state ==2 and position >= self.apogee):
            self.current_state = 3
        eif(self.current_state ==3 and velocity <= self.threshold_of_landing):
            self.current_state = 4
        pass

    def get_state(self):
        return self.state_list[self.current_state]
