#Manages state transitions for the ABS system


class StateManager():
    def __init__(self):
        self.state_list = ['Armed','Launched','Burnout','Apogee','Overshoot','Landed']
        self.current_state = 0
        self.threshold_of_liftoff = 0
        self.threshold_of_burnout = 0
        self.emergency_height = 1000
        self.apogee = 4444
        self.threshold_of_landing = 0
        #The exact values of threshold of liftoff,burnout, and landing should replace the 0's
  

    #Takes in input data from Kalman Filter and adjusts the current state if necessary
    def check_transition(self,height,velocity,acceleration):
        #Use the syntax "self.current_state" to get the number representing the current state
        #Use the syntax "self.get_state()" to get the name of the state as a string
        next_state = self.current_state

        if self.current_state == 0: #Armed
            if acceleration > self.threshold_of_liftoff or height > self.emergency_height:
                next_state = 1

        if self.current_state == 1: 
            if acceleration < self.threshold_of_burnout or height > self.emergency_height:
                next_state = 2
            
            if 

        if self.current_state == 0:
            pass
        if self.current_state == 0:
            pass
        if self.current_state == 0:
            pass
        if self.current_state == 0:
            pass

        self.current_state = next_state


    def get_state(self):
        return self.state_list[self.current_state]
