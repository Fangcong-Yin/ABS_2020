#Manages state transitions for the ABS system


class StateManager():
    def __init__(self):
        self.state_list = ['Armed','Launched','Burnout','Apogee','Landed']
        self.current_state = 0

    #Takes in input data from Kalman Filter and adjusts the current state if necessary
    def check_transition(self,position,velocity,acceleration):
        #Use the syntax "self.current_state" to get the number representing the current state
        #Use the syntax "self.get_state()" to get the name of the state as a string
        pass

    def get_state(self):
        return self.state_list[self.current_state]