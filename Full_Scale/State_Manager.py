#Manages state transitions for the ABS system


class StateManager():
    def __init__(self):
        self.state_list = ['Armed','Launched','Burnout','Apogee','Overshot','Landed']
        self.current_state = 0
        #Threshold values for lift off
        self.threshold_acc_of_liftoff = 130 #130ft/s²
        self.threshold_height_of_liftoff = 100 #in ft
        #Threshold values for burn out
        self.threshold_acc_of_burnout = -20 #-20ft/s²
        self.emergency_height = 1500
        #Threshold values for apogee
        self.apogee = 4444
        self.threshold_height_of_landing = 30
       
  

    #Takes in input data from Kalman Filter and adjusts the current state if necessary
    def check_transition(self,height,velocity,acceleration):
        #Use the syntax "self.current_state" to get the number representing the current state
        #Use the syntax "self.get_state()" to get the name of the state as a string
        next_state = self.current_state

        if self.current_state == 0: #Armed
            if acceleration > self.threshold_of_liftoff or height > self.threshold_height_of_liftoff:
                next_state = 1

        if self.current_state == 1: #Launched
            if acceleration < self.threshold_of_acc_burnout or height > self.emergency_height:
                next_state = 2

        if self.current_state == 2: #Burnout
            if acceleration > self.threshold_acc_of_burnout:
                next_stage = 1
                #Return to the Launched stage because the noises in acceleration
            if height >= self.apogee and velocity <=0:
                next_stage = 3
                #Change to the Apogee stage
            elif height > self.apogee and velocity > 0:
                next_stage = 4
                #Change to the Overshot stage
                
        if self.current_state == 3: #Apogee
            if velocity > 0:
                next_stage = 2
                #Return to the Burnout stage if the velocity is still greater than 0
            if height < self.threshold_height_of_landing:
                next_stage = 4
                #Change to the Landed Stage
                
        if self.current_state == 4: #Overshot
            if velocity == 0:
                next_stage = 3
            #Change to Apogee stage

        self.current_state = next_state


    def get_state(self):
        return self.state_list[self.current_state]
