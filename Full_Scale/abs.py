# The main file
from Data_Logger import *
from Kalman import *
from State_Manager import *

#TODO: consider adding parallelization
dlogger = DataLogger()
dfilter = DataFilter()
state_machine = StateManager()

while True:
    #Read in data from the data logger
    dlogger.read_data()
    dlogger.write_data()
    raw_data = dlogger.get_data()

    #Filter the data
    t,theta,y,v,a = dfilter.process_data(raw_data)

    #Check flight state
    state_machine.check_transition(y,v,a)
    state = state_machine.get_state()

    #do PID stuff if necessary
    if state == 'Burnout':
        pass
