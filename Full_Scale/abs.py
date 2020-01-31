# The main file
import Data_Logger
import Kalman
import PID

data_logger.init()

kalman.init()
servo.move(pid.test(kalman.predict(data_logger.state)))


def return_nums():
    a = 1
    b = 2
    c = 3

    return a,b,c

a,b,c = return_nums()
