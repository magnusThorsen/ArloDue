import numpy as np
import time
from time import sleep

import robot
arlo = robot.Robot()


# Create a robot object and initialize

def turnLeft(degree):
    sleep(0.041)
    print(arlo.go_diff(64, 70, 0, 1))

    sleep(0.0074 * degree + ((degree**2)*0.000001))
    # send a stop command
    print(arlo.stop())
        
    # Wait a bit before next command
    sleep(0.5)

def turnRight(degree):
    sleep(0.041)
    print(arlo.go_diff(64, 70, 1, 0))

    sleep(0.0074 * degree + ((degree**2)*0.000001))
    # send a stop command
    print(arlo.stop())
        
    # Wait a bit before next command
    sleep(0.5)