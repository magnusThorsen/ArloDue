"""
some simple first-order robot dynamics models
"""
import numpy as np
import time
from time import sleep

import robot
arlo = robot.Robot()


# Create a robot object and initialize

class RobotModel:

    def __init__(self, ctrl_range) -> None:
        #record the range of action that can be used to integrate the state
        self.ctrl_range = ctrl_range
        return
    
    def forward_dyn(self, x, u, T):
        #need to return intergated path of X with u along the horizon of T
        return NotImplementedError

    def inverse_dyn(self, x, x_goal, T):
        #return dynamically feasible path to move to the x_goal as close as possible
        return NotImplementedError
    
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

    def turnRobot(self, v1, v2):
        def unit_vector(vector):
            return vector / np.linalg.norm(vector)
        
        v1_u = unit_vector(v1)
        v2_u = unit_vector(v2)

        degrees = np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))
        
        if np.cross(v1, v2) > 0:
            self.turnRight(degrees)
        elif np.cross(v1, v2) < 0:
            self.turnLeft(degrees)

    def angle_between(self, p1, p2):
        ang1 = np.arctan2(*p1[::-1])
        ang2 = np.arctan2(*p2[::-1])

        degrees = np.rad2deg((ang1 - ang2) % (2 * np.pi))

        if degrees > 180:
            degrees -= 180
            self.turnLeft(degrees)
        elif degrees < 180:
            self.turnRight(degrees)

    
    
    A = (1, 0)
    B = (1, -1)
    print(angle_between(B, A))


class PointMassModel(RobotModel):
    #Note Arlo is differential driven and may be simpler to avoid Dubins car model by rotating in-place to direct and executing piecewise straight path  
    def forward_dyn(self, x, u, T):
        path = [x]
        #note u must have T ctrl to apply
        for i in range(T):
            x_new = path[-1] + u[i] #u is velocity command here
            path.append(x_new)    
        
        return path[1:]

    def inverse_dyn(self, x, x_goal, T):
        #for point mass, the path is just a straight line by taking full ctrl_range at each step
        dir = (x_goal-x)/np.linalg.norm(x_goal-x)

        u = np.array([dir*self.ctrl_range[1] for _ in range(T)])

        return self.forward_dyn(x, u, T)