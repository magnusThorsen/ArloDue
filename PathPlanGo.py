import time
from time import sleep
from pprint import *
import robot
import sys
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library
import matplotlib.pyplot as plt
import random

np.set_printoptions(threshold=sys.maxsize)

# Create a robot object and initialize
arlo = robot.Robot()


""" # Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100) """

def betterGoDiff(leftSpeed, rightSpeed, directionL, directionR, sleeptime):
   print(arlo.go_diff(leftSpeed/2, rightSpeed/2, directionL, directionR))
   sleep(0.1)
   print(arlo.go_diff(leftSpeed, rightSpeed, directionL, directionR))
   sleep(float(sleeptime) - 0.1)



def drive(distance):
    left_speed = 30
    right_speed = 34

    # Calculate time based on distance and wheel speeds
    #average_speed = (left_speed + right_speed) / 2
    time = distance / 16.75
    print("time",time)
    print("distance",distance)

    # Move the robot
    betterGoDiff(left_speed, right_speed, 1, 1, time)

    # Send a stop command
    print(arlo.stop())

    # Wait a bit before the next command
    sleep(0.5)



import rrt, robot_models

def pathPlanning():
    path = rrt.main()
    drive(10)
    robo = robot_models.RobotModel(1)
    """ robo.turnLeft(180)
    robo.turnRight(180) """
    robo.turnRobo([0,0],[0,1],[1,2])
    """ for i in range(len(path)-3):
        robo = robot_models.RobotModel(1)
        robo.turnRobo(path[i+1], path[i+2], path[i+3])
        distance = np.linalg.norm(np.subtract(path[i],path[i+1]))
        drive(distance * 10) """
    for i in range(len(path)-3):
        robo.turnRobo(path[i+1], path[i+2], path[i+3])
        distance = np.linalg.norm(np.subtract(path[i],path[i+1]))
        drive(distance * 10)

        
pathPlanning()
# Finished successfully


