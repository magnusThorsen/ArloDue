import time
from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()


sleep(1)

while(True):
    # request to read Front sonar ping sensor
    print("Front sensor = ", arlo.read_front_ping_sensor())
    if input() == "q":
        break

while(True):
    # request to read Back sonar ping sensor
    print("Back sensor = ", arlo.read_back_ping_sensor())
    if input() == "q":
        break

while(True):
    # request to read Right sonar ping sensor
    print("Right sensor = ", arlo.read_right_ping_sensor())
    if input() == "q":
        break

while(True):
    # request to read Left sonar ping sensor
    print("Left sensor = ", arlo.read_left_ping_sensor())
    if input() == "q":
        break
    
print("Finished")