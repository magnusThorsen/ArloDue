import time
from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()


sleep(1)


# request to read Front sonar ping sensor
print("Front sensor = ", arlo.read_front_ping_sensor())

# request to read Back sonar ping sensor
print("Back sensor = ", arlo.read_back_ping_sensor())

# request to read Right sonar ping sensor
print("Right sensor = ", arlo.read_right_ping_sensor())

# request to read Left sonar ping sensor
print("Left sensor = ", arlo.read_left_ping_sensor())  
    
print(arlo.go_diff(64, 64, 1, 1))
isDriving = True
while (isDriving): # or some other form of loop
    frontSensor = arlo.read_front_ping_sensor()
    backSensor = arlo.read_back_ping_sensor()
    rightSensor = arlo.read_right_ping_sensor()
    leftSensor = arlo.read_left_ping_sensor()

    if frontSensor < 500:
        print(arlo.go_diff(64, 64, 0, 0))
        sleep(0.5)
        isDriving = False
        

print("Finished")
