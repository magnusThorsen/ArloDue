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

def turnLeft(degree):
   sleep(0.041)
   print(arlo.go_diff(64, 64, 0, 1))

   sleep(0.0074 * degree + ((degree**2)*0.000001))
   # send a stop command
   print(arlo.stop())
    
   # Wait a bit before next command
   sleep(0.5)

print(arlo.go_diff(40, 40, 1, 1))
isDriving = True
while (isDriving): # or some other form of loop
    frontSensor = arlo.read_front_ping_sensor()
    backSensor = arlo.read_back_ping_sensor()
    rightSensor = arlo.read_right_ping_sensor()
    leftSensor = arlo.read_left_ping_sensor()

    if frontSensor < 200:
        print(arlo.stop())
        sleep(0.5)
        """ print(arlo.go_diff(64, 64, 0, 0))
        sleep(0.5) """
        isDriving = False
        

print("Finished")
