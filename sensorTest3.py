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
   print(arlo.go_diff(64, 68, 0, 1))

   sleep(0.0074 * degree + ((degree**2)*0.000001))
   # send a stop command
   print(arlo.stop())
    
   # Wait a bit before next command
   sleep(0.5)

def turnRight(degree):
   sleep(0.041)
   print(arlo.go_diff(64, 68, 1, 0))

   sleep(0.0074 * degree + ((degree**2)*0.000001))
   # send a stop command
   print(arlo.stop())
    
   # Wait a bit before next command
   sleep(0.5)

rotation = 0

isDriving = True
while (isDriving): # or some other form of loop
        
    print(arlo.go_diff(64, 68, 1, 1))
    frontSensor = arlo.read_front_ping_sensor()
    backSensor = arlo.read_back_ping_sensor()
    rightSensor = arlo.read_right_ping_sensor()
    leftSensor = arlo.read_left_ping_sensor()

    while frontSensor < 300 or rightSensor < 300 or leftSensor < 300:
        print(arlo.stop())
        sleep(0.5)
        frontSensor = arlo.read_front_ping_sensor()
        rightSensor = arlo.read_right_ping_sensor()
        leftSensor = arlo.read_left_ping_sensor()
        if rightSensor < 300:
            rotation -= 90
            turnLeft(90)
            sleep(1.0)
            
        elif leftSensor < 300:
            rotation += 90
            turnRight(90)
            sleep(1.0)
            
        elif frontSensor < 300:
            if leftSensor > 300 and rightSensor > 300:
               if rotation = 90:
                  turnLeft(90)
                  rotation -= 90
               else:
                  turnRight(90)
                  rotation += 90
            if leftSensor < rightSensor:
                rotation += 90
                turnRight(90)
                sleep(1.0)
            else:
                rotation -= 90
                turnLeft(90)
                sleep(1.0)

        

print("Finished")
