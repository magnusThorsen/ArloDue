from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward
leftSpeed = 64
rightSpeed = 70
turnTime = 1.32
forwardTime = 1.5

def driveAndturn():
 print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
 
 # Wait a bit while robot moves forward
 sleep(forwardTime)
 
 # send a stop command
 print(arlo.stop())
 
 # turn 
 print(arlo.go_diff(0, rightSpeed, 1, 1))
 sleep(turnTime)
 
 # Wait a bit before next command
 sleep(0.041)

 # send a stop command
 print(arlo.stop())
 
 # Wait a bit before next command
 sleep(0.041)

def oneSquare():
 for i in range(4):
  driveAndturn()

#oneSquare()
#oneSquare()
#oneSquare()
#oneSquare()

def drive1Meter():
    print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))
    
    # Wait a bit while robot moves forward
    sleep(2.2)
    
    # send a stop command
    print(arlo.stop())
    
    # Wait a bit before next command
    sleep(0.041)

def turnnintydegrees():
   sleep(0.041)
   print(arlo.go_diff(leftSpeed, rightSpeed, 0, 1))

   sleep(turnTime/2-0.04)
 
   sleep(0.041)
   print(arlo.go_diff(0, 0, 0, 1))
   sleep(1.)


#drive1Meter()
turnnintydegrees()
turnnintydegrees()
turnnintydegrees()
turnnintydegrees()

""" 

# request to read Front sonar ping sensor
print("Front sensor = ", arlo.read_front_ping_sensor())
sleep(0.041)


# request to read Back sonar ping sensor
print("Back sensor = ", arlo.read_back_ping_sensor())
sleep(0.041)

# request to read Right sonar ping sensor
print("Right sensor = ", arlo.read_right_ping_sensor())
sleep(0.041)

# request to read Left sonar ping sensor
print("Left sensor = ", arlo.read_left_ping_sensor())
sleep(0.041)



# send a go_diff command to drive forward in a curve turning right
leftSpeed = 64
rightSpeed = 32
print(arlo.go_diff(leftSpeed, rightSpeed, 1, 1))

# Wait a bit while robot moves forward
sleep(3)

# send a stop command
print(arlo.stop())

# Wait a bit before next command
sleep(0.041)

# send a go_diff command to drive backwards the same way we came from
print(arlo.go_diff(leftSpeed, rightSpeed, 0, 0))

# Wait a bit while robot moves backwards
sleep(3)

# send a stop command
print(arlo.stop())
 """


print("Finished")
