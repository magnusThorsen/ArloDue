from time import sleep

import robot

# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")


# send a go_diff command to drive forward
leftSpeed = 64
rightSpeed = 66

def halfeight():
 print(arlo.go_diff(80, 30, 1, 1))
 
 # Wait a bit while robot moves forward
 sleep(6)
 
 # turn 
 print(arlo.go_diff(30, 80, 1, 1))
 sleep(6)
 

def eight():
 for i in range(2):
  halfeight()

eight()
eight()


print("Finished")
