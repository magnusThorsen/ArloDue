import time
from time import sleep
from pprint import *
import sys
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library
import mknRallySelf2 as rs

np.set_printoptions(threshold=sys.maxsize)

# Create a robot object and initialize
onRobot = True # Whether or not we are running on the Arlo robot
needNewLocation = False
try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

print("OpenCV version = " + cv2.__version__)

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot

if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../Arlo/python")


try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False


arlo = robot.Robot()

isDriving = False

xSize = 640
ySize = 480
#focal = 350
focal = 1335.517241

# Open a camera device for capturing
imageSize = (xSize, ySize)
FPS = 30
cam = picamera2.Picamera2()
frame_duration_limit = int(1/FPS * 1000000) # Microseconds
# Change configuration to set resolution, framerate
picam2_config = cam.create_video_configuration({"size": imageSize, "format": 'RGB888'},
                                                            controls={"FrameDurationLimits": (frame_duration_limit, frame_duration_limit)},
                                                            queue=False)
cam.configure(picam2_config) # Not really necessary
cam.start(show_preview=False)

pprint(cam.camera_configuration()) # Print the camera configuration in use

time.sleep(1)  # wait for camera to setup

# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [1, 2, 3, 4, 1] # 3 er fjernet
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}
landmark_colors = [CRED, CGREEN, CBLUE, CYELLOW] # Colors used when drawing the landmarks

def betterGoDiff(leftSpeed, rightSpeed, directionL, directionR, sleeptime):
   print(arlo.go_diff(leftSpeed/2, rightSpeed/2, directionL, directionR))
   sleep(0.1)
   print(arlo.go_diff(leftSpeed, rightSpeed, directionL, directionR))
   sleep(float(sleeptime) - 0.1)

def turnRobo(angle, particles):
    # convert angle to degrees:
    angle = angle * 180 / np.pi
    print("Angle: ", angle)
    # turn the robot accordingly
    if angle > 0:
        turnLeft(angle, particles)
    elif angle < 0:
        turnRight(-angle, particles)
    else:
        print("No turn needed")


def angleCalc(tvec):
    beta = np.arccos(np.dot( (tvec/np.linalg.norm(tvec)) , (0,0,1)))
    print("beta", beta)
    return beta[0][0]


""" def drive(distance):
    isDriving = True
    sensor(isDriving)
    left_speed = 31
    right_speed = 37.5

    # Calculate time based on distance and wheel speeds
    #average_speed = (left_speed + right_speed) / 2
    shortdist = distance - 20
    time = shortdist / 16.75 
    print("drive: time",time)
    print("drive: distance",distance)

    # Move the robot
    betterGoDiff(left_speed, right_speed, 1, 1, time)

    # Send a stop command
    print(arlo.stop())

    # Wait a bit before the next command
    sleep(0.5) """


def driveWithTime(distance, particles):
    rs.driveWithTime(distance, particles)
            
def turnLeft(degree, particles):
    rs.turnLeft(degree, particles)

def turnRight(degree, particles):
    rs.turnRight(degree, particles)

""" def sensor(isDriving):
    while (isDriving): # or some other form of loop
        frontSensor = arlo.read_front_ping_sensor()
        print(frontSensor)
        backSensor = arlo.read_back_ping_sensor()
        rightSensor = arlo.read_right_ping_sensor()
        leftSensor = arlo.read_left_ping_sensor()   
        if frontSensor < 250 or rightSensor < 200 or leftSensor < 200:
            print(arlo.stop())
            sleep(0.2)
            if rightSensor < 300:
                turnLeft(90)
                sleep(0.2)
            elif leftSensor < 300:
                turnRight(90)
                sleep(0.2)
            elif frontSensor < 250:
                if leftSensor <= rightSensor:
                    turnRight(90)
                    sleep(0.2)
                else:
                    turnLeft(90)
                    sleep(0.2)
        else: 
            print(arlo.go_diff(64, 68, 1, 1)) """

def searchAndShowLandmark(ImpID): 
    detected = False
    # Load the image
    image = cam.capture_array("main")  # Load your image here

    # Define the dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Detect markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary)

    cameraMatrix = np.array([[focal, 0, xSize/2],
                            [0, focal, ySize/2],
                            [0, 0, 1]])
    
   # Draw the detected markers on the image
    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        # Estimate pose for each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 200, cameraMatrix, None)

        # Iterate through the detected markers and print their IDs and pose information
        for i in range(len(ids)):
            marker_id = ids[i][0]
            translation_vector = tvecs

            # Calculate the Euclidean distance (norm) from the camera to the marker
            distance = np.linalg.norm(translation_vector)

            print(f"sasLandmark: Detected Marker ID: {marker_id}")
            print(f"sasLandmark: Distance to Marker {marker_id}: {distance} units")
            if marker_id == ImpID:
                detected = True
                return detected, distance, translation_vector
    # Display the image with detected markers
    cv2.imshow("sasLandmark: Detected Markers", image)
    return detected, 0.0, None

def turnDetectLandmark(landmarkID, particles):
    counter = 0
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        # print go diff 
        print("tdLandmark: Finding landmark: ", landmarkID)
        detected, distance, t_vec = searchAndShowLandmark(landmarkID)
        if counter == 21:
            print(arlo.stop())
            landmarkFound = False
            return landmarkFound, 0.0, None
        if not detected: 
            turnLeft(20, particles)
            sleep(0.9)
            counter += 1
            print("tdLandmark: This is the counter: ", counter)
        else: 
            print(arlo.stop())
            landmarkFound = True
            return landmarkFound, (distance / 14.086079), t_vec

def searchAndShowObstacle():
    detected = False
    # Load the image
    image = cam.capture_array("main")  # Load your image here

    # Define the dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Detect markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(image, dictionary)

    cameraMatrix = np.array([[focal, 0, xSize/2],
                            [0, focal, ySize/2],
                            [0, 0, 1]])
    
    # Draw the detected markers on the image
    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        
        # Estimate pose for each detected marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 200, cameraMatrix, None)

        # Iterate through the detected markers and print their IDs and pose information
        for id in range(len(ids)):
            marker_id = ids[id][0]
            translation_vector = tvecs

            # Calculate the Euclidean distance (norm) from the camera to the marker
            distance = np.linalg.norm(translation_vector)

            print(f"sasObstacle: Detected Marker ID: {marker_id}")
            print(f"sasObstacle: Distance to Marker {marker_id}: {distance} units")

            if marker_id not in landmarkIDs:
                detected = True
                return detected, distance, marker_id
    return detected, 0.0, 0

def turnDetectObstacle(particles):
    counter = 0
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        # print go diff 
        detected, distance, id = searchAndShowObstacle()
        if counter == 21:
            print(arlo.stop())
            return detected, 0.0, 0
        if not detected: 
            turnLeft(20, particles)
            sleep(0.9)
            counter += 1
            print("tdObstacle: This is the counter: ", counter)
        else: 
            print(arlo.stop())
            return detected, (distance / 14.086079), id

def reposition(visitedObstacles, particles):
    detected, distance, id = turnDetectObstacle()
    print("reposition: Detected in reposition: ", id)
    print("reposition: Visited obstacles: ", visitedObstacles)
    if detected and id not in visitedObstacles and id != 0:
        print("reposition: driving at ",id )
        #TURN TO OBSTACLE
        visitedObstacles.append(id)
        driveWithTime(distance/2, particles)
    else: reposition(visitedObstacles, particles)
    return visitedObstacles      


def main():
    for landmark in landmarkIDs:
        visitedObstacles = []
        landmarkReached = False
        numtries = 0
        while not landmarkReached:
            particles = rs.initialize_particles(num_particles=1000)
            rs.selfLocalize(particles)
            detected, distance, tvecs = turnDetectLandmark(landmark, particles)
            print(landmark)
            if detected:
                print("Main: Found the landmark: " ,landmark)
                # Find the distance of the landmark
                print("Main: Distance: ", distance)
                # Drive to the landmark
                # Turn to landmark
                # SENSORES
                turnRobo(angleCalc(tvecs)*0.8, particles)
                print("tvecs", tvecs)
                
                if distance < 150:
                    if driveWithTime(distance, particles):
                        landmarkReached = True
                else: 
                    driveWithTime(70, particles)
                    turnRight(30, particles)

                # Self localize and create a path to the landmark
            else: 
                print("Main: didn't find the landmark")
                visitedObstacles = reposition(visitedObstacles, particles)
                numtries += 1
                if numtries > 3: 
                    turnRight(90, particles)
                    driveWithTime(4, particles)
                    numtries = 0
                    visitedObstacles = []
                print("Main: Visited obstacles: ", visitedObstacles)
    print("Succesfully completed the course! Time: 00:15 minutes")
        


main()


