import time
from time import sleep
from pprint import *
import robot
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library

# Create a robot object and initialize
arlo = robot.Robot()

try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

print("OpenCV version = " + cv2.__version__)

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


""" # Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100) """

def betterGoDiff(leftSpeed, rightSpeed, directionL, directionR, sleeptime):
   print(arlo.go_diff(leftSpeed/2, rightSpeed/2, directionL, directionR))
   sleep(0.1)
   print(arlo.go_diff(leftSpeed, rightSpeed, directionL, directionR))
   sleep(sleeptime-0.1)




def drive30CM():
    betterGoDiff(30, 35, 1, 1, 1.6)
    # Wait a bit while robot moves forward
   
    
    # send a stop command
    print(arlo.stop())
    
    # Wait a bit before next command
    sleep(0.5)

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


def turnLeft(degree):
   sleep(0.041)
   print(arlo.go_diff(64, 70, 0, 1))

   sleep(0.0074 * degree + ((degree**2)*0.000013))
   # send a stop command
   print(arlo.stop())
    
   # Wait a bit before next command
   sleep(0.5)
   
def searchAndshow(): 
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
    rvecs = None
    tvecs = None
    marker_id = None
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

            """ print(f"Detected Marker ID: {marker_id}")
            print(f"Distance to Marker {marker_id}: {distance} units") """
        detected = True

    # Display the image with detected markers
    cv2.imshow("Detected Markers", image)
    return detected, rvecs, tvecs, marker_id

""" 
val = True

detectedLandmarksReal = list()
detectedLandmarks = list()
counter = 0
while counter < 53: # Wait for a key pressed event (cv2.waitKey(4) == -1)
    # print go diff
    detected, _, tvecs, marker = searchAndshow()
    if marker in detectedLandmarks or marker == None:
       ()
    else: 
        detectedLandmarksReal.append((marker, tvecs))
        detectedLandmarks.append(marker)

        
    print(detectedLandmarksReal)
    print(arlo.go_diff(32, 35, 0, 1))
    sleep(0.15)
    print(arlo.stop())
    sleep(0.1)
    counter += 1 """

""" def calculateRadius(x, y):
    radius =  """

def detectCollision(x, y):
    robotRadius = 23
    for i in detectedLandmarksReal:
        if np.sqrt((x-i[0])**2 + (y-i[1])**2) < calculateRadius(i[0], i[1]) + robotRadius:
            return True

        return False

    
    



detected, _, tvecs, marker = searchAndshow()
print("marker",marker)

detected, _, tvecs, marker = searchAndshow()
print("marker",marker)

detected, _, tvecs, marker = searchAndshow()
print("marker",marker)

# Finished successfully