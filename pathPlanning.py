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
    returnlst = list()
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
        """ print("tvecs", tvecs)
        print("rvecs", rvecs)
        print("corners", corners)
        print("ids", ids) """

        # Iterate through the detected markers and print their IDs and pose information
        for i in range(len(ids)):
            marker_id = ids[i][0]
            translation_vector = tvecs[i]

            # Calculate the Euclidean distance (norm) from the camera to the marker
            distance = np.linalg.norm(translation_vector) / 14.086079
            ycord = rvecs[i][0]/np.linalg.norm(rvecs[i][0])
            
            returnlst.append((marker_id, tvecs[i][0][2]/14.086079, tvecs[i][0][0]/-14.086079))

            """ print(f"Detected Marker ID: {marker_id}")
            print(f"Distance to Marker {marker_id}: {distance} units") """

        detected = True

    # Display the image with detected markers
    cv2.imshow("Detected Markers", image)
    return returnlst


def detectCollision(x, y):
    robotRadius = 23
    for i in detectedLandmarksReal:
        if np.sqrt((x-i[0])**2 + (y-i[1])**2) < calculateRadius(i[0], i[1]) + robotRadius:
            return True

        return False

    
    


detectedLandmarksReal = (searchAndshow())
#print(detectedLandmarksReal)
#make a plot of the landmarks and the robot in 0.0

def makePlot():
    fig, ax = plt.subplots()
    ax.set_xlim(-50, 700)
    ax.set_ylim(-200, 200)
    ax.set_aspect(1)
    ax.plot(0, 0, 'o', color='red')
    for i in detectedLandmarksReal:
        # plot the landmarks with names on the dots: 
        ax.plot(i[1], i[2], 'o', color='blue')
        ax.annotate(i[0], (i[1], i[2]))
        
    plt.show() 

def getmap():
    #create a boolean 2d array of size 70 * 40 and set the values to true: 
    map = np.ones((35, 20), dtype=bool)
    # check if cordinates from detected landmarks are in the map and set the values to false:
    for i in detectedLandmarksReal:
        # if location is on the map, set the value to false:
        if i[1]/2 > 0 and i[1]/2 < 350 and i[2]/2 > -100 and i[2]/2 < 100:
            x,y = int(i[1]/20), int(i[2]/20) 
            # set all values in a circle around the landmark of 3 to false:
            for j in range(-3, 3):
                for k in range(-3, 3):
                    map[x+j][y+k] = False
        else:
            ()
    return map
    
#RRT Pseudo Code
#Qgoal //region that identifies success
#Counter = 0 //keeps track of iterations
#lim = n //number of iterations algorithm should run for
#G(V,E) //Graph containing edges and vertices, initialized as empty
#While counter < lim:
#    Xnew  = RandomPosition()
#    if IsInObstacle(Xnew) == True:
#        continue
#    Xnearest = Nearest(G(V,E),Xnew) //find nearest vertex
#    Link = Chain(Xnew,Xnearest)
#    G.append(Link)
#    if Xnew in Qgoal:
#        Return G
#Return G 
#make RRT

#RRT takes a map and a goal as input



def move(coordinates):
    x, y = coordinates
    direction = random.choice(['x', 'y'])
    
    if direction == 'x':
        x += random.choice([-1, 1])
    else:
        y += random.choice([-1, 1])

    x = min(max(x, 0), 10)
    y = min(max(y, 0), 10)
    
    return x, y



def reset_coordinates():
    return (0, 0)

def check_coordinates(map,coordinates):
    x,y = coordinates
    if x > 69 or x < 0 or y > 19 or y < -19:
        return True
        #check if the point is in an obstacle
    if map[x][y] == False:
        return True
    return False

def RRT2(map, goal):
    shortest_path = None
    numtries = 0
    current_coordinates = reset_coordinates()
    path = [current_coordinates]
    visited = set()
    visited.add(current_coordinates)
    tries = 0
    
    while numtries < 100:
        if current_coordinates == goal:
            if shortest_path == None or len(path) < len(shortest_path):
                shortest_path = path
                print("Current shortest Path",shortest_path, len(shortest_path))
            
            current_coordinates = reset_coordinates()
            path = [current_coordinates]
            visited = set()
            visited.add(current_coordinates)
            tries = 0
            numtries += 1
                
        tmp_coordinates = move(current_coordinates)
        if check_coordinates(map,tmp_coordinates):
            tries += 1
            continue
        if tmp_coordinates not in visited :
            current_coordinates = tmp_coordinates
            path.append(current_coordinates)
            visited.add(current_coordinates)
        tries += 1

        if tries >= 10000:
            current_coordinates = reset_coordinates()
            path = [current_coordinates]
            visited = set()
            visited.add(current_coordinates)
            tries = 0
            numtries += 1
            # print numtries mod 100
    return shortest_path


# create a 70 * 40 np bool array and set three blobs of false and the rest true: 
# 1. a 20*20 square in the middle
# 2. a 10*10 square in the bottom left corner
# 3. a 10*10 square in the top right corner
map1 = np.ones((70, 40), dtype=bool)
map1[5:8, 5:10] = False

themap = getmap()
#use matplotlib to show the map and the path 
import matplotlib.pyplot as plt
path = RRT2(themap, (10,10))
print(path, len(path))
path = np.array(path)
plt.imshow(themap.T, cmap='Greys', origin='lower')
plt.plot(path[:,0], path[:,1], 'r-')
plt.show()




# Finished successfully


