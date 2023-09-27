import time
from time import sleep
from pprint import *
import robot
import sys
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library
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

import matplotlib.pyplot as plt
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
    map = np.ones((70, 40), dtype=bool)
    # check if cordinates from detected landmarks are in the map and set the values to false:
    for i in detectedLandmarksReal:
        # if location is on the map, set the value to false:
        if i[1] > 0 and i[1] < 700 and i[2] > -200 and i[2] < 200:
            x,y = int(i[1]/10), int(i[2]/10) 
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
""" def RRT(map,goal):
    startpoint = (0,0)
    x_goal, y_goal = goal
    path = list()
    path.append(startpoint)
    counter_x = 0
    counter_y = 0
    notThereYet = True
    changedvariable = None
    path_counter = 0
    while notThereYet:  
        path_counter += 1
        if path_counter > 1000:
            path_counter = 0
            counter_x = 0
            counter_y = 0

            print("no path found", path)
            path = list()
            path.append(startpoint)
            continue
        #choose x or y randomly
        if np.random.randint(0,2) == 0:
            if np.random.randint(0,2) == 0:
                counter_x += 1
                changedvariable = (counter_x, 1)
            else:
                counter_x -= 1
                changedvariable = (counter_x, -1)
        else:
            if np.random.randint(0,2) == 0:
                counter_y += 1
                changedvariable = (counter_y, 1)
            else:
                counter_y -= 1
                changedvariable = (counter_x, -1)
        # if already in path, continue
        if (counter_x, counter_y) in path:
            continue
        #check if the point is in the map
        if counter_x > 11 or counter_x < 0 or counter_y > 11 or counter_y < 0:
            if changedvariable[0] == counter_x:
                counter_x -= changedvariable[1]
            else: 
                counter_y -= changedvariable[1]
            continue
        #check if the point is in an obstacle
        if map[counter_x][counter_y] == False:
            continue
        #check if the point is in the goal
        if counter_x == x_goal and counter_y == y_goal:
            notThereYet = False
        #add the point to the path
        path.append((counter_x, counter_y))
        #print(path)
    return path


    

print("the right path: " , RRT((np.ones((70, 40), dtype=bool)), (10, 10))) """



# Function to visualize the path
def visualize_path(map, path):
    plt.imshow(map, cmap='gray')
    plt.colorbar()
    plt.scatter(*zip(*path), color='red', marker='.')
    plt.gca().invert_yaxis()
    plt.show()

# RRT takes a map and a goal as input
def RRT(map, goal):
    x_goal, y_goal = goal
    path = []
    path.append((0, 0))

    while True:
        # Randomly sample a point
        sample = (np.random.randint(0, 11), np.random.randint(0, 11))

        # Find the nearest point in the path
        nearest = min(path, key=lambda x: np.linalg.norm(np.array(x) - np.array(sample)))

        # Steer towards the sampled point within a maximum step size
        max_step = 1
        direction = np.array(sample) - np.array(nearest)
        if np.linalg.norm(direction) > max_step:
            direction = direction / np.linalg.norm(direction) * max_step
        new_point = tuple(np.array(nearest) + direction)

        # Check if the new point is inside the map and not in an obstacle
        if (0 <= new_point[0] <= 11) and (0 <= new_point[1] <= 11) and map[new_point[0]][new_point[1]]:
            path.append(new_point)

            # Check if the new point is close to the goal
            if np.linalg.norm(np.array(new_point) - np.array(goal)) < max_step:
                path.append(goal)
                break

    return path

# Define a map with obstacles (1 represents obstacles, 0 represents free space)
map = np.ones((12, 12), dtype=bool)
map[1:4, 3:9] = 0
map[5:9, 2:5] = 0

# Define the goal
goal = (10, 10)

# Find the path
path = RRT(map, goal)

# Visualize the path
visualize_path(map, path)


# Finished successfully


