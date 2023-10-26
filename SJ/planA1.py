import time
from time import sleep
from pprint import *
import sys
import random
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library
import python.selfLocalizeRightA1 as SL

np.set_printoptions(threshold=sys.maxsize)

# Create a robot object and initialize
onRobot = True # Whether or not we are running on the Arlo robot
needNewLocation = False
showGUI = True  # Whether or not to open GUI windows
# Driving parameters
velocity = 0.0 # cm/sec
angular_velocity = 0.0 # radians/sec
sigma_d = 25 # cm
sigma_theta = 0.2 # radians

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


def turnRobo(angle):
    # convert angle to degrees:
    angle = angle * 180 / np.pi
    print("Angle: ", angle)
    # turn the robot accordingly
    if angle > 0:
        turnLeft(angle)
    elif angle < 0:
        turnRight(-angle)
    else:
        print("No turn needed")


def angleCalc(tvec):
    beta = np.arccos(np.dot( (tvec/np.linalg.norm(tvec)) , (0,0,1)))
    print("beta", beta)
    return beta[0][0]



def driveWithTime(distance):
    shortdist = distance - 25
    timeDrive = shortdist / 16.75
    succeded = True
    print(time.time())
    print("driveWithTime: time",timeDrive)
    print("driveWithTime: distance",distance)
    start_time = time.time()
    end_time = start_time + timeDrive
    left_speed = 31
    right_speed = 37.5
    while time.time() < end_time:
        frontSensor = arlo.read_front_ping_sensor()
        rightSensor = arlo.read_right_ping_sensor()
        leftSensor = arlo.read_left_ping_sensor()   
        if frontSensor < 250 or rightSensor < 200 or leftSensor < 200:
            succeded = False
            print(arlo.stop())
            sleep(0.2)
            end_time = time.time() + 5
            print("driveWithTime: turned")
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
            print(arlo.go_diff(left_speed, right_speed, 1, 1))
    return succeded
            



def turnLeft(degree):
    sleep(0.041)
    print(arlo.go_diff(64, 68, 0, 1))

    sleep(0.0074 * degree + ((degree**2)*0.000001))
    # send a stop command
    print(arlo.stop())
        
    # Wait a bit before next command
    sleep(0.2)

def turnRight(degree):
    sleep(0.041)
    print(arlo.go_diff(64, 70, 1, 0))

    sleep(0.0074 * degree + ((degree**2)*0.000001))
    # send a stop command
    print(arlo.stop())
        
    # Wait a bit before next command
    sleep(0.5)


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


def updateParticle(particles):
    for part in particles: 
        part.setX(part.getX() + velocity*np.cos(part.getTheta()))
        part.setY(part.getY() + velocity*np.sin(part.getTheta()))
        part.setTheta(part.getTheta() + angular_velocity)




def selfLocalize(particle, world, WIN_RF1, WIN_World): 
    # Fetch next frame
    colour = cam.get_next_frame()

    num_particles = len(particles)
    
    # Detect objects
    objectIDs, dists, angles = cam.detect_aruco_objects(colour)
    
    particle.add_uncertainty(particles,3.5, 0.1)
    if not isinstance(objectIDs, type(None)): #If the robot can see a landmark then the following
        # List detected objects
        imp_landmarks = [] #landmark id
        imp_landmarks_index = [] #landmark index in a list
        
        for i in range(len(objectIDs)):
            print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
            if objectIDs[i] in landmarkIDs and not objectIDs[i] in imp_landmarks: #makes sure it only adds a landmark on
                imp_landmarks_index.append(i)
                imp_landmarks.append(objectIDs[i])

        Xtbar = []
        for part in particles: 
            weightDist = 1
            weightAngle = 1
            for indx in imp_landmarks_index: 
                weightDist = weightDist * SL.p_dist_M(dists[indx],landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
                weightAngle = weightAngle * SL.p_meas_M(angles[indx],landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
            part.setWeight(weightAngle*weightDist)
            Xtbar.append(weightDist*weightAngle)

        # Normalize
        Xtbar_norm = []
        sum_Xtbar = sum(Xtbar)
        #print(sum_Xtbar)
        for i in range(len(Xtbar)):
            if sum_Xtbar == 0:
                break
            Xtbar_norm.append(Xtbar[i]/sum_Xtbar)

        # Resampling
        new_particles = np.random.choice(particles, size=len(particles), replace=True, p=Xtbar_norm)
        
        new2_part = []
        for part in new_particles: 
            new2_part.append(particle.Particle(part.getX(),part.getY(),part.getTheta(),part.getWeight()))
        
        #remove 5% of the particles and add new random particles
        
        num_elements_to_update = int(len(particles) * 0.05)
        indices_to_update = random.sample(range(len(particles)), num_elements_to_update)
        
        # Create new random particles
        new_rand_particles = [particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles) for _ in range(num_elements_to_update)]
        
        for i, index in enumerate(indices_to_update):
            new2_part[index] = new_rand_particles[i]
        
        particles = new2_part

        

        # Draw detected objects
        cam.draw_aruco_objects(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    
    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    if showGUI:
        # Draw map
        SL.draw_world(est_pose, particles, world)

        # Show 10
        cv2.imshow(WIN_RF1, colour)

        # Show world
        cv2.imshow(WIN_World, world)


def turnDetectLandmark(landmarkID):
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
            turnLeft(20)
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

def turnDetectObstacle():
    counter = 0
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        # print go diff 
        detected, distance, id = searchAndShowObstacle()
        if counter == 21:
            print(arlo.stop())
            return detected, 0.0, 0
        if not detected: 
            turnLeft(20)
            sleep(0.9)
            counter += 1
            print("tdObstacle: This is the counter: ", counter)
        else: 
            print(arlo.stop())
            return detected, (distance / 14.086079), id

def reposition(visitedObstacles):
    detected, distance, id = turnDetectObstacle()
    print("reposition: Detected in reposition: ", id)
    print("reposition: Visited obstacles: ", visitedObstacles)
    if detected and id not in visitedObstacles and id != 0:
        print("reposition: driving at ",id )
        #TURN TO OBSTACLE
        visitedObstacles.append(id)
        driveWithTime(distance/2)
    else: reposition(visitedObstacles)
    return visitedObstacles      


def main():
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)
    
    # Initialize particles
    num_particles = 1000
    particles = SL.initialize_particles(num_particles)

    est_pose = SL.particle.estimate_pose(particles) # The estimate of the robots current pose

    # Initialize the robot (XXX: You do this)

    # Allocate space for world map
    world = np.zeros((500,500,3), dtype=np.uint8)

    # Draw map
    SL.draw_world(est_pose, particles, world)

    for landmark in landmarkIDs:
        visitedObstacles = []
        landmarkReached = False
        numtries = 0
        while not landmarkReached:
            detected, distance, tvecs = turnDetectLandmark(landmark)
            print(landmark)
            if detected:
                print("Main: Found the landmark: " ,landmark)
                # Find the distance of the landmark
                print("Main: Distance: ", distance)
                # Drive to the landmark
                # Turn to landmark
                # SENSORES
                turnRobo(angleCalc(tvecs)*0.8)
                print("tvecs", tvecs)
                
                if distance < 150:
                    if driveWithTime(distance):
                        landmarkReached = True
                else: 
                    driveWithTime(70)
                    turnRight(30)

                # Self localize and create a path to the landmark
            else: 
                print("Main: didn't find the landmark")
                visitedObstacles = reposition(visitedObstacles)
                numtries += 1
                if numtries > 3: 
                    turnRight(90)
                    driveWithTime(4)
                    numtries = 0
                    visitedObstacles = []
                print("Main: Visited obstacles: ", visitedObstacles)
    print("Succesfully completed the course! Time: 00:15 minutes")
        
main()


























""" 

# Main program #
#try:


    
     print("Opening and initializing camera")
    if camera.isRunningOnArlo():
        cam = camera.Camera(0, 'arlo', useCaptureThread = True)
    else:
        cam = camera.Camera(0, 'macbookpro', useCaptureThread = True)
    
while True:

        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        if not isRunningOnArlo():
            if action == ord('w'): # Forward
                velocity += 4.0
            elif action == ord('x'): # Backwards
                velocity -= 4.0
            elif action == ord('s'): # Stop
                velocity = 0.0
                angular_velocity = 0.0
            elif action == ord('a'): # Left
                angular_velocity += 0.2
            elif action == ord('d'): # Right
                angular_velocity -= 0.2
        
        # Use motor controls to update particles
        # XXX: Make the robot drive
        
            
        
        
        

  
finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread() 
 """