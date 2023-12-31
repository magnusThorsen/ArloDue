import time
from time import sleep
from pprint import *
import sys
import random
import cv2 # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np # Import Numpy library
import selfLocalizeRightA1 as SL
import camera
import particle

np.set_printoptions(threshold=sys.maxsize)

# Create a robot object and initialize
onRobot = True # Whether or not we are running on the Arlo robot
needNewLocation = False
showGUI = True  # Whether or not to open GUI windows
# Driving parameters
velocity = 0.0 # cm/sec
angular_velocity = 0.0 # radians/sec


WIN_RF1 = "Robot view"
WIN_World = "World view"



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
landmarkIDs = [1, 2, 3, 4, 1] 
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    2: (0.0, 300.0),  # Coordinates for landmark 2
    3: (400.0, 0.0),  # Coordinates for landmark 3
    4: (400.0, 300.0)  # Coordinates for landmark 4
}
landmark_colors = [CRED, CGREEN, CBLUE, CYELLOW] # Colors used when drawing the landmarks


def turnRobo(angle, particles):
    # convert angle to degrees:
    angle = angle * 180 / np.pi
    print("Angle: ", angle)
    # turn the robot accordingly
    if angle > 0:
        particles = turnLeft(angle,particles)
    elif angle < 0:
        particles = turnRight(-angle, particles)
    else:
        print("No turn needed")
    return particles


def angleCalc(tvec):
    beta = np.arccos(np.dot( (tvec/np.linalg.norm(tvec)) , (0,0,1)))
    print("beta", beta)
    return beta[0][0]



def updateParticles(particles):
    # Use motor controls to update particles
    # XXX: Make the robot drive
    for part in particles: 
        part.setX(part.getX() + velocity*np.cos(part.getTheta()))
        part.setY(part.getY() + velocity*np.sin(part.getTheta()))
        part.setTheta(part.getTheta() + angular_velocity) 
    return particles

            
def driveWithTime(distance, particles):
    global velocity
    shortdist = distance - 25
    timeDrive = shortdist / 16.75
    succeded = True
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
            if time.time() - end_time > 1.5: # Hvis robotten stadig er langt væk fra målet tidsmæssigt, sætter vi succ til false
                succeded = False
            print(arlo.stop())
            sleep(0.2)
            end_time = time.time() + 5
            print("driveWithTime: turned")
            if rightSensor < 300:
                particles = turnLeft(90,particles)
                sleep(0.2)
            elif leftSensor < 300:
                particles = turnRight(90, particles)
                sleep(0.2)
            elif frontSensor < 250:
                if leftSensor <= rightSensor:
                    particles = turnRight(90, particles)
                    sleep(0.2)
                else:
                    particles = turnLeft(90,particles)
                    sleep(0.2)
        else:
            print(arlo.go_diff(left_speed, right_speed, 1, 1))

    velocity = distance
    particles = updateParticles(particles)
    print(arlo.stop())
    velocity = 0.0
    return succeded, particles


def turnLeft(degree, particles):
    global angular_velocity
    turnTime = time.time()
    endtime = turnTime + (0.0074 * degree + ((degree**2)*0.000001))
    while time.time() < endtime:        
        print(arlo.go_diff(64, 68, 0, 1))
    angular_velocity = degree*(np.pi/180)
    print(angular_velocity)
    particles = updateParticles(particles)
    # send a stop command
    print(arlo.stop())
    angular_velocity = 0.0
            
    # Wait a bit before next command
    sleep(0.2)
    return particles


def turnRight(degree, particles):
    global angular_velocity
    turnTime = time.time()
    endtime = turnTime + (0.0074 * degree + ((degree**2)*0.000001))
    while time.time() < endtime:
        print(arlo.go_diff(64, 70, 1, 0))
    angular_velocity = - degree*(np.pi/180)
    particles = updateParticles(particles)
    # send a stop command
    print(arlo.stop())
    angular_velocity = 0.0
        
    # Wait a bit before next command
    sleep(0.2)
    return particles



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
                return detected, distance, translation_vector, len(corners)
    #Display the image with detected markers
    #cv2.imshow("sasLandmark: Detected Markers", image)
    return detected, 0.0, None, len(corners)


def updateParticle(particles):
    for part in particles: 
        part.setX(part.getX() + velocity*np.cos(part.getTheta()))
        part.setY(part.getY() + velocity*np.sin(part.getTheta()))
        part.setTheta(part.getTheta() + angular_velocity)


def detect_aruco_objects(img):
    # Define the dictionary
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    cameraMatrix = np.array([[focal, 0, xSize/2],
                            [0, focal, ySize/2],
                            [0, 0, 1]])

    # Detect markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(img, dictionary)
    if len(corners) == 0:
        return None, None, None
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 200, cameraMatrix, None)
    dists = np.linalg.norm(tvecs, axis=len(tvecs.shape) - 1) * 100
    dists = dists.reshape((dists.shape[0],))


    angles = np.zeros(dists.shape, dtype=dists.dtype)
    for i in range(dists.shape[0]):
        tobj = tvecs[i] * 100 / dists[i]
        zaxis = np.zeros(tobj.shape, dtype=tobj.dtype)
        zaxis[0,-1] = 1
        xaxis = np.zeros(tobj.shape, dtype=tobj.dtype)
        xaxis[0,0] = 1

        # We want the horizontal angle so project tobjt onto the x-z plane
        tobj_xz = tobj
        tobj_xz[0,1] = 0
        # Should the sign be clockwise or counter-clockwise (left or right)?
        # In this version it is positive to the left as seen from the camera.
        direction = -1*np.sign(tobj_xz[0,0])  # The same as np.sign(np.dot(tobj, xaxis.T))
        angles[i] = direction * np.arccos(np.dot(tobj_xz, zaxis.T))
    # convert ids, dists and angles to lists 
    ids = ids.flatten().tolist()
    dists = dists.tolist()
    angles = angles.tolist()

    return ids, dists, angles



def selfLocalize(particles, world, WIN_RF1, WIN_World): 
    # Fetch next frame
    img = cam.capture_array("main")

    num_particles = len(particles)
    
    # Detect objects
    objectIDs, dists, angles = detect_aruco_objects(img)
    
    particle.add_uncertainty(particles,10, 0.2)
    if not isinstance(objectIDs, type(None)): #If the robot can see a landmark then the following
        print("\n\n\n")
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

                DW = SL.p_dist_M(dists[indx]/1408.6079,landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
                if DW == 0: # vægtene blev for små og crashede derfor
                    DW = (1/len(particles))
                weightDist = weightDist * DW
                weightAngle = weightAngle * SL.p_meas_M(angles[indx],landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
            part.setWeight(weightAngle*weightDist)
            Xtbar.append(weightDist*weightAngle)

        # Normalize
        Xtbar_norm = []
        sum_Xtbar = sum(Xtbar)
        #print(sum_Xtbar)
        for i in range(len(Xtbar)):
            if sum_Xtbar == 0:
                print("sum_Xtbar = 0")
                # fill with uniform distribution
                for i in range(len(Xtbar)):
                    Xtbar_norm.append(1/len(Xtbar))
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
        #cam.draw_aruco_objects(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    
    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
    print("X:",est_pose.getX()," Y:", est_pose.getY()," Theta:", est_pose.getTheta())

    if showGUI:
        # Draw map
        SL.draw_world(est_pose, particles, world)

        # Show 10
        #cv2.imshow(WIN_RF1, colour)

        # Show world
        cv2.imshow(WIN_World, world)
    return particles, est_pose


def turnDetectLandmark(landmarkID, particles, world, WIN_RF1, WIN_World):
    counter = 0
    detCounter = 0 
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        particles, _ = selfLocalize(particles, world, WIN_RF1, WIN_World)
        # print go diff 
        print("tdLandmark: Finding landmark: ", landmarkID)
        detected, distance, t_vec, num_landmark = searchAndShowLandmark(landmarkID)
        detCounter += num_landmark
        if counter == 21:
            print(arlo.stop())
            landmarkFound = False
            return particles, landmarkFound, 0.0, None, num_landmark
        if not detected: 
            particles = turnLeft(20, particles)
            sleep(0.9)
            counter += 1
            print("tdLandmark: This is the counter: ", counter)
        else: 
            print(arlo.stop())
            landmarkFound = True
            return particles, landmarkFound, (distance / 14.086079), t_vec, detCounter

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

def turnDetectObstacle(particles, world, WIN_RF1, WIN_World):
    counter = 0
    while cv2.waitKey(4) == -1: # Wait for a key pressed event
        # print go diff 
        particles, _ = selfLocalize(particles, world, WIN_RF1, WIN_World)
        detected, distance, id = searchAndShowObstacle()
        if counter == 21:
            print(arlo.stop())
            return particles, detected, 0.0, 0
        if not detected: 
            particles = turnLeft(20, particles)
            sleep(0.9)
            counter += 1
            print("tdObstacle: This is the counter: ", counter)
        else: 
            print(arlo.stop())
            return particles, detected, (distance / 14.086079), id

def reposition_clever(particles, est_pose, landmark):
    print("BEING CLEVER!!")
    vecfromRobotolandmark = np.array([landmarks[landmark][0] - est_pose.getX(), landmarks[landmark][1] - est_pose.getY()])
    robovec = np.array([np.cos(est_pose.getTheta()), np.sin(est_pose.getTheta())])
    #angle between robot and landmark in radians 
    angleToRotate = np.arccos(np.dot(vecfromRobotolandmark, robovec)/(np.linalg.norm(vecfromRobotolandmark)*np.linalg.norm(robovec)))
    print("angle", angleToRotate)
    distance = np.linalg.norm(vecfromRobotolandmark)
    particles = turnRobo(angleToRotate, particles)
    if distance < 150:
        succ, particles = driveWithTime(distance, particles)
    else: 
        succ , particles = driveWithTime(70, particles)
        succ = False
        particles = turnRight(40, particles)
    
    return succ, particles 

def reposition_dum(visitedObstacles, particles, world, WIN_RF1, WIN_World, numtries = 0):
    print("BEING DUMB!!")
    particles, detected, distance, id = turnDetectObstacle(particles, world, WIN_RF1, WIN_World)
    print("reposition: Detected in reposition: ", id)
    print("reposition: Visited obstacles: ", visitedObstacles)
    if detected and id not in visitedObstacles and id != 0:
        print("reposition: driving at ",id )
        #TURN TO OBSTACLE
        visitedObstacles.append(id)
        _, particles = driveWithTime(distance/2, particles)
    elif numtries > 2: reposition_dum(visitedObstacles, particles, world, WIN_RF1, WIN_World, numtries + 1)
    return particles, visitedObstacles



def main():
    if showGUI:
        # Open windows
        #cv2.namedWindow(WIN_RF1)
        #cv2.moveWindow(WIN_RF1, 50, 50)

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
        Begin_time = time.time()
        visitedObstacles = []
        landmarkReached = False
        numtries = 0
        while not landmarkReached:
            particles, detected, distance, tvecs, num_landmark = turnDetectLandmark(landmark, particles, world, WIN_RF1, WIN_World)
            print(landmark)
            print("Num_landmarks ", num_landmark)
            if detected:
                print("Main: Found the landmark: " ,landmark)
                # Find the distance of the landmark
                print("Main: Distance: ", distance)
                # Drive to the landmark
                # Turn to landmark
                # SENSORES
                particles = turnRobo(angleCalc(tvecs)*0.8, particles)
                print("tvecs", tvecs)
                
                if distance < 150:
                    succ, particles = driveWithTime(distance, particles)
                    if succ: 
                        landmarkReached = True
                else: 
                    _ , particles = driveWithTime(70, particles)
                    particles = turnRight(40, particles)

                # Self localize and create a path to the landmark
            elif num_landmark < 1 :
            
                print("Main: didn't find the landmark")
                particles, visitedObstacles = reposition_dum(visitedObstacles, particles, world, WIN_RF1, WIN_World)
                numtries += 1
                if numtries > 1: 
                    particles = turnRight(90, particles)
                    _ , particles = driveWithTime(4, particles)
                    numtries = 0
                    visitedObstacles = []
                print("Main: Visited obstacles: ", visitedObstacles)

            else:
                print("Main: didn't find the landmark")
                # do self localize
                particles, est_pose = selfLocalize(particles, world, WIN_RF1, WIN_World)
                succ, particles = reposition_clever(particles, est_pose, landmark)
                if succ: 
                    landmarkReached = True
                

    print("Succesfully completed the course! Time ca. :", int(time.time() - Begin_time)*6 ,"seconds")
        
main()











