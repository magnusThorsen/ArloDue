import sys
import cv2
import cv2.aruco as aruco
import particle as particle
import random_numbers as rn
import camera as camera
import numpy as np
import random
import time
from time import sleep
from timeit import default_timer as timer
import math


# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True # Whether or not we are running on the Arlo robot


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../Arlo/Robot")


try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False

arlo = robot.Robot()

# Driving parameters
velocity = 0.0 # cm/sec
angular_velocity = 0.0 # radians/sec


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
landmarkIDs = [1, 7]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    7: (100.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks





def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 100
    offsetY = 250

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)



def initialize_particles(num_particles):
    particles = []
    for p in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles

def turnParticle(degree):
    angularVelocity = (degree*(np.pi/180))/(0.0074 * degree + ((degree**2)*0.000001))
    return angularVelocity

def moveParticleForward(distance):
    shortdist = (distance - 25)
    timeDrive = shortdist / 16.75
    velocity = distance/timeDrive
    return velocity

def driveWithTime(distance, particles):
    global velocity
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
                turnLeft(90, particles)
                sleep(0.2)
            elif leftSensor < 300:
                turnRight(90, particles)
                sleep(0.2)
            elif frontSensor < 250:
                if leftSensor <= rightSensor:
                    turnRight(90, particles)
                    sleep(0.2)
                else:
                    turnLeft(90,particles)
                    sleep(0.2)
        else:
            velocity = moveParticleForward(distance)
            updateParticles(particles)
            print(arlo.go_diff(left_speed, right_speed, 1, 1))
    print(arlo.stop())
    velocity = 0.0
    return succeded

def turnLeft(degree, particles):
    global angular_velocity
    turnTime = time.time()
    endtime = turnTime + (0.0074 * degree + ((degree**2)*0.000001))
    while time.time() < endtime:        
        print(arlo.go_diff(64, 68, 0, 1))
    angular_velocity = turnParticle(degree)
    print(angular_velocity)
    updateParticles(particles)
    # send a stop command
    print(arlo.stop())
    angular_velocity = 0.0
            
    # Wait a bit before next command
    sleep(0.2)


def turnRight(degree, particles):
    global angular_velocity
    turnTime = time.time()
    endtime = turnTime + (0.0074 * degree + ((degree**2)*0.000001))
    while time.time() < endtime:
        print(arlo.go_diff(64, 70, 1, 0))
    angular_velocity = angular_velocity - turnParticle(degree)
    updateParticles(particles)
    # send a stop command
    print(arlo.stop())
    angular_velocity = 0.0
        
    # Wait a bit before next command
    sleep(0.2)

sigma_d = 25 # cm
sigma_theta = 0.2 # radians

# XXX: You do this
# calc d^(i) from equation 2
def dist_part_landm(lx, ly, x, y):
    d = np.sqrt(((lx - x)**2) + ((ly - y)**2))
    return d

def e_l (lx, ly, x, y):
    result =  np.array([(lx - x) /dist_part_landm(lx,ly,x,y), (ly - y) /dist_part_landm(lx,ly,x,y)])
    return result


# equation 2
def p_dist_M (dm,lx,ly,part):
    result = (1/(np.sqrt(2*(np.pi)*(sigma_d**2))))*math.exp(-(((dm-(dist_part_landm(lx,ly,part.getX(),part.getY())))**2)/(2*sigma_d**2)))
    return result 

# Equation 4
def phi_i (lx, ly, part):
    x = part.getX()
    y = part.getY()
    theta = part.getTheta()
    e_theta_i = (np.array([np.cos(theta),np.sin(theta)])) 
    e_theta_i_hat = np.array([-np.sin(theta),np.cos(theta)])
    result = np.sign(np.dot(e_theta_i_hat,e_l(lx, ly, x, y)))*np.arccos(np.dot(e_theta_i,e_l(lx, ly, x, y)))
    return result 
    
# equation 3 
def p_meas_M (tm,lx,ly, part):
    result = (1/(np.sqrt(2*(np.pi)*(sigma_theta**2))))*math.exp(-(((tm-(phi_i(lx,ly,part)))**2)/(2*sigma_theta**2)))
    return result 

# Equation 5
def prob_product(landmarks):
    result = 1
    for landmark in landmarks:
        result = dist_part_landm(landmark) * result
    return result 

def updateParticles(particles):
    # Use motor controls to update particles
    # XXX: Make the robot drive
    for part in particles: 
        part.setX(part.getX() + velocity*np.cos(part.getTheta()))
        part.setY(part.getY() + velocity*np.sin(part.getTheta()))
        part.setTheta(part.getTheta() + angular_velocity) 

def selfLocalize():
    # Main program #
    try:
        velocity = 0.0 # cm/sec
        angular_velocity = 0.0 # radians/sec
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
        particles = initialize_particles(num_particles)

        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

        

        # Initialize the robot (XXX: You do this)

        # Allocate space for world map
        world = np.zeros((500,500,3), dtype=np.uint8)

        # Draw map
        draw_world(est_pose, particles, world)

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
        
            if isRunningOnArlo():
                if action == ord('w'): # Forward
                    print("started driveWithTime")
                    driveWithTime(70, particles)
                    print("ended driveWithTime")
                elif action == ord('x'): # Backwards
                    velocity -= 0.0
                elif action == ord('s'): # Stop
                    print(arlo.stop())
                    velocity = 0.0
                    angular_velocity = 0.0
                elif action == ord('a'): # Left
                    turnLeft(90, particles)
                elif action == ord('d'): # Right
                    turnRight(90, particles)
                
            updateParticles(particles)

            # Fetch next frame
            colour = cam.get_next_frame()
            
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
                        weightDist = weightDist * p_dist_M(dists[indx],landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
                        weightAngle = weightAngle * p_meas_M(angles[indx],landmarks[objectIDs[indx]][0],landmarks[objectIDs[indx]][1],part)
                    part.setWeight(weightAngle*weightDist)
                    Xtbar.append(weightDist*weightAngle)

                # Normalize
                Xtbar_norm = []
                sum_Xtbar = sum(Xtbar)
                #print(sum_Xtbar)
                for i in range(len(Xtbar)):
                    if sum_Xtbar == 0:
                        # set weights to uniform distribution
                        for i in range(len(Xtbar)):
                            Xtbar_norm.append(1.0/num_particles)
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
                draw_world(est_pose, particles, world)
        
                # Show 10
                cv2.imshow(WIN_RF1, colour)

                # Show world
                cv2.imshow(WIN_World, world)
        
    
    finally: 
        # Make sure to clean up even if an exception occurred
        
        # Close all windows
        cv2.destroyAllWindows()

        # Clean-up capture thread
        cam.terminateCaptureThread()

selfLocalize()