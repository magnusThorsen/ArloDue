import cv2
import particle
import camera
import numpy as np
import time
from timeit import default_timer as timer
import sys


# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = False # Whether or not we are running on the Arlo robot


def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../")


try:
    import robot
    onRobot = True
    
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False





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
landmarkIDs = [1, 9]
landmarks = {
    1: (0.0, 0.0),  # Coordinates for landmark 1
    9: (300.0, 0.0)  # Coordinates for landmark 2
}
landmark_colors = [CRED, CGREEN] # Colors used when drawing the landmarks



def gaussian_likelihood(x, mean, sigma):
    # Compute the likelihood of x given a Gaussian distribution with mean and sigma
    exponent = -0.5 * ((x - mean) / sigma) ** 2
    return (1.0 / (sigma * np.sqrt(2 * np.pi))) * np.exp(exponent)

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
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        particles.append(p)

    return particles


# Main program #
try:
    if showGUI:
        # Open windows
        WIN_RF1 = "Robot view"
        cv2.namedWindow(WIN_RF1)
        cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)


    # Initialize particles
    num_particles = 1
    particles = initialize_particles(num_particles)

    est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec


    # initializing robot XXX
    if isRunningOnArlo():
        spaceRanger = robot.Robot() # Create a robot object


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
        def updateParticles():
            for i in particles:
                particle.move_particle()
        # XXX: You do this
        
       

        

        


        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):
            # List detected objects
            for i in range(len(objectIDs)):
                print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
                if objectIDs[i] == 9:
                    x0 = 300
                    y0 = 0
                    r2 = dists[i]**2
                    for part in particles:
                        # Magnus bud:jeg er mega dejlig 3====D
                        x = np.random.randint(300 - (dists[i]-1), 300 + (dists[i]-1))
                        y =  (y0 - np.sqrt(r2 - x**2 + 2*x* x0- x0**2)) * np.random.choice([-1,1])
                        part.setX(x)
                        part.setY(y)
                        
                        # calculate the angle between the vector ((x,y) - (x0,y0)) and (5,0) in radians
                        v1 = np.array([x0-x,y0-y])
                        v2 = np.array([1,1])
                        angle = np.arccos(np.dot(v1,v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))   
                        part.setTheta(angle)            

                        
                elif objectIDs[i] == 1:
                    x0 = 0
                    y0 = 0
                    r2 = dists[i]**2
                    for part in particles:
                        x = np.random.randint(-(dists[i]-1),(dists[i]-1))
                        y =  (y0 - np.sqrt(r2 - x**2 + 2*x* x0- x0**2)) * np.random.choice([-1,1])
                        part.setX(x)
                        part.setY(y)
                    
                
                # XXX: Do something for each detected object - remember, the same ID may appear several times
                
                
        

            # Compute particle weights
            # XXX: You do this
            
            # Define the standard deviations for your measurement noise
            """ distance_sigma = 1.0  # Adjust this based on your sensor noise
            angle_sigma = 0.1    # Adjust this based on your sensor noise
            
            for part in particles:
                expected_measurement = part.getExpectedMeasurements(landmarks)  # Compute expected measurement based on particle's pose
                actual_measurement = (objectIDs[0], dists[0], angles[0])  # The detected object's measurements

                # Calculate the likelihood of distance and angle using Gaussian distributions
                distance_likelihood = gaussian_likelihood(actual_measurement[1], expected_measurement[1], distance_sigma)
                angle_likelihood = gaussian_likelihood(actual_measurement[2], expected_measurement[2], angle_sigma)

                # Calculate the weight as the product of likelihood
                particle_weight = distance_likelihood * angle_likelihood

                # Update the particle's weight
                part.setWeight(particle_weight)

            # Normalize particle weights to form a probability distribution
            total_weight = sum(particle.getWeight() for particle in particles)
            for i in range(len(particles)):
                particle = particles[i]
                particle.setWeight(particle.getWeight() / total_weight) """

            # Resampling
            # XXX: You do this

            # Draw detected objects
            cam.draw_aruco_objects(colour)
        else:
            
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)

        #particle.add_uncertainty(particles,5, 0.1)
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose
        print("theta:", est_pose.getTheta())

        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # Show frame
            cv2.imshow(WIN_RF1, colour)

            # Show world
            cv2.imshow(WIN_World, world)
    
  
finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()

