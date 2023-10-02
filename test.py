import random
import numpy as np


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
    
    while numtries < 1000:
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

#use matplotlib to show the map and the path 
import matplotlib.pyplot as plt
path = RRT2(map1, (10,10))
print(path, len(path))
path = np.array(path)
plt.imshow(map1.T, cmap='Greys', origin='lower')
plt.plot(path[:,0], path[:,1], 'r-')
plt.show()



#print("the right path", RRT2(map1, (65,10)))