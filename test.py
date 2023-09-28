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
    longest_path = (None,0)
    numtries = 0
    current_coordinates = reset_coordinates()
    path = [current_coordinates]
    visited = set()
    visited.add(current_coordinates)
    tries = 0
    
    while current_coordinates != goal :
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
            if len(path) > longest_path[1]:
                longest_path = (path, len(path))
            if numtries % 1000 == 0:
                print(numtries)
                return longest_path[0], numtries
    
    return path, numtries


# create a 70 * 40 np bool array and set three blobs of false and the rest true: 
# 1. a 20*20 square in the middle
# 2. a 10*10 square in the bottom left corner
# 3. a 10*10 square in the top right corner
map1 = np.ones((70, 40), dtype=bool)
""" map1[25:45, 10:30] = False
map1[60:70, 30:40] = False """
map1[5:8, 5:10] = False

#use matplotlib to show the map and the path 
import matplotlib.pyplot as plt
path, numtries = RRT2(map1, (20,10))
print(numtries)
path = np.array(path)
plt.imshow(map1.T, cmap='Greys', origin='lower')
plt.plot(path[:,0], path[:,1], 'r-')
plt.show()



#print("the right path", RRT2(map1, (65,10)))