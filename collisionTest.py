import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math

obstacleList = [[(70,50),(100,50),(100,30),(70,30)],[(15,120),(65,120),(65,90),(15,90)],[(10,30),(30,50),(50,30),(30,10),(10,10)]]
point = (35,119.9)

def calculateEdgeNormals():
    res = []
    for obstacle in obstacleList:
        edgenormals = []
        edgenormals.append(((obstacle[1][0] - obstacle[0][0]),(obstacle[1][1] - obstacle[0][1])))
        edgenormals.append(((obstacle[2][0] - obstacle[1][0]),(obstacle[2][1] - obstacle[1][1])))
        edgenormals.append(((obstacle[3][0] - obstacle[2][0]),(obstacle[3][1] - obstacle[2][1])))
        edgenormals.append(((obstacle[0][0] - obstacle[3][0]),(obstacle[0][1] - obstacle[3][1])))
        res.append(edgenormals)
    return res

obstacleEdgeNormals = calculateEdgeNormals()

def detectCollision():
    for i in range(len(obstacleEdgeNormals)): #for each obstacle
        collision = True
        for j in range(len(obstacleEdgeNormals[i])): #for each obstacle normal
            pointdot = np.dot(point,obstacleEdgeNormals[i][j])
            #print(f'point:{point} edgenormal:{obstacleEdgeNormals[i][j]}')
            edgedot1 = np.dot(obstacleList[i][j],obstacleEdgeNormals[i][j])
            if j == len(obstacleEdgeNormals[i])-1:
                edgedot2 = np.dot(obstacleList[i][0],obstacleEdgeNormals[i][j])
            else:
                edgedot2 = np.dot(obstacleList[i][j+1],obstacleEdgeNormals[i][j])
            #print(f"pointdot is {pointdot} edgedot1 is {edgedot1} edgedot2 is {edgedot2}")
            # if pointdot[0]<edgedot2[0] and pointdot[0]>edgedot1[0]:
            #     print(f'collision true between {point} and {obstacleList[i]}')
            if not(pointdot<edgedot2 and pointdot>edgedot1):
                print(f'no collision between {point} and {obstacleList[i]}')
                collision = False
                break
        if collision:
            print(f'COLLISION between {point} and {obstacleList[i]}')
            return i
    return -1

def findCollisionEdge(i):
    min_distance = 999
    nearest_edge = -1
    collision_obstacle = i
    for j in range(len(obstacleList[i])):
        edge1 = obstacleList[i][j]
        if j == len(obstacleList[i])-1:
            edge2 = obstacleList[i][0]
        else:
            edge2 = obstacleList[i][j+1]
        print(edge1)
        print(edge2)
        parallelogram_area = abs((edge2[0]-edge1[0]) * (point[1]-edge1[1]) - (edge2[1]-edge1[1]) * (point[0]-edge1[0]))
        base = math.sqrt((edge1[0]-edge2[0])*(edge1[0]-edge2[0])+(edge1[1]-edge2[1])*(edge1[1]-edge2[1]))
        distance = parallelogram_area/base
        print(f'distance:{distance} edge:{j} between {edge1} and {edge2}')
        if distance < min_distance:
            min_distance = distance
            nearest_edge = j
    print(f'nearest edge is {nearest_edge}')
    return nearest_edge



fig, ax = plt.subplots()

# Plot obstacles as rectangles
for obstacle in obstacleList:
    polygon = Polygon(obstacle, facecolor='gray', alpha=0.5)
    ax.add_patch(polygon)

# Plot point as a particle
ax.plot(point[0], point[1], 'ro', markersize=10, label='Particle')
collisionIdx = detectCollision()
if collisionIdx !=-1:
    collisionEdge = findCollisionEdge(collisionIdx)

    if collisionEdge == len(obstacleList[collisionIdx]) - 1:
        x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][0][0]]
        y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][0][1]]
    else:
        x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][collisionEdge+1][0]]
        y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][collisionEdge+1][1]]
    ax.plot(x_values, y_values, 'bo', linestyle="--")

# Set plot properties
ax.set_xlim(0, 150)
ax.set_ylim(0, 150)
ax.set_aspect('equal')
ax.grid(True)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.legend()

plt.title('Obstacles and Particle Visualization')
plt.show()