import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import math

#obstacleList = [[(70,50),(100,50),(100,30),(70,30)],[(15,120),(65,120),(65,90),(15,90)],[(10,30),(30,50),(50,30),(30,10),(10,10)],[(100,80),(120,110),(130,100)]]
#obstacleList = [[(20,50),(100,110),(130,110),(50,50)]]
obstacleList = [[(90,80),(50,20),(90,130)],[(20,50),(100,110),(130,110),(50,50)]]
point = (65,50)

def calculateEdgeNormals():
    res = []
    for obstacle in obstacleList:
        edgenormals = []
        for vertices in range(len(obstacle)):
            if vertices != len(obstacle)-1:
                edgenormals.append((-(obstacle[vertices+1][1] - obstacle[vertices][1]),(obstacle[vertices+1][0] - obstacle[vertices][0])))
                #edgenormals.append(((obstacle[vertices+1][0] - obstacle[vertices][0]),(obstacle[vertices+1][1] - obstacle[vertices][1])))
            else:
                edgenormals.append((-(obstacle[0][1] - obstacle[vertices][1]),(obstacle[0][0] - obstacle[vertices][0])))
                #edgenormals.append(((obstacle[0][0] - obstacle[vertices][0]),(obstacle[0][1] - obstacle[vertices][1])))
        res.append(edgenormals)
    print(res)
    return res

obstacleEdgeNormals = calculateEdgeNormals()


def detectCollision():
    for i in range(len(obstacleEdgeNormals)): #for each obstacle
        collision = True
        print(f'checking {len(obstacleEdgeNormals[i])} normals')
        mindot = 999
        maxdot = -999
        for j in range(len(obstacleEdgeNormals[i])): #for each obstacle normal
            pointdot = np.dot(point,obstacleEdgeNormals[i][j])
            for k in range(len(obstacleList[i])):
                edgedot = np.dot(obstacleList[i][k],obstacleEdgeNormals[i][j])
                if maxdot<edgedot:
                    maxdot = edgedot
                if mindot > edgedot:
                    mindot = edgedot
            if not(pointdot<maxdot and pointdot>mindot):
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
    collisionEdgeNormal = obstacleEdgeNormals[collisionIdx][collisionEdge]
    # Plot normal vector
    normal_scale = 20  # Scale factor for the normal vector
    ax.quiver(point[0], point[1], 
                collisionEdgeNormal[0], collisionEdgeNormal[1], 
                angles='xy', scale_units='xy', scale=1/normal_scale,
                color='g', label='Parallel Vector')
    
    if collisionEdge == len(obstacleList[collisionIdx]) - 1:
        x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][0][0]]
        y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][0][1]]
    else:
        x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][collisionEdge+1][0]]
        y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][collisionEdge+1][1]]
    ax.plot(x_values, y_values, 'bo', linestyle="--")

# Plot all edge normals
normal_scale = 20  # Scale factor for the normal vectors
for i, obstacle in enumerate(obstacleList):
    for j in range(len(obstacle)):
        # Get edge points
        edge1 = obstacle[j]
        edge2 = obstacle[0] if j == len(obstacle)-1 else obstacle[j+1]
        
        # Calculate midpoint of edge for normal vector origin
        midpoint_x = (edge1[0] + edge2[0]) / 2
        midpoint_y = (edge1[1] + edge2[1]) / 2
        
        # Get normal vector for this edge
        normal = obstacleEdgeNormals[i][j]
        
        # Plot normal vector from edge midpoint
        ax.quiver(midpoint_x, midpoint_y,
                 normal[0], normal[1],
                 angles='xy', scale_units='xy', scale=1/normal_scale,
                 color='r', alpha=0.5)

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