import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon

import sph

import os
from datetime import datetime

import numpy as np
#Simulation graphical setup
simulationsteps = 1200
particleMarkerSize = 2 #size of particle in axes units
plotSize = 50.0
ratio = (3,4) #ratio of plot size
axesScaling = 10 #size of axes scaling factor e.g. 10 units/axesScaling = plot cm size
floodRisingFrameStart = 10
plotFloor = 0.0
plotFloorSpeed = 0.2

#Simulation physics setup
obstacleList = [[[75.98, 31.76], [57.23, 49.03], [110.78, 85.56], [114.23, 64.58]],[[70,35], [70,80], [120,80], [120,30]]]
numParticles = 1
floodRising = True
gravityOn = False

pressureMultiplier = 500000
targetDensity = 0.002
smoothingRadius = 40.0
collisionDamping = 0.8
mass = 1.0
gravity = 10.0
deltaTime = 0.02
velDamp = 1.0
bodyforce = (0,-20.0) #only when gravity is turned off
viscosityStrength = 0

particleList = [(80,40)]
velocity = (0,4)
rectangles = []

SPHObject = sph.SPH(particleList=particleList,obstacleList=obstacleList,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=False,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
                    pressureMultiplier=pressureMultiplier,targetDensity=targetDensity,\
                    smoothingRadius=smoothingRadius,collisionDamping=collisionDamping,mass=mass,gravity=gravity,deltaTime=deltaTime,\
                    velDamp=velDamp,bodyforce=bodyforce,plotFloorSpeed=plotFloorSpeed,viscosityStrength=viscosityStrength)


### ANIMATION CODE ##############################################################
#Simulation graphical size calculations
x_limits = (0, ratio[0]*plotSize)
y_limits = (0, ratio[1]*plotSize)
figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
points_per_unit = inches_per_unit * 72  # convert to points
markersize = particleMarkerSize * points_per_unit  # 5 axes unit marker diameter

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
ax.set_xlim(x_limits)
ax.set_ylim(y_limits)
ax.set_aspect('equal')
plt.xticks(np.arange(x_limits[0], x_limits[1], 5.0))
plt.title(f"SPH Simulation\nParticles: {numParticles}, Target Density: {targetDensity:.2e}")
ax.set_position([0.1, 0.1, 0.8, 0.8])

for obstacle in obstacleList:
    polygon = Polygon(obstacle, facecolor='gray', alpha=0.5)
    ax.add_patch(polygon)

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([SPHObject.particleList[i][0]], [SPHObject.particleList[i][1]], 'b^', markersize=markersize)[0])
floorLine = ax.axhline(y=SPHObject.plotFloor, color='red', linestyle='-')
collisionIdxs = SPHObject.detectCollision(particleList[0])

if collisionIdxs:
    collisionEdges = SPHObject.findCollisionEdge(collisionIdxs,0)
    for j in collisionEdges:
        collisionIdx = j[0]
        collisionEdge = j[1]
        collisionEdgeNormal = SPHObject.obstacleEdgeNormals[collisionIdx][collisionEdge]
        collisionEdgeParallel = (-collisionEdgeNormal[1],collisionEdgeNormal[0])
        v_normal = (velocity[0] * collisionEdgeNormal[0] + velocity[1] * collisionEdgeNormal[1])
        v_parallel = (velocity[0] * collisionEdgeParallel[0] + velocity[1] * collisionEdgeParallel[1])
        # Update velocity based on collision response
        if gravityOn:
            # Reflect normal component with damping, preserve parallel component
            new_v_normal = -v_normal * 0.5
            new_velocity = (new_v_normal * collisionEdgeNormal[0] + v_parallel * collisionEdgeParallel[0],
                                        new_v_normal * collisionEdgeNormal[1] + v_parallel * collisionEdgeParallel[1])
        else:
            # Zero out normal component, preserve parallel component
            new_velocity = (v_parallel * collisionEdgeParallel[0],
                                        v_parallel * collisionEdgeParallel[1])
        # Plot normal vector
        normal_scale = 20  # Scale factor for the normal vector
        ax.quiver(SPHObject.particleList[0][0], SPHObject.particleList[0][1], 
                    collisionEdgeNormal[0], collisionEdgeNormal[1], 
                    angles='xy', scale_units='xy', scale=1/normal_scale,
                    color='g', label='Normal Vector')
        
        ax.quiver(SPHObject.particleList[0][0], SPHObject.particleList[0][1], 
                    collisionEdgeParallel[0], collisionEdgeParallel[1], 
                    angles='xy', scale_units='xy', scale=1/normal_scale,
                    color='k', label='Parallel Vector')
        
        ax.quiver(SPHObject.particleList[0][0], SPHObject.particleList[0][1], 
                    new_velocity[0], new_velocity[1],
                    angles='xy', scale_units='xy', scale=1/normal_scale,
                    color='r', label='new vel Vector')
                    
        print(f'ori vel was {velocity} new vel is {new_velocity}')
        nextIdx = 0 if collisionEdge == len(obstacleList[collisionIdx]) - 1 else collisionEdge + 1
        x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][nextIdx][0]]
        y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][nextIdx][1]]
        # if collisionEdge == len(obstacleList[collisionIdx]) - 1:
        #     x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][0][0]]
        #     y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][0][1]]
        # else:
        #     x_values = [obstacleList[collisionIdx][collisionEdge][0], obstacleList[collisionIdx][collisionEdge+1][0]]
        #     y_values = [obstacleList[collisionIdx][collisionEdge][1], obstacleList[collisionIdx][collisionEdge+1][1]]
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
        normal = SPHObject.obstacleEdgeNormals[i][j]
        
        # Plot normal vector from edge midpoint
        ax.quiver(midpoint_x, midpoint_y,
                 normal[0], normal[1],
                 angles='xy', scale_units='xy', scale=1/normal_scale,
                 color='r', alpha=0.5)
ax.quiver(SPHObject.particleList[0][0], SPHObject.particleList[0][1], 
                velocity[0], velocity[1], 
                angles='xy', scale_units='xy', scale=1/normal_scale,
                color='y', label='ori vel Vector')

plt.title(f"SPH Simulation\nParticles: {numParticles}, Target Density: {targetDensity:.2e}")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()
