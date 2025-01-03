import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import sph

import numpy as np
#Simulation graphical setup
simulationsteps = 1000
particleMarkerSize = 2 #size of particle in axes units
plotSize = 50.0
ratio = (2,4) #ratio of plot size
axesScaling = 10 #size of axes scaling factor e.g. 10 units/axesScaling = plot cm size
floodRisingFrameStart = 50
plotFloor = 0.0
plotFloorSpeed = 0.1

#Simulation physics setup
obstacleList = [[(70,50),(100,50),(100,30),(70,30)],[(15,120),(65,120),(65,90),(15,90)]]
numParticles = 49
floodRising = True
gravityOn = True

#pressureMultiplier = 300
#pressureMultiplier = 50
pressureMultiplier = 200
targetDensity = 0.002
smoothingRadius = 20.0
collisionDamping = 0.8
mass = 1.0
gravity = 10.0
deltaTime = 0.02
velDamp = 1.0
bodyforce = (0,-20.0) #only when gravity is turned off
#bodyforce = (0,0)
#viscosityStrength = 200
viscosityStrength = 0

#Tools
save = False
savename = "test.mp4"
debug = False
particleList = []
rectangles = []

def generateParticleGrid(numParticles):#particle generation
    particleGridLength = plotSize*ratio[0] * 0.8
    gridSize = int(math.sqrt(numParticles))
    if gridSize == 0 or gridSize == 1:
        gridSize = numParticles
    if gridSize == 1:
        spacing = particleGridLength
    else:
        spacing = particleGridLength / (gridSize - 1)
    offset = (plotSize*ratio[0] * 0.1,0)
    if gridSize == numParticles:
        print("small case")
        for i in range(gridSize):
            x = i * spacing + offset[0]
            y = offset[1]
            particleList.append((x, y))
    else:
        for i in range(gridSize):
            for j in range(gridSize):
                x = i * spacing + offset[0]
                y = j * spacing + offset[1]
                particleList.append((x, y))

generateParticleGrid(numParticles)
SPHObject = sph.SPH(particleList=particleList,obstacleList=obstacleList,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=debug,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
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
ax.set_position([0.1, 0.1, 0.8, 0.8])

for obstacle in obstacleList:
    rect = Rectangle((obstacle[3][0], obstacle[3][1]), obstacle[1][0] - obstacle[0][0],obstacle[0][1] - obstacle[3][1], fill=True, facecolor='gray')
    ax.add_patch(rect)
    rectangles.append(rect)

particleOfInterest = 25

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    if i == particleOfInterest:
        ballaxs.append(ax.plot([SPHObject.particleList[i][0]], [SPHObject.particleList[i][1]], 'r^', markersize=markersize)[0])
    else:
        ballaxs.append(ax.plot([SPHObject.particleList[i][0]], [SPHObject.particleList[i][1]], 'b^', markersize=markersize)[0])
floorLine = ax.axhline(y=SPHObject.plotFloor, color='red', linestyle='-')

collisionFactor = 1.4

SPHObject2 = sph.SPH(particleList=particleList,obstacleList=obstacleList,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=debug,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
                    pressureMultiplier=pressureMultiplier,targetDensity=targetDensity,\
                    smoothingRadius=smoothingRadius*collisionFactor,collisionDamping=collisionDamping,mass=mass,gravity=gravity,deltaTime=deltaTime,\
                    velDamp=velDamp,bodyforce=bodyforce,plotFloorSpeed=plotFloorSpeed,viscosityStrength=viscosityStrength)
SPHObject3 = sph.SPH(particleList=particleList,obstacleList=obstacleList,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=debug,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
                    pressureMultiplier=pressureMultiplier,targetDensity=targetDensity,\
                    smoothingRadius=smoothingRadius*collisionFactor*collisionFactor,collisionDamping=collisionDamping,mass=mass,gravity=gravity,deltaTime=deltaTime,\
                    velDamp=velDamp,bodyforce=bodyforce,plotFloorSpeed=plotFloorSpeed,viscosityStrength=viscosityStrength)
SPHObject4 = sph.SPH(particleList=particleList,obstacleList=obstacleList,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=debug,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
                    pressureMultiplier=pressureMultiplier,targetDensity=targetDensity,\
                    smoothingRadius=smoothingRadius*collisionFactor*collisionFactor*collisionFactor,collisionDamping=collisionDamping,mass=mass,gravity=gravity,deltaTime=deltaTime,\
                    velDamp=velDamp,bodyforce=bodyforce,plotFloorSpeed=plotFloorSpeed,viscosityStrength=viscosityStrength)

print(f'density with smoothing radius {SPHObject.smoothingRadius} is: {SPHObject.calculateDensity(particleOfInterest)}')
print(f'density with smoothing radius {SPHObject2.smoothingRadius} is: {SPHObject2.calculateDensity(particleOfInterest)}')
print(f'density with smoothing radius {SPHObject3.smoothingRadius} is: {SPHObject3.calculateDensity(particleOfInterest)}')
print(f'density with smoothing radius {SPHObject4.smoothingRadius} is: {SPHObject4.calculateDensity(particleOfInterest)}')

# Add circle to show smoothing radius around particle of interest
smoothing_circle = plt.Circle((SPHObject.particleList[particleOfInterest][0], 
                             SPHObject.particleList[particleOfInterest][1]),
                             radius=smoothingRadius,
                             fill=False,
                             color='red',
                             linestyle='--')
ax.add_patch(smoothing_circle)
smoothing_circle2 = plt.Circle((SPHObject2.particleList[particleOfInterest][0], 
                             SPHObject.particleList[particleOfInterest][1]),
                             radius=SPHObject2.smoothingRadius,
                             fill=False,
                             color='red',
                             linestyle='--')
ax.add_patch(smoothing_circle2)
smoothing_circle3 = plt.Circle((SPHObject3.particleList[particleOfInterest][0], 
                             SPHObject.particleList[particleOfInterest][1]),
                             radius=SPHObject3.smoothingRadius,
                             fill=False,
                             color='red',
                             linestyle='--')
ax.add_patch(smoothing_circle3)
smoothing_circle4 = plt.Circle((SPHObject4.particleList[particleOfInterest][0], 
                             SPHObject.particleList[particleOfInterest][1]),
                             radius=SPHObject4.smoothingRadius,
                             fill=False,
                             color='red',
                             linestyle='--')
ax.add_patch(smoothing_circle4)




plt.title("SPH Simulation")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()
