import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
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
obstacleList = [[(70,50),(100,50),(100,30),(70,30)],[(15,120),(65,120),(65,90),(15,90)]]
numParticles = 3
floodRising = True
gravityOn = False

#pressureMultiplier = 200
#pressureMultiplier = 100000
pressureMultiplier = 500000
targetDensity = 0.002
smoothingRadius = 40.0
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

#particleList = [(25,10),(45,10),(75,10),(95,10)]
particleList = [(85,25),(95,25),(100,25)]
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

#generateParticleGrid(numParticles)
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
plt.title(f"SPH Simulation\nParticles: {numParticles}, Target Density: {targetDensity:.2e}")
ax.set_position([0.1, 0.1, 0.8, 0.8])

for obstacle in obstacleList:
    rect = Rectangle((obstacle[3][0], obstacle[3][1]), obstacle[1][0] - obstacle[0][0],obstacle[0][1] - obstacle[3][1], fill=True, facecolor='gray')
    ax.add_patch(rect)
    rectangles.append(rect)

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([], [], 'b^', markersize=markersize)[0])
floorLine = ax.axhline(y=SPHObject.plotFloor, color='red', linestyle='-')

def update(frame):
    # Update position and velocity for both balls
    SPHObject.step()
    # print("densities table")
    # for i in range(len(SPHObject.densities)):
    #     print(f'{i}: {SPHObject.densities[i]}')
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data([SPHObject.particleList[i][0]],[SPHObject.particleList[i][1]])
    print(f'{frame}----------------')
    if frame > floodRisingFrameStart and floodRising:
        #print("floodrising")
        SPHObject.plotFloor +=plotFloorSpeed
        floorLine.set_ydata([SPHObject.plotFloor, SPHObject.plotFloor])
    if frame > floodRisingFrameStart and not floodRising:
        print("bodyforce changed")
        SPHObject.bodyforce = (-bodyforce[0],-bodyforce[1])

    return ballaxs + [floorLine]
    #return ballaxs 

# Create the animation
anim = animation.FuncAnimation(fig, update, frames=simulationsteps, interval=deltaTime * 1000, blit=True,repeat=False)
if save:
    # Create timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Create folder name with simulation name and timestamp
    folder_name = f"results/{savename.split('.')[0]}_{timestamp}"
    
    # Create folders if they don't exist
    os.makedirs(folder_name, exist_ok=True)
    
    # Update save path to include new folder
    save_path = os.path.join(folder_name, savename)
    
    # Save animation to the new folder
    writervideo = animation.FFMpegWriter(fps=60)
    anim.save(save_path, writer=writervideo)
    
    # Create and write parameters to txt file
    params_file = os.path.join(folder_name, "parameters.txt")
    with open(params_file, "w") as f:
        f.write(f"Simulation Parameters:\n")
        f.write(f"------------------------------------\n")
        f.write(f"Simulation graphical setup\n")
        f.write(f"Simulation Steps: {simulationsteps}\n")
        f.write(f"Particle Marker Size: {particleMarkerSize}\n")
        f.write(f'Plot size: {plotSize}\n')
        f.write(f'Ratio: {ratio}\n')
        f.write(f"Axes Scaling: {axesScaling}\n")
        f.write(f"Flood Rising Frame Start: {floodRisingFrameStart}\n")
        f.write(f'Plot floor: {plotFloor}\n')
        f.write(f"Plot Floor Speed: {plotFloorSpeed}\n")
        f.write(f"X Limits: {x_limits}\n")
        f.write(f"Y Limits: {y_limits}\n")
        f.write(f"------------------------------------\n")
        f.write(f"Simulation physics setup\n")
        f.write(f'Obstacle list: {obstacleList}\n')
        f.write(f"Number of Particles: {numParticles}\n")
        f.write(f"Flood Rising: {floodRising}\n")
        f.write(f'GravityOn: {gravityOn}\n')
        f.write(f'Pressure multiplier: {pressureMultiplier}\n')
        f.write(f'Target density: {targetDensity}\n')
        f.write(f'Smoothing radius: {smoothingRadius}\n')
        f.write(f'Collision damping: {collisionDamping}\n')
        f.write(f'Mass: {mass}\n')
        f.write(f'Gravity: {gravity}\n')
        f.write(f'Delta time: {deltaTime}\n')
        f.write(f'VelDamp: {velDamp}\n')
        f.write(f"Body Force: {bodyforce}\n")
        f.write(f'Viscosity strength: {viscosityStrength}\n')

plt.title(f"SPH Simulation\nParticles: {numParticles}, Target Density: {targetDensity:.2e}")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()
