import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Rectangle
import sph_3D

import os
from datetime import datetime

import numpy as np

#Simulation graphical setup
simulationsteps = 2000
particleMarkerSize = 2 #size of particle in axes units
plotSize = 50.0
ratio = [3,4,2] #ratio of plot size
axesScaling = 10 #size of axes scaling factor e.g. 10 units/axesScaling = plot cm size
floodRisingFrameStart = 50
plotFloor = 0.0
plotFloorSpeed = 0.1
gridSpacing = 10

#Simulation physics setup
#obstacleList = [[(70,50),(100,50),(100,30),(70,30)],[(15,120),(65,120),(65,90),(15,90)]] #TODO: add 3D
obstacleList = []
numParticles = 20
floodRising = True
gravityOn = False

#pressureMultiplier = 20000
#pressureMultiplier = 100000
pressureMultiplier = 500000
targetDensity = 0.002
smoothingRadius = 40.0
collisionDamping = 0.8
mass = 1.0
gravity = 10.0
deltaTime = 0.02
velDamp = 1.0
bodyforce = [0,0.0,-20.0] #TODO: z or y is down??  #only when gravity is turned off
#bodyforce = (0,0)
#viscosityStrength = 200
viscosityStrength = 0

#Tools
save = True
savename = "3D.mp4"
debug = False
mapname = "upcup"

particleList = []
rectangles = []

def generateParticleGrid(numParticles):#particle generation
    x_cap = plotSize * ratio[0] / gridSpacing
    if x_cap > numParticles:
        for i in range(numParticles):
            x = i * gridSpacing
            y = gridSpacing
            particleList.append([x,y,0.0])
    else:
        y_offset = 1
        rownum = 0
        for i in range(numParticles):
            if i // x_cap > rownum:
                y_offset +=1
                rownum +=1
            x = i%x_cap * gridSpacing
            y = y_offset * gridSpacing
            particleList.append([x,y,15.0])
    particleList[-1] = [x,y,5.0]
            
    # particleGridLength = plotSize*ratio[0] * 0.8
    # gridSize = int(math.sqrt(numParticles))
    # if gridSize == 0 or gridSize == 1:
    #     gridSize = numParticles
    # if gridSize == 1:
    #     spacing = particleGridLength
    # else:
    #     spacing = particleGridLength / (gridSize - 1)
    # offset = (plotSize*ratio[0] * 0.1,0)
    # if gridSize == numParticles:
    #     print("small case")
    #     for i in range(gridSize):
    #         x = i * spacing + offset[0]
    #         y = offset[1]
    #         particleList.append((x, y))
    # else:
    #     for i in range(gridSize):
    #         for j in range(gridSize):
    #             x = i * spacing + offset[0]
    #             y = j * spacing + offset[1]
    #             particleList.append((x, y))

def readMap(mapname):
    with open(f'maps/{mapname}.txt', 'r') as f:
        content = f.read()
        # Convert string representation of list to actual list
        points = eval(content)
        #print(points)
        # Since we have a single obstacle, we'll return it in the obstacleList format
        obstacleList = points
        #obstacleList = [[(x, y) for x, y in points]]
        return obstacleList
    
def create_cuboid(center, size):
    x, y, z = center
    dx, dy, dz = size
    vertices = np.array([
        [x-dx/2, y-dy/2, z-dz/2],
        [x+dx/2, y-dy/2, z-dz/2],
        [x+dx/2, y+dy/2, z-dz/2],
        [x-dx/2, y+dy/2, z-dz/2],
        [x-dx/2, y-dy/2, z+dz/2],
        [x+dx/2, y-dy/2, z+dz/2],
        [x+dx/2, y+dy/2, z+dz/2],
        [x-dx/2, y+dy/2, z+dz/2]
    ])
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[0], vertices[3], vertices[7], vertices[4]]
    ]
    return faces
#obstacleList = readMap(mapname)
cuboids = [
    ((75, 150, 0), (20, 20, 40), 'red'),
    ((40, 70, 0), (20, 20, 40), 'grey'),
    ((120, 50, 0), (20, 20, 40), 'green'),
    ((20, 190, 0),(20, 20, 40), 'yellow')
]
generateParticleGrid(numParticles)

SPHObject = sph_3D.SPH_3D(particleList=particleList,obstacleList=cuboids,numParticles=numParticles,plotSize=plotSize,plotFloor=plotFloor,\
                    debug=debug,ratio=ratio,gravityOn=gravityOn,floodRising=floodRising,\
                    pressureMultiplier=pressureMultiplier,targetDensity=targetDensity,\
                    smoothingRadius=smoothingRadius,collisionDamping=collisionDamping,mass=mass,gravity=gravity,deltaTime=deltaTime,\
                    velDamp=velDamp,bodyforce=bodyforce,plotFloorSpeed=plotFloorSpeed,viscosityStrength=viscosityStrength)


### ANIMATION CODE ##############################################################
#Simulation graphical size calculations
x_limits = (0, ratio[0]*plotSize)
y_limits = (0, ratio[1]*plotSize)
z_limits = (0, ratio[2]*plotSize)
figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
points_per_unit = inches_per_unit * 72  # convert to points
markersize = particleMarkerSize * points_per_unit  # 5 axes unit marker diameter

# Set up the figure and axis
fig = plt.figure(figsize=(figsize), dpi=122)
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(x_limits)
ax.set_ylim(y_limits)
ax.set_zlim(z_limits)
ax.set_box_aspect([1, 1, 1])
#plt.xticks(np.arange(x_limits[0], x_limits[1], 5.0))
plt.title(f"SPH Simulation\nParticles: {numParticles}, Target Density: {targetDensity:.2e}")
ax.set_position([0.1, 0.1, 0.8, 0.8])
ax.set_proj_type('persp')  # Use perspective projection
#ax.set_sort_zpos(True)


# Plot each cuboid
for center, size, color in cuboids:
    faces = create_cuboid(center, size)
    ax.add_collection3d(Poly3DCollection(faces, facecolor=color, alpha=1.0, edgecolor='k',sort_zpos=True, zorder =2))
    #collection.set_sort_zpos(True)
    #ax.add_collection3d(collection)


for obstacle in obstacleList:
    rect = Rectangle((obstacle[3][0], obstacle[3][1]), obstacle[1][0] - obstacle[0][0],obstacle[0][1] - obstacle[3][1], fill=True, facecolor='gray')
    ax.add_patch(rect)
    rectangles.append(rect)

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([], [], [], 'b^', markersize=markersize,zorder = 1)[0])

# Create the line at initialization
x_line = np.linspace(x_limits[0], x_limits[1], 2)
y_line = np.full_like(x_line, SPHObject.plotFloor)
z_line = np.zeros_like(x_line)
floor_line = ax.plot(x_line, y_line, z_line, 'r-', linewidth=2)[0]

def update(frame):
    # Update position and velocity for both balls
    SPHObject.step()
    # print("densities table")
    # for i in range(len(SPHObject.densities)):
    #     print(f'{i}: {SPHObject.densities[i]}')
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data_3d([SPHObject.particleList[i][0]],[SPHObject.particleList[i][1]],[SPHObject.particleList[i][2]])
    print(f'{frame}----------------')
    if frame > floodRisingFrameStart and floodRising:
        #print("floodrising")
        SPHObject.plotFloor +=plotFloorSpeed
        y_line = np.full_like(x_line, SPHObject.plotFloor)
        floor_line.set_data_3d(x_line, y_line, z_line)
    
    return ballaxs + [floor_line]

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
