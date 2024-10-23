import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random
import time


numParticles = 4
densities= [0.0 for i in range(numParticles)]
particleList = []
gridLength = 15.0
pressureMultiplier = 0.1
targetDensity = 0.0001
mass = 1.0
smoothingRadius = 10.0
collisionDamping = 0.2
gravity = 10.0
deltaTime = 0.02
velocities = [(0.0,0.0) for x in range(numParticles)]
plotSize = 40.0

############particle generation################
gridSize = int(math.sqrt(numParticles))
spacing = gridLength / (gridSize - 1)
for i in range(gridSize):
    for j in range(gridSize):
        x = i * spacing
        y = j * spacing
        particleList.append((x, y))
#print(particleList)
##############################################
 

def smoothingKernel(radius,dist):
    if dist > radius:
        return 0
    volume = math.pi * pow(radius,8)/4
    value = max(0,radius * radius - dist * dist)
    return value * value / volume

def smoothingKernelDerivative(radius,dist):
    if dist > radius:
        return 0
    return (12 * -1 * (radius - dist)) / (math.pi * pow(radius,4))
    
def calculateDensity(point):
    density = 0.0
    for i in range(len(particleList)): #except youtself
        dist = math.dist(particleList[point],particleList[i])
        #if dist < smoothingRadius:
            #print(dist)
        influence = smoothingKernel(smoothingRadius,dist)
        density += mass * influence
    if density < 1e-7:
        print(f"{point} density zero")
    return density

def updateDensities():
    for i in range(numParticles):
        densities[i] = calculateDensity(i)

def densityToPressure(density):
    densityError = density - targetDensity
    return densityError * pressureMultiplier



def calculatePressureForce(particleIndex):
    pressureForce = (0,0)
    for i in range(numParticles):
        if i == particleIndex:
            continue
        dist = math.dist(particleList[particleIndex],particleList[i])
        if dist == 0:
            dirx = -1 if random.randint(0,1) == 0 else 1
            dirx = -1 if random.randint(0,1) == 0 else 1
        else:
            dirx = (particleList[i][0] - particleList[particleIndex][0])/dist
            diry = (particleList[i][1] - particleList[particleIndex][1])/dist
        slope = smoothingKernelDerivative(smoothingRadius,dist)
        density = densities[i]
        pressureForce = (pressureForce[0] + densityToPressure(density) * dirx * slope * mass / density, pressureForce[1] + densityToPressure(density) * diry * slope * mass / density)
        # if abs(sum(pressureForce))<1e-7:
        print(f'{particleIndex} to {i} densityToPressue:{densityToPressure(density):.5f}  dirx:{dirx:.5f} slope: {slope:.5f} density:{density:.5f} dist:{dist:.5f}')
        print(f'pf:{pressureForce}')
        #print(f'{particleIndex} - {i}')
    # if abs(sum(pressureForce))<1e-7:
    #     print(f'{particleIndex} has no pf')
    # else:
    #     print(f'{particleIndex} has pf {pressureForce}')
        
    return pressureForce



# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlim([-plotSize, plotSize])
ax.set_ylim([-plotSize, plotSize])
ax.set_aspect('equal')

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([], [], 'ro', markersize=20)[0])

def update_sim():
    for i in range(len(particleList)): #update velocities
        velocities[i] = (velocities[i][0], velocities[i][1] - gravity * deltaTime)
        print(f'{i} velocity is {velocities[i]}')
        #if abs(sum(velocities[i]))<1e-7:
            #print(f'{i} not moving')
    
    for i in range(len(particleList)): #update positions
        newi =[particleList[i][0] + velocities[i][0] * deltaTime, particleList[i][1] + velocities[i][1] * deltaTime]
        
        # Check for collision with ground
        if newi[0] < -plotSize:
            velocities[i] = (-velocities[i][0],velocities[i][1])
            newi[0] = particleList[i][0] + velocities[i][0] * deltaTime
        # Check for collision with walls
        if abs(newi[1]) > plotSize:
            velocities[i] = (velocities[i][0],-velocities[i][1] * collisionDamping)
            newi[1] = particleList[i][1] + velocities[i][1] * deltaTime

        particleList[i] = (newi[0],newi[1])

def update(frame):
    # Update position and velocity for both balls
    update_sim()
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data(particleList[i][0],particleList[i][1])
    print(frame)
    return ballaxs

# Create the animation
anim = animation.FuncAnimation(fig, update, frames=500, interval=20, blit=True,repeat=False)

plt.title("Two Bouncing Balls Simulation")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()