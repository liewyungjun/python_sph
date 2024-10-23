import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

import numpy as np
import random
import time

simulationsteps = 2000
numParticles = 49
densities= [0.0 for i in range(numParticles)]
particleList = []
gridLength = 30.0
pressureMultiplier = 20
targetDensity = 0.00003
mass = 1.0
smoothingRadius = 20.0
collisionDamping = 0.2
gravity = 10.0
deltaTime = 0.02
velocities = [(0.0,0.0) for x in range(numParticles)]
plotSize = 30.0
global plotFloor 
plotFloor = -plotSize
obstacleList = [[(-5,-4),(8,-4),(8,-8),(-5,-8)]]
rectangles = []

gravityOn = False
save = False
savename = "test.mp4"
bodyforce = (0,-20.0)
debug = False

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
        
predictedPositions = particleList.copy()

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
    for i in range(numParticles): #except youtself
        dist = math.dist(predictedPositions[point],predictedPositions[i])
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

def calculateSharedPressure(a,b):
    return (densityToPressure(a) + densityToPressure(b))/2

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
        sharedPressure = calculateSharedPressure(density,densities[particleIndex])
        pressureForce = (pressureForce[0] + densityToPressure(sharedPressure) * dirx * slope * mass / density, pressureForce[1] + densityToPressure(sharedPressure) * diry * slope * mass / density)
    return pressureForce

def resolveCollisions(pos,posIdx,obstacles):
    # Check for collision with side walls
    if abs(pos[0]) > plotSize:
        # if i == 2 and debug:
        #     print(f"{i} touch wall")
        velocities[posIdx] = (-velocities[posIdx][0]* collisionDamping,velocities[posIdx][1]) 
        pos[0] = particleList[posIdx][0] + velocities[posIdx][0] * deltaTime
    # Check for collision with ground and top
    if (pos[1]) < plotFloor or pos[1] > plotSize:
        # if i == 2 and debug:
        #     print(f"{i} touch ground")
        #     print(f'y: {particleList[i][1]} + {velocities[i][1]} * {deltaTime} = {pos[1]}')
        #     print(f'changing from {velocities[i][1]} to {-velocities[i][1] * collisionDamping}')
        velocities[posIdx] = (velocities[posIdx][0],-velocities[posIdx][1] * collisionDamping)        
        pos[1] = particleList[posIdx][1] + velocities[posIdx][1] * deltaTime

    for i in obstacles:
        precollisionx = particleList[posIdx][0]>i[0][0] and particleList[posIdx][0]<i[1][0]
        precollisiony = particleList[posIdx][1]<i[0][1] and particleList[posIdx][1]>i[3][1]
        collisionx = pos[0]>i[0][0] and pos[0]<i[1][0]
        collisiony = pos[1]<i[0][1] and pos[1]>i[3][1]
        if posIdx == 1:
            print(f'{posIdx} pre x due to {particleList[posIdx][0]} > {i[0][0]} and {particleList[posIdx][0]} < {i[1][0]}')
            print(f'{posIdx} pre y due to {particleList[posIdx][1]} < {i[0][1]} and {particleList[posIdx][1]} > {i[3][1]}')
        if collisionx and collisiony:
            # print(f'{posIdx}: collision!>>>>>>>>>>>>>>>>>>')
            # print(f'{posIdx} pre x is {precollisionx} and pre y is {precollisiony}')
            # print(f'{posIdx} pre x due to {particleList[posIdx][0]} > {i[0][0]} and {particleList[posIdx][0]} < {i[1][0]}')
            # print(f'{posIdx} pre y due to {particleList[posIdx][1]} < {i[0][1]} and {particleList[posIdx][1]} > {i[3][1]}')
            #inside rectangle
            if not precollisionx and precollisiony:
                print(f'{posIdx}: not pre x but pre y!')
            #if collisionx:
                velocities[posIdx] = (-velocities[posIdx][0]* collisionDamping,velocities[posIdx][1]) 
                pos[0] = particleList[posIdx][0] + velocities[posIdx][0] * deltaTime
            if precollisionx and not precollisiony:
                print(f'{posIdx}: not pre y but pre x!')
            #if collisiony:
                velocities[posIdx] = (velocities[posIdx][0],-velocities[posIdx][1] * collisionDamping)        
                pos[1] = particleList[posIdx][1] + velocities[posIdx][1] * deltaTime
    if posIdx == 1:
        print(f'moving from {particleList[posIdx]} to {pos}')
    return (pos[0],pos[1])


### ANIMATION CODE ##############################################################
# Set up the figure and axis
fig, ax = plt.subplots(figsize=(plotSize/4, plotSize/4))  # This scales the window size with plotSize
ax.set_xlim([-plotSize, plotSize])
ax.set_ylim([-plotSize, plotSize])
ax.set_aspect('equal')

for obstacle in obstacleList:
    rect = Rectangle((obstacle[3][0], obstacle[3][1]), obstacle[1][0] - obstacle[0][0],obstacle[0][1] - obstacle[3][1], fill=True, facecolor='gray')
    ax.add_patch(rect)
    rectangles.append(rect)

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([], [], 'bo', markersize=10)[0])

def update_sim():

    for i in range(numParticles):#add gravity
        velocities[i] = (velocities[i][0], velocities[i][1] -gravity * deltaTime )
        predictedPositions[i] = (particleList[i][0] + velocities[i][0] * deltaTime, particleList[i][1] + velocities[i][1] * deltaTime)
    
    updateDensities() #update densities
    
    for i in range(numParticles): #update velocities
        pf = calculatePressureForce(i)
        pa = (pf[0]/densities[i],pf[1]/densities[i]) #pressure acceleration
        if i == 2 and debug:
            print(f'{i}: {velocities[i][1]} + ({pa[1]} - {gravity}) * {deltaTime}')
        if gravityOn: #add pressure acceleration
            velocities[i] = (velocities[i][0] + pa[0] * deltaTime, velocities[i][1] + pa[1] * deltaTime )
        else: #direct acceleration assignment + bodyforce
            velocities[i] = (pa[0] * deltaTime + bodyforce[0], pa[1] * deltaTime + bodyforce[1])
        ###TODO fix gravity, as direct assignment does not work for obstacles
    for i in range(numParticles): #update positions
        newi =[particleList[i][0] + velocities[i][0] * deltaTime, particleList[i][1] + velocities[i][1] * deltaTime]
        if i == 1:
            print(f'testing to move from {particleList[i]} to {newi}')
        particleList[i] = resolveCollisions(newi,i,obstacleList)
        if i == 1:
            print(f'Particle Location is now {particleList[i]}')

def update(frame):
    # Update position and velocity for both balls
    update_sim()
    # print("densities table")
    # for i in range(len(densities)):
    #     print(f'{i}: {densities[i]}')
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data([particleList[i][0]],[particleList[i][1]])
    print(f'{frame}----------------')
    if frame > 50:
        global plotFloor
        plotFloor +=0.1
    return ballaxs

# Create the animation
anim = animation.FuncAnimation(fig, update, frames=simulationsteps, interval=20, blit=True,repeat=False)
if save:
    writervideo = animation.FFMpegWriter(fps=60) 
    anim.save(savename, writer=writervideo) 


plt.title("SPH Simulation")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()
