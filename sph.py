import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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
        
predictedPositions = particleList

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

# Set up the figure and axis
fig, ax = plt.subplots(figsize=(plotSize/4, plotSize/4))  # This scales the window size with plotSize
ax.set_xlim([-plotSize, plotSize])
ax.set_ylim([-plotSize, plotSize])
ax.set_aspect('equal')

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
        
    
    for i in range(numParticles): #update positions
        newi =[particleList[i][0] + velocities[i][0] * deltaTime, particleList[i][1] + velocities[i][1] * deltaTime]
        
        # Check for collision with side walls
        if abs(newi[0]) > plotSize:
            if i == 2 and debug:
                print(f"{i} touch wall")
            velocities[i] = (-velocities[i][0]* collisionDamping,velocities[i][1]) 
            newi[0] = particleList[i][0] + velocities[i][0] * deltaTime
        
        # Check for collision with ground and top
        if abs(newi[1]) > plotSize:
            if i == 2 and debug:
                print(f"{i} touch ground")
                print(f'y: {particleList[i][1]} + {velocities[i][1]} * {deltaTime} = {newi[1]}')
                print(f'changing from {velocities[i][1]} to {-velocities[i][1] * collisionDamping}')
            velocities[i] = (velocities[i][0],-velocities[i][1] * collisionDamping)        
            newi[1] = particleList[i][1] + velocities[i][1] * deltaTime
        if i == 2 and debug:
            print(f'{i} velocity is {velocities[i]}')
        particleList[i] = (newi[0],newi[1])

def update(frame):
    # Update position and velocity for both balls
    update_sim()
    print("densities table")
    for i in range(len(densities)):
        print(f'{i}: {densities[i]}')
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data(particleList[i][0],particleList[i][1])
    print(f'{frame}----------------')
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
