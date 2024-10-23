import math 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random
import time

simulationsteps = 2000
numParticles = 64
densities= [0.0 for i in range(numParticles)]
particleList = []
gridLength = 15.0
pressureMultiplier = 30
targetDensity = 0.00003
mass = 1.0
smoothingRadius = 20.0
collisionDamping = 0.2
gravity = 10.0
deltaTime = 0.02
velocities = [(0.0,0.0) for x in range(numParticles)]
plotSize = 20.0

gravityOn = False
save = False
savename = "test.mp4"
bodyforce = (0,-20.0)

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
        # if abs(sum(pressureForce))<1e-7:
        #print(f'{particleIndex} to {i} densityToPressue:{densityToPressure(density):.5f}  dirx:{dirx:.5f} slope: {slope:.5f} density:{density:.5f} dist:{dist:.5f}')
        #print(f'pf:{pressureForce}')
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
ax.figure(figsize=(10,6))

# Initialize the balls
ballaxs = []
for i in range(numParticles):
    ballaxs.append(ax.plot([], [], 'bo', markersize=1)[0])

def update_sim():

    updateDensities() #update densities
    for i in range(len(particleList)):
        velocities[i]  = (velocities[i][0],velocities[i][1] - gravity * deltaTime)
    #print(densities)

    for i in range(len(particleList)): #update velocities
        pf = calculatePressureForce(i)
        # if abs(sum(pf))<1e-7:
        #     print(f'{i} no pressure force with {sum(pf)}')
        pa = (pf[0]/densities[i],pf[1]/densities[i]) #pressure acceleration
        #velocities[i] = (velocities[i][0] + pa[0] * deltaTime, velocities[i][1] + pa[1] * deltaTime  - gravity * deltaTime)
        if gravityOn:
            velocities[i] = (velocities[i][0] + pa[0] * deltaTime, velocities[i][1] + pa[1] * deltaTime)
        else:
            velocities[i] = (pa[0] * deltaTime + bodyforce[0], pa[1] * deltaTime + bodyforce[1])
        print(f'{i} velocity is {velocities[i]}')
        #if abs(sum(velocities[i]))<1e-7:
            #print(f'{i} not moving')
    
    for i in range(len(particleList)): #update positions
        newi =[particleList[i][0] + velocities[i][0] * deltaTime, particleList[i][1] + velocities[i][1] * deltaTime]
        
        # Check for collision with and top
        if abs(newi[0]) > plotSize:
            velocities[i] = (-velocities[i][0]* collisionDamping,velocities[i][1]) 
            newi[0] = particleList[i][0] + velocities[i][0] * deltaTime
            #vy[-1] = -collisionDamping * vy[-1]
        
        # Check for collision with walls
        if abs(newi[1]) > plotSize:
            velocities[i] = (velocities[i][0],-velocities[i][1] * collisionDamping)        
            newi[1] = particleList[i][1] + velocities[i][1] * deltaTime
            #vx[-1] = -collisionDamping * vx[-1]

        particleList[i] = (newi[0],newi[1])
    #time.sleep(2)

def update(frame):
    # Update position and velocity for both balls
    update_sim()
    print("densities table")
    for i in range(len(densities)):
        print(f'{i}: {densities[i]}')
    #update ball loc 
    for i in range(len(ballaxs)):
        ballaxs[i].set_data(particleList[i][0],particleList[i][1])
    print(frame)
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
