import math 

import numpy as np
import random

class SPH:
    def __init__(self,particleList,obstacleList,numParticles,plotSize,plotFloor,\
                ratio,debug,floodRising,gravityOn,pressureMultiplier,targetDensity,\
                smoothingRadius, collisionDamping,mass,gravity,deltaTime,velDamp,\
                bodyforce):
        self.particleList = particleList
        self.numParticles = numParticles
        self.obstacleList = obstacleList
        self.gravityOn = gravityOn
        self.floodRising = floodRising

        #physics params
        self.pressureMultiplier = pressureMultiplier
        self.targetDensity = targetDensity
        self.smoothingRadius = smoothingRadius
        self.collisionDamping = collisionDamping
        self.mass = mass
        self.gravity = gravity
        self.deltaTime = deltaTime
        self.velDamp = velDamp
        self.bodyforce = bodyforce
        
        #simulation params
        self.plotSize = plotSize
        self.plotFloor = plotFloor
        self.ratio = ratio
        self.debug = debug
        

        #Initialisers
        self.densities= [0.0 for i in range(self.numParticles)]
        self.velocities = [(0.0,0.0) for x in range(self.numParticles)]
        self.predictedPositions = particleList.copy()

    def smoothingKernel(self,radius,dist):
        if dist > radius:
            return 0
        volume = math.pi * pow(radius,8)/4
        value = max(0,radius * radius - dist * dist)
        return value * value / volume

    def smoothingKernelDerivative(self,radius,dist):
        if dist > radius:
            return 0
        return (12 * -1 * (radius - dist)) / (math.pi * pow(radius,4))
        
    def calculateDensity(self,point):
        density = 0.0
        for i in range(self.numParticles): #except youtself
            dist = math.dist(self.predictedPositions[point],self.predictedPositions[i])
            influence = self.smoothingKernel(self.smoothingRadius,dist)
            density += self.mass * influence
        if density < 1e-7:
            print(f"{point} density zero")
        return density

    def updateDensities(self):
        for i in range(self.numParticles):
            self.densities[i] = self.calculateDensity(i)

    def densityToPressure(self,density):
        densityError = density - self.targetDensity
        return densityError * self.pressureMultiplier

    def calculateSharedPressure(self,a,b):
        return (self.densityToPressure(a) + self.densityToPressure(b))/2

    def calculatePressureForce(self,particleIndex):
        pressureForce = (0,0)
        for i in range(self.numParticles):
            if i == particleIndex:
                continue
            dist = math.dist(self.particleList[particleIndex],self.particleList[i])
            if dist == 0:
                dirx = -1 if random.randint(0,1) == 0 else 1
                dirx = -1 if random.randint(0,1) == 0 else 1
            else:
                dirx = (self.particleList[i][0] - self.particleList[particleIndex][0])/dist
                diry = (self.particleList[i][1] - self.particleList[particleIndex][1])/dist
            slope = self.smoothingKernelDerivative(self.smoothingRadius,dist)
            density = self.densities[i]
            sharedPressure = self.calculateSharedPressure(density,self.densities[particleIndex])
            pressureForce = (pressureForce[0] + \
                             self.densityToPressure(sharedPressure) * dirx * slope * self.mass / density, \
                             pressureForce[1] + \
                             self.densityToPressure(sharedPressure) * diry * slope * self.mass / density)
        return pressureForce

    def resolveCollisions(self,pos,posIdx,obstacles):
        # Check for collision with side walls
        if pos[0] > self.plotSize * self.ratio[0] or pos[0]<0.0:
            if self.gravityOn:
                self.velocities[posIdx] = (-self.velocities[posIdx][0]* self.collisionDamping,self.velocities[posIdx][1]) 
                pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
            else:
                pos[0] = self.particleList[posIdx][0]
        # Check for collision with ground and top
        if (pos[1]) < self.plotFloor or pos[1] > self.ratio[1]*self.plotSize:
            if self.gravityOn:
                self.velocities[posIdx] = (self.velocities[posIdx][0],-self.velocities[posIdx][1] * self.collisionDamping)        
                pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
            else:
                pos[1] = self.particleList[posIdx][1]
            if self.floodRising and pos[1] < self.plotFloor:
                #pos[1] = self.plotFloor
                pos[1] += 0.2

        for i in obstacles:
            precollisionx = self.particleList[posIdx][0]>i[0][0] and self.particleList[posIdx][0]<i[1][0]
            precollisiony = self.particleList[posIdx][1]<i[0][1] and self.particleList[posIdx][1]>i[3][1]
            collisionx = pos[0]>i[0][0] and pos[0]<i[1][0]
            collisiony = pos[1]<i[0][1] and pos[1]>i[3][1]
            if collisionx and collisiony:
                #inside rectangle
                if not precollisionx and precollisiony:
                #if collisionx:
                    if self.gravityOn:
                        self.velocities[posIdx] = (-self.velocities[posIdx][0]* self.collisionDamping,self.velocities[posIdx][1]) 
                        pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
                    else:
                        pos[0] = self.particleList[posIdx][0]
                if precollisionx and not precollisiony:
                #if collisiony:
                    if self.gravityOn:
                        self.velocities[posIdx] = (self.velocities[posIdx][0],-self.velocities[posIdx][1] * self.collisionDamping)        
                        pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
                    else:
                        pos[1] = self.particleList[posIdx][1]
        return (pos[0],pos[1])

    def step(self):

        for i in range(self.numParticles):#add gravity
            self.velocities[i] = (self.velocities[i][0], self.velocities[i][1] -self.gravity * self.deltaTime )
            #print(f'predicted Pos:{self.predictedPositions} particleList:{self.particleList} velocicities:{self.velocities}')
            self.predictedPositions[i] = (self.particleList[i][0] + self.velocities[i][0] * self.deltaTime, self.particleList[i][1] + self.velocities[i][1] * self.deltaTime)
        
        self.updateDensities() #update densities
        
        for i in range(self.numParticles): #update velocities
            pf = self.calculatePressureForce(i)
            pa = (pf[0]/self.densities[i],pf[1]/self.densities[i]) #pressure acceleration
            if self.gravityOn: #add pressure acceleration
                self.velocities[i] = (self.velDamp * self.velocities[i][0] + pa[0] * self.deltaTime, self.velDamp * self.velocities[i][1] + pa[1] * self.deltaTime )
            else: #direct acceleration assignment + bodyforce
                self.velocities[i] = (pa[0] * self.deltaTime + self.bodyforce[0], pa[1] * self.deltaTime + self.bodyforce[1])
            ###TODO fix gravity, as direct assignment does not work for obstacles
        for i in range(self.numParticles): #update positions
            newi =[self.particleList[i][0] + self.velocities[i][0] * self.deltaTime, self.particleList[i][1] + self.velocities[i][1] * self.deltaTime]
            if i == 1 and self.debug:
                print(f'testing to move from {self.particleList[i]} to {newi}')
            self.particleList[i] = self.resolveCollisions(newi,i,self.obstacleList)