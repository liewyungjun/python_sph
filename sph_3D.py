import math 

import numpy as np
import random

class SPH_3D:
    def __init__(self,particleList,obstacleList,numParticles,plotSize,plotFloor,\
                ratio,debug,floodRising,gravityOn,pressureMultiplier,targetDensity,\
                smoothingRadius, collisionDamping,mass,gravity,deltaTime,velDamp,\
                bodyforce,plotFloorSpeed,viscosityStrength):
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
        self.plotFloorSpeed = plotFloorSpeed
        self.viscosityStrength = viscosityStrength
        
        #simulation params
        self.plotSize = plotSize
        self.plotFloor = plotFloor
        self.ratio = ratio
        self.debug = debug
        

        #Initialisers (global information)
        self.densities= [0.0 for i in range(self.numParticles)]
        self.velocities = [np.array([0.0,0.0,0.0]) for x in range(self.numParticles)]
        self.predictedPositions = particleList.copy()

    def smoothingKernel(self,radius,dist):
        if dist > radius:
            return 0
        volume = math.pi * pow(radius,4)/6
        return ((radius-dist) * (radius-dist))/ volume

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
            # if influence >0:
            #     print(f'density contribution to {point} of {i} is {influence} with distance {dist}')
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
        pressureForce = np.array([0.0,0.0,0.0])
        for i in range(self.numParticles):
            if i == particleIndex:
                continue
            dist = math.dist(self.particleList[particleIndex],self.particleList[i])
            if dist == 0:
                dirx = -1 if random.randint(0,1) == 0 else 1
                diry = -1 if random.randint(0,1) == 0 else 1
                dirz = -1 if random.randint(0,1) == 0 else 1
            else:
                dirx = (self.particleList[i][0] - self.particleList[particleIndex][0])/dist
                diry = (self.particleList[i][1] - self.particleList[particleIndex][1])/dist
                dirz = (self.particleList[i][2] - self.particleList[particleIndex][2])/dist #TODO: dirz is always zero, need to spawn them on top of each other?
            slope = self.smoothingKernelDerivative(self.smoothingRadius,dist)
            density = self.densities[i]
            sharedPressure = self.calculateSharedPressure(density,self.densities[particleIndex])
            
            # pressureForce1 = (pressureForce[0] + \
            #                  self.densityToPressure(sharedPressure) * dirx * slope * self.mass / density, \
            #                  pressureForce[1] + \
            #                  self.densityToPressure(sharedPressure) * diry * slope * self.mass / density)
            pressureForce = np.array([pressureForce[0] + \
                             sharedPressure * dirx * slope * self.mass / density, \
                             pressureForce[1] + \
                             sharedPressure * diry * slope * self.mass / density, \
                             pressureForce[2] + \
                             sharedPressure * dirz * slope * self.mass / density])
            #if particleIndex == 55:
                #print(f'pressure force = sharedPressure {sharedPressure} * dirz {dirz} * slope {slope} * self.mass {self.mass} / density {density}')
            # print(f'sharedPressure is {sharedPressure}')
            # print(f'Other is {self.densityToPressure(sharedPressure)}')
            # print(f'pressureforce is {pressureForce}')
            # print(f'pressureforce1 is {pressureForce1}')
        #print(f'{pressureForce}')
        return pressureForce

    def resolveCollisions(self,pos,posIdx,obstacles):
        # Check for collision with side walls
        if pos[0] > self.plotSize * self.ratio[0] or pos[0]<0.0:
            if self.gravityOn:
                self.velocities[posIdx][0] = -self.velocities[posIdx][0]* self.collisionDamping
                pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
            else:
                pos[0] = self.particleList[posIdx][0]
        if pos[1] > self.plotSize * self.ratio[1] or pos[1]<0.0:
            if self.gravityOn:
                self.velocities[posIdx][1] = -self.velocities[posIdx][1]* self.collisionDamping
                pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
            else:
                pos[1] = self.particleList[posIdx][1]
        # Check for collision with ground and top
        #if (pos[1]) < self.plotFloor or pos[1] > self.ratio[1]*self.plotSize:
        if (pos[2]) < 0.0:
            if self.gravityOn:
                if self.velocities[posIdx][2]>0 and (pos[2]) < self.plotFloor:
                    pos[2] = pos[2]
                else:
                    #TODO: 
                    self.velocities[posIdx][2] = -self.velocities[posIdx][2] * self.collisionDamping
                    pos[2] = self.particleList[posIdx][2] + self.velocities[posIdx][2] * self.deltaTime
            else:
                pos[2] = self.particleList[posIdx][2]
            # if self.floodRising and pos[1] < self.plotFloor: #below flood level
            #     #pos[1] = self.plotFloor
            #     pos[1] += self.plotFloorSpeed * 3

        # for i in obstacles:
        #     precollisionx = self.particleList[posIdx][0]>i[0][0] and self.particleList[posIdx][0]<i[1][0]
        #     precollisiony = self.particleList[posIdx][1]<i[0][1] and self.particleList[posIdx][1]>i[3][1]
        #     collisionx = pos[0]>i[0][0] and pos[0]<i[1][0]
        #     collisiony = pos[1]<i[0][1] and pos[1]>i[3][1]
        #     if collisionx and collisiony:
        #         #inside rectangle
        #         if not precollisionx and precollisiony:
        #         #if collisionx:
        #             if self.gravityOn:
        #                 #TODO: 
        #                 self.velocities[posIdx] = (-self.velocities[posIdx][0]* self.collisionDamping,self.velocities[posIdx][1]) 
        #                 pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
        #             else:
        #                 pos[0] = self.particleList[posIdx][0]
        #         if precollisionx and not precollisiony:
        #         #if collisiony:
        #             if self.gravityOn:
        #                 #TODO: 
        #                 self.velocities[posIdx] = (self.velocities[posIdx][0],-self.velocities[posIdx][1] * self.collisionDamping)        
        #                 pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
        #             else:
        #                 pos[1] = self.particleList[posIdx][1]
        return pos[0],pos[1],pos[2]
    
    def viscositySmoothingKernel(self,radius,dist):
        if dist > radius:
            return 0
        volume = math.pi * pow(radius,8)/4
        rd = radius * radius - dist * dist
        return (rd * rd * rd)/ volume
    
    def calculateViscosityForce(self,particleIndex):
        viscosityForce = np.array([0.0,0.0,0.0])
        position = self.particleList[particleIndex]
        for i in range(self.numParticles): #except youtself
            dist = math.dist(self.particleList[i],self.particleList[particleIndex])
            influence = self.viscositySmoothingKernel(self.smoothingRadius,dist)
            temp = (self.velocities[i] - self.velocities[particleIndex]) * influence
            #temp = ((self.velocities[i][0] - self.velocities[particleIndex][0]) * influence,(self.velocities[i][1] - self.velocities[particleIndex][1]) * influence)
            viscosityForce += temp
        return viscosityForce * self.viscosityStrength
    
    def step(self):

        for i in range(self.numParticles):#add gravity
            self.velocities[i][2] -= self.gravity * self.deltaTime

            self.predictedPositions[i] = self.particleList[i] + self.velocities[i] * self.deltaTime
        
        self.updateDensities() #update densities
        
        for i in range(self.numParticles): #update velocities
            pf = self.calculatePressureForce(i)
            pa = pf/self.densities[i] #pressure acceleration
            vf = self.calculateViscosityForce(i)
            if self.gravityOn: #add pressure acceleration 
                self.velocities[i] = self.velocities[i] * self.velDamp + pa * self.deltaTime + vf * self.deltaTime
            else: #direct acceleration assignment + bodyforce
                self.velocities[i] = pa * self.deltaTime + self.bodyforce
            #print(self.velocities[i])

        for i in range(self.numParticles): #update positions and resolve collision
            newi = self.particleList[i] + self.velocities[i] * self.deltaTime
            if i == 1 and self.debug:
                print(f'testing to move from {self.particleList[i]} to {newi}')
            self.particleList[i] = self.resolveCollisions(newi,i,self.obstacleList)
            #self.particleList[i] = newi