import math 
from matplotlib.patches import Polygon
import numpy as np
import random
import time

class SPH_Decentralised_Parent:
    def __init__(self,particleList,obstacleList,numParticles,plotSize,plotFloor,\
                ratio,debug,floodRising,gravityOn,pressureMultiplier,targetDensity,\
                smoothingRadius, collisionDamping,mass,gravity,deltaTime,velDamp,\
                bodyforce,plotFloorSpeed,viscosityStrength):
        self.particleList = particleList
        self.numParticles = numParticles
        self.obstacleList = obstacleList
        self.obstacleEdgeNormals = self.calculateEdgeNormals()
        self.gravityOn = gravityOn
        self.floodRising = floodRising

        #physics params
        self.pressureMultiplier = pressureMultiplier
        self.targetDensity = targetDensity
        self.smoothingRadius = smoothingRadius
        self.collisionDamping = collisionDamping
        self.mass = [mass for i in range(numParticles)]
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
        self.velocities = [(0.0,0.0) for x in range(self.numParticles)]
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
        bottom = True
        for i in range(self.numParticles): #except youtself
            dist = math.dist(self.predictedPositions[point],self.predictedPositions[i])
            if self.predictedPositions[point][1] > self.predictedPositions[i][1]:
                bottom = False
            influence = self.smoothingKernel(self.smoothingRadius,dist)
            density += self.mass[i] * influence
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
                diry = -1 if random.randint(0,1) == 0 else 1
            else:
                dirx = (self.particleList[i][0] - self.particleList[particleIndex][0])/dist
                diry = (self.particleList[i][1] - self.particleList[particleIndex][1])/dist
            slope = self.smoothingKernelDerivative(self.smoothingRadius,dist)
            density = self.densities[i]
            sharedPressure = self.calculateSharedPressure(density,self.densities[particleIndex])
            pressureForce = (pressureForce[0] + \
                             sharedPressure * dirx * slope * self.mass[i] / density, \
                             pressureForce[1] + \
                             sharedPressure * diry * slope * self.mass[i] / density)
        return pressureForce
    
    def calculateEdgeNormals(self):
        res = []
        for obstacle in self.obstacleList:
            edgenormals = []
            for vertices in range(len(obstacle)):
                if vertices != len(obstacle)-1:
                    length = math.sqrt(((obstacle[vertices+1][1] - obstacle[vertices][1])*(obstacle[vertices+1][1] - obstacle[vertices][1]))+((obstacle[vertices+1][0] - obstacle[vertices][0])*(obstacle[vertices+1][0] - obstacle[vertices][0])))
                    edgenormals.append((-(obstacle[vertices+1][1] - obstacle[vertices][1])/length,(obstacle[vertices+1][0] - obstacle[vertices][0])/length))
                else:
                    length = math.sqrt(((obstacle[0][1] - obstacle[vertices][1])*(obstacle[0][1] - obstacle[vertices][1]))+((obstacle[0][0] - obstacle[vertices][0])*(obstacle[0][0] - obstacle[vertices][0])))
                    edgenormals.append((-(obstacle[0][1] - obstacle[vertices][1])/length,(obstacle[0][0] - obstacle[vertices][0])/length))
            res.append(edgenormals)
        return res
    
    def detectCollision(self,point):
        res = []
        for i in range(len(self.obstacleEdgeNormals)): #for each obstacle
            collision = True
            for j in range(len(self.obstacleEdgeNormals[i])): #for each obstacle normal
                mindot = float('inf')
                maxdot = float('-inf')
                pointdot = np.dot(point,self.obstacleEdgeNormals[i][j])
                for k in range(len(self.obstacleList[i])): #for each vertice
                    edgedot = np.dot(self.obstacleList[i][k],self.obstacleEdgeNormals[i][j])
                    if maxdot<edgedot:
                        maxdot = edgedot
                    if mindot > edgedot:
                        mindot = edgedot
                edgeAnalyse = j+1 if j != len(self.obstacleEdgeNormals[i])-1 else 0
                if not(pointdot<maxdot and pointdot>mindot):
                    collision = False
                    break
            if collision:
                res.append(i)
        return res

    def findCollisionEdge(self,obstacleIdxs,particleIdx):
        min_distance = 999
        nearest_edges = []
        nearest_edge = -1
        for obstacleIdx in obstacleIdxs: #for each obstacle
            min_distance = 999
            for j in range(len(self.obstacleList[obstacleIdx])): #for each obstacle edge
                edge1 = self.obstacleList[obstacleIdx][j]
                if j == len(self.obstacleList[obstacleIdx])-1:
                    edge2 = self.obstacleList[obstacleIdx][0]
                else:
                    edge2 = self.obstacleList[obstacleIdx][j+1]
                parallelogram_area = abs((edge2[0]-edge1[0]) * (self.particleList[particleIdx][1]-edge1[1]) - (edge2[1]-edge1[1]) * (self.particleList[particleIdx][0]-edge1[0]))
                base = math.sqrt((edge1[0]-edge2[0])*(edge1[0]-edge2[0])+(edge1[1]-edge2[1])*(edge1[1]-edge2[1]))
                distance = parallelogram_area/base
                if distance < min_distance:
                    min_distance = distance
                    nearest_edge = j
            nearest_edges.append((obstacleIdx,nearest_edge))
        return nearest_edges
    
    def resolveCollisions(self,pos,posIdx,obstacles):
        # Check for collision with side walls
        if pos[0] > self.plotSize * self.ratio[0] or pos[0]<0.0:
            if self.gravityOn:
                self.velocities[posIdx] = (-self.velocities[posIdx][0]* self.collisionDamping,self.velocities[posIdx][1]) 
            else:
                self.velocities[posIdx] = (0.0,self.velocities[posIdx][1])                 
        # Check for collision with ground and top
        if (pos[1]) < self.plotFloor: #if going down below floor
            #print("below floor")
            if self.gravityOn:
                if self.velocities[posIdx][1]<0: #reflect if going down
                    self.velocities[posIdx] = (self.velocities[posIdx][0],-self.velocities[posIdx][1] * self.collisionDamping)        
            else:
                if self.velocities[posIdx][1]<0: #stop if going down
                    self.velocities[posIdx] = (self.velocities[posIdx][0],0.0)  
            if self.floodRising and pos[1] < self.plotFloor and self.particleList[posIdx][1] < self.plotFloor: 
                #if already below flood level
                posdist = self.plotFloor - pos[1]
                veldist = min(2, max(1, posdist))
                self.velocities[posIdx] = (self.velocities[posIdx][0],self.plotFloorSpeed * veldist/self.deltaTime)   
        pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
        pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime

        collisionIdxs = self.detectCollision(pos)
        if collisionIdxs:
            collisionEdges = self.findCollisionEdge(collisionIdxs,posIdx)[0]
            collisionEdgeNormal = self.obstacleEdgeNormals[collisionIdxs[0]][collisionEdges[1]]
            collisionEdgeParallel = (-collisionEdgeNormal[1],collisionEdgeNormal[0])
            # Project velocity onto normal and parallel components
            v_normal = (self.velocities[posIdx][0] * collisionEdgeNormal[0] + self.velocities[posIdx][1] * collisionEdgeNormal[1])
            v_parallel = (self.velocities[posIdx][0] * collisionEdgeParallel[0] + self.velocities[posIdx][1] * collisionEdgeParallel[1])
            # Update velocity based on collision response
            if self.gravityOn:
                # Reflect normal component with damping, preserve parallel component
                new_v_normal = -v_normal * self.collisionDamping
                self.velocities[posIdx] = (new_v_normal * collisionEdgeNormal[0] + v_parallel * collisionEdgeParallel[0],
                                            new_v_normal * collisionEdgeNormal[1] + v_parallel * collisionEdgeParallel[1])
            else:
                # Zero out normal component, preserve parallel component
                self.velocities[posIdx] = (v_parallel * collisionEdgeParallel[0],
                                            v_parallel * collisionEdgeParallel[1])
            # Update position
            pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
            pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
        return (pos[0],pos[1])
    
    def viscositySmoothingKernel(self,radius,dist):
        if dist > radius:
            return 0
        volume = math.pi * pow(radius,8)/4
        rd = radius * radius - dist * dist
        return (rd * rd * rd)/ volume
    
    def calculateViscosityForce(self,particleIndex):
        viscosityForce = (0.0,0.0)
        position = self.particleList[particleIndex]
        for i in range(self.numParticles): #except youtself
            dist = math.dist(self.particleList[i],self.particleList[particleIndex])
            influence = self.viscositySmoothingKernel(self.smoothingRadius,dist)
            temp = ((self.velocities[i][0] - self.velocities[particleIndex][0]) * influence,(self.velocities[i][1] - self.velocities[particleIndex][1]) * influence)
            viscosityForce = (viscosityForce[0] + temp[0],viscosityForce[1] + temp[1])
        return (viscosityForce[0] * self.viscosityStrength,viscosityForce[1] * self.viscosityStrength)
    
    def step(self):

        for i in range(self.numParticles):#add gravity
            self.velocities[i] = (self.velocities[i][0], self.velocities[i][1] -self.gravity * self.deltaTime )
            self.predictedPositions[i] = (self.particleList[i][0] + self.velocities[i][0] * self.deltaTime, self.particleList[i][1] + self.velocities[i][1] * self.deltaTime)
        
        self.updateDensities() #update densities
        
        for i in range(self.numParticles): #update velocities
            pf = self.calculatePressureForce(i)
            pa = (pf[0]/self.densities[i],pf[1]/self.densities[i]) #pressure acceleration
            vf = self.calculateViscosityForce(i)
            if self.gravityOn: #add pressure acceleration
                self.velocities[i] = (self.velDamp * self.velocities[i][0] + pa[0] * self.deltaTime + vf[0] * self.deltaTime,\
                                      self.velDamp * self.velocities[i][1] + pa[1] * self.deltaTime + vf[1] * self.deltaTime)
            else: #direct acceleration assignment + bodyforce
                self.velocities[i] = (pa[0] * self.deltaTime + self.bodyforce[0], pa[1] * self.deltaTime + self.bodyforce[1])
            #print(self.velocities[i])

        for i in range(self.numParticles): #update positions and resolve collision
            newi =[self.particleList[i][0] + self.velocities[i][0] * self.deltaTime, self.particleList[i][1] + self.velocities[i][1] * self.deltaTime]
            self.particleList[i] = self.resolveCollisions(newi,i,self.obstacleList)
