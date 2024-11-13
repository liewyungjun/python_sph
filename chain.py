import math
import numpy as np

target_dist = 1.2
#force_radius = 2.2
force_radius = target_dist + 0.2
movement_factor = 0.05
bond_factor = 0.4
observation_id = -4


class Chain:
    def __init__(self,id,startPos,plotSize,obstacleList = []):
        self.neighbours = {} #dict of {id:[state,posx,posy,posz]}
        self.velocity = np.array([0.0,0.0,0.0])
        self.position = np.array(startPos)
        self.acceleration = np.zeros(3)
        self.id = id
        self.plotSize = plotSize
        self.obstacleList = obstacleList
        self.bondForce = np.array([0.0,0.0,0.0])
        self.movementForce = np.array([0.0,0.0,0.0])
        self.gravityOn = False
        self.collisionDamping = 0.8
        self.deltaTime = 0.1
        self.state = 0
    
    def calculateForce(self): #calculate bondforces based on neighbours
        totalforce = np.array([0.0,0.0,0.0])
        maxdelta = np.zeros(3)
        for i in self.neighbours:
            ##print(f'key is {i}')
            ##print(self.neighbours[i])
            deltax = self.position[0] - self.neighbours[i][1]
            deltay = self.position[1] - self.neighbours[i][2]
            deltaz = self.position[2] - self.neighbours[i][3]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            dist = max(0.01,dist)
            deltaforce = np.array([deltax/dist,deltay/dist,deltaz/dist])
            maxdelta = np.maximum(np.abs(maxdelta),np.abs(deltaforce))
            if dist > target_dist + 0.1:
                totalforce -= deltaforce
            if dist < target_dist - 0.1:
                totalforce += deltaforce
        self.bondForce = totalforce
        return 
    
    def nearEdge(self):
        margin = 0.1
        if abs(self.position[0] - 0.0) < margin or abs(self.position[0] - self.plotSize[0]) < margin:
            return True
        return False
    
    def removeNeighbour(self,neighbour):
        if self.neighbours.get(neighbour[0]):
            self.neighbours.pop(neighbour[0])
        return
    
    def addNeighbour(self,neighbour):
        self.neighbours[neighbour[0]] = neighbour[1:]
        return

    def checkNeighbours(self,predictedPos): #check if next pos will have any neighbours
        for neighbour in self.neighbours.values():
            deltax = predictedPos[0] - neighbour[1]
            deltay = predictedPos[1] - neighbour[2]
            deltaz = predictedPos[2] - neighbour[3]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < force_radius - 1.5:
                return True
        return False
        
    def hasLeftRightNeighbours(self):
        has_left_neighbour = False
        has_right_neighbour = False
        for neighbour in self.neighbours.values():
            deltax = self.position[0] - neighbour[1]
            deltay = self.position[1] - neighbour[2]
            deltaz = self.position[2] - neighbour[3]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < force_radius - 1.5:
                if deltax < 0.0:
                    has_left_neighbour = True
                if deltax > 0.0:
                    has_right_neighbour = True
        return has_left_neighbour,has_right_neighbour
    
    def scanSurroundings(self,globalComms):
        #globalComms is an array [id,state,posx,posy,poz]
        for i in globalComms:
            if i[0] == self.id:
                continue
            deltax = self.position[0] - i[2]
            deltay = self.position[1] - i[3]
            deltaz = self.position[2] - i[4]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < force_radius:
                self.addNeighbour(i)
            if dist > force_radius:
                self.removeNeighbour(i)
        return
    
    def resolveCollisions(self,predictedPos):
        # Check for collision with side walls
        if predictedPos[0] > self.plotSize[0] or predictedPos[0]<0.0:
            predictedPos[0] = self.position[0]
        # Check for collision with ground and top
        if (predictedPos[1]) < 0.0 or predictedPos[1] > self.plotSize[1]:
            predictedPos[1] = self.position[1]
        for i in self.obstacleList:
            precollisionx = self.position[0]>i[0][0] and self.position[0]<i[1][0]
            precollisiony = self.position[1]<i[0][1] and self.position[1]>i[3][1]
            collisionx = predictedPos[0]>i[0][0] and predictedPos[0]<i[1][0]
            collisiony = predictedPos[1]<i[0][1] and predictedPos[1]>i[3][1]
            if collisionx and collisiony:
                #inside rectangle
                if not precollisionx and precollisiony:
                #if collisionx:
                    if self.gravityOn:
                        self.velocity = (-self.velocity[0]* self.collisionDamping,self.velocity[1]) 
                        predictedPos[0] = self.position[0] + self.velocity[0] * self.deltaTime
                    else:
                        predictedPos[0] = self.position[0]
                if precollisionx and not precollisiony:
                #if collisiony:
                    if self.gravityOn:
                        self.velocity = (self.velocity[0],-self.velocity[1] * self.collisionDamping)        
                        predictedPos[1] = self.position[1] + self.velocity[1] * self.deltaTime
                    else:
                        predictedPos[1] = self.position[1]
        return np.array([predictedPos[0],predictedPos[1],0])
        
    
    def sendComms(self):
        return [self.id,self.state,self.position[0],self.position[1],self.position[2]]
    
    def step(self):
        
        self.calculateForce() #calculate bond forces
        self.velocity = self.movementForce * movement_factor + self.bondForce * bond_factor

        if self.id == observation_id:
            print(f'{self.id} velocity is {self.velocity} = {self.movementForce} + {self.bondForce}')
        if self.id == observation_id:
            print(f'at {self.position}')
        if self.id == observation_id:
            print(f'try to move to {self.position + self.velocity * self.deltaTime}')
        
        #self.position = self.resolveCollisions(self.position + self.velocity * self.deltaTime + np.array([0.0,0.01,0.0])) #predict pos and resolve collision
        self.position = self.resolveCollisions(self.position + self.velocity * self.deltaTime) #predict pos and resolve collision
        
        if self.id == observation_id:
            print(f'moved to {self.position}')
        return