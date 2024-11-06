import math
import numpy as np

force_radius = 3.0

class Slime:
    def __init__(self,id,startPos,plotSize):
        self.neighbours = {} #dict of {id:[state,pos]}
        self.state = 0
        self.velocity = np.array([0.0,0.0,0.0])
        self.position = np.array(startPos)
        self.id = id
        self.plotSize = plotSize
    
    def calculateForce(self):
        totalforce = np.array([0.0,0.0,0.0])
        for i in self.neighbours:
            #print(f'key is {i}')
            #print(self.neighbours[i])
            deltax = self.position[0] - self.neighbours[i][1]
            deltay = self.position[1] - self.neighbours[i][2]
            deltaz = self.position[2] - self.neighbours[i][3]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist > 2.0:
                totalforce[0] -= deltax/dist
                totalforce[1] -= deltay/dist
                totalforce[2] -= deltaz/dist
            if dist < 1.0:
                dist = max(0.01,dist)
                totalforce[0] += deltax/dist
                totalforce[1] += deltay/dist
                totalforce[2] += deltaz/dist
        self.velocity = totalforce * 0.1
        print(f'{self.id}: vel = {self.velocity}')
        return 
    
    def switchState(self):
        return
    
    def forwardMove(self):
        has_left_neighbour = False
        has_right_neighbour = False
        has_top_neighbour = False
        for neighbour in self.neighbours.values():
            if neighbour[0] < self.position[0] and abs(neighbour[1] - self.position[1]) < 2.0:  # Check x position
                has_left_neighbour = True
            if neighbour[0] > self.position[0] and abs(neighbour[1] - self.position[1]) < 2.0:  # Check x position
                has_right_neighbour = True
            if neighbour[1] > self.position[1]:
                has_top_neighbour = True
        if has_top_neighbour:
            self.velocity[1] += 0.1  # Move up by updating y coordinate
            return
        if not has_left_neighbour:
            if self.velocity[0] > 0.1:  # Move left by updating x coordinate
                self.velocity[0] = 0.0
            else:
                self.velocity[0] -= 0.1  # Move left by updating x coordinate
        else:
            if not has_right_neighbour:
                if self.velocity[0] < 0.1:  # Move left by updating x coordinate
                    self.velocity[0] = 0.0
                else:
                    self.velocity[0] += 0.1  # Move right by updating x coordinate
            else:
                self.velocity[1] += 0.1  # Move up by updating y coordinate
        return
    
    def removeNeighbour(self,neighbour):
        if self.neighbours.get(neighbour[0]):
            self.neighbours.pop(neighbour[0])
        return
    
    def addNeighbour(self,neighbour):
        self.neighbours[neighbour[0]] = neighbour[1:]
        return

    def scanSurroundings(self,globalComms):
        #globalComms is an array of id,state, and pos
        for i in globalComms:
            deltax = self.position[0] - i[2]
            deltay = self.position[1] - i[3]
            deltaz = self.position[2] - i[4]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < force_radius:
                self.addNeighbour(i)
            if dist > force_radius:
                self.removeNeighbour(i)
        return
    
    def resolveCollisions(self,pos,obstacles):
        # Check for collision with side walls
        if pos[0] > self.plotSize[0] or pos[0]<0.0:
            pos[0] = self.position[0]
        # Check for collision with ground and top
        if (pos[1]) < 0.0 or pos[1] > self.plotSize[1]:
            pos[1] = self.position[1]
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
        #                 self.velocities[posIdx] = (-self.velocities[posIdx][0]* self.collisionDamping,self.velocities[posIdx][1]) 
        #                 pos[0] = self.particleList[posIdx][0] + self.velocities[posIdx][0] * self.deltaTime
        #             else:
        #                 pos[0] = self.particleList[posIdx][0]
        #         if precollisionx and not precollisiony:
        #         #if collisiony:
        #             if self.gravityOn:
        #                 self.velocities[posIdx] = (self.velocities[posIdx][0],-self.velocities[posIdx][1] * self.collisionDamping)        
        #                 pos[1] = self.particleList[posIdx][1] + self.velocities[posIdx][1] * self.deltaTime
        #             else:
        #                 pos[1] = self.particleList[posIdx][1]
        return np.array([pos[0],pos[1],0])
        
    
    def sendComms(self):
        return [self.id,self.state,self.position[0],self.position[1],self.position[2]]
    
    def step(self):
        self.calculateForce()
        self.forwardMove()
        self.position = self.resolveCollisions(self.position + self.velocity,[])
        return