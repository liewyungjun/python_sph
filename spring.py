import math
import numpy as np

comms_radius = 3.0
target_separation=0.8
repulsion_distance=0.3
debug_id = -1

class Spring:
    def __init__(self,id,startPos,plotSize,obstacleList = []):
        self.id = id
        self.position = startPos
        self.plotSize = plotSize
        self.obstacleList = obstacleList
        self.velocity = np.zeros(3)
        self.neighbours = self.getNeighbourIDs()
        self.state = 0

    def getNeighbourIDs(self):
        if self.id == 0:
            return [1]
        else:
            return [self.id - 1, self.id + 1]

    #TODO: add comms limit
    #TODO: add stuck state

    def balanceNeighbours(self,global_comms):
        balance_force = np.zeros(3)
        for i in self.neighbours:
            targetdeltas = np.array([-target_separation * (i - self.id),0.0,0.0])
            if i >= len(global_comms):
                continue
            deltas = self.position - global_comms[i][:-1]
            magnitude = np.absolute(targetdeltas - deltas)
            sign = np.sign([targetdeltas[j]-deltas[j] for j in range(len(deltas))])
            resultant = magnitude * sign
            balance_force += resultant
        for j in global_comms:
            if j is None or j is global_comms[self.id]:
                continue
            distance = np.linalg.norm(self.position - j[:-1])
            if distance < repulsion_distance:  # Minimum safe distance
                repulsion = (self.position - j[:-1])*0.4 # Add small offset to avoid division by zero
                balance_force += repulsion

        return balance_force
    
    def sendComms(self,global_comms):
        global_comms[self.id] = np.array([self.position[0],self.position[1],self.position[2],self.state])

    #TODO: if collide, project vectors to parallel breakage
    def resolveCollisions(self,predictedPos,global_comms):
        # Check for collision with side walls
        if predictedPos[0] > self.plotSize[0] or predictedPos[0]<0.0:
            predictedPos[0] = self.position[0]
        # Check for collision with ground and top
        if (predictedPos[1]) < 0.0 or predictedPos[1] > self.plotSize[1]:
            predictedPos[1] = self.position[1]
        collided = False
        for i in self.obstacleList:
            precollisionx = self.position[0]>i[0][0] and self.position[0]<i[1][0]
            precollisiony = self.position[1]<i[0][1] and self.position[1]>i[3][1]
            #print(f'precolision: x={precollisionx} y={precollisiony}')
            collisionx = predictedPos[0]>i[0][0] and predictedPos[0]<i[1][0]
            collisiony = predictedPos[1]<i[0][1] and predictedPos[1]>i[3][1]
            if self.id == debug_id:
                print(f'colision: x={collisionx} y={collisiony}')
                print(f'collisiony {predictedPos[1]}<{i[0][1]} {predictedPos[1]}>{i[3][1]}')
            if collisionx and collisiony:
                collided = True
                #TODO: differentiate by state
                #if im being pulled by a free drone, and held back by a stuck drone, break up with stuck drone
                # Check neighbor heights and move towards higher one
                left_neighbor = self.id - 1 if self.id > 0 else None
                right_neighbor = self.id + 1 if self.id < len(global_comms)-1 else None
                
                left_height = global_comms[left_neighbor][1] if left_neighbor is not None and left_neighbor < len(global_comms) else -float('inf')
                right_height = global_comms[right_neighbor][1] if right_neighbor is not None and right_neighbor < len(global_comms) else -float('inf')
                
                #self.neighbours = []
                #inside rectangle
                if not precollisionx and precollisiony: 
                    predictedPos[0] = self.position[0]
                    predictedPos[1] +=0.01
                    min_distance = 0.1  # Minimum allowed distance between drones
                    for i, drone_pos in enumerate(global_comms):
                        if i != self.id:  # Don't check distance to self
                            distance = np.sqrt((predictedPos[0] - drone_pos[0])**2 + (predictedPos[1] - drone_pos[1])**2)
                            if distance < min_distance:
                                # If too close, maintain current position
                                predictedPos[0] = self.position[0]
                                predictedPos[1] = self.position[1]
                                break
                elif precollisionx and not precollisiony:
                    if self.id == debug_id:
                        print("collision top")
                    predictedPos[1] = self.position[1]
                    if left_height > right_height:
                        if self.id == debug_id:
                            print("left higher")
                        predictedPos[0] = -0.01 + self.position[0]  # Move right
                        if len(self.neighbours)==2:
                            self.neighbours = [self.neighbours[0]]
                    else:
                        if self.id == debug_id:
                            print("right higher")
                        predictedPos[0] = 0.01 + self.position[0]  # Move right
                        print(self.neighbours)
                        if len(self.neighbours)==2:
                            self.neighbours = [self.neighbours[1]]
                    #predictedPos[0] +=0.01
                else:
                    predictedPos[0] -=predictedPos[0] - self.position[0]
        if collided:
            self.state = 1
        else:
            self.state = 0
            self.neighbours = self.getNeighbourIDs()
        return np.array([predictedPos[0],predictedPos[1],0])

    def step(self,global_comms):
        self.position =self.resolveCollisions(self.position +  self.balanceNeighbours(global_comms) * 0.1 + np.array([0.0,0.01,0.0]),global_comms) 
