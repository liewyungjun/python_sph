import math
import numpy as np
target_dist = 2.0
force_radius = target_dist + 0.2
movement_factor = 0.14
bond_factor = 0.1
observation_id = -4
side_movement = 1.0
break_margin = 0.5
forward_movement = 0.8

class Slime:
    #0:advancer 1:spreader 2:front-waiter 3:follower 4:wall-stuck
    STATE_ADVANCER = 0
    STATE_SPREADER = 1
    STATE_FRONT_WAITER = 2
    STATE_FOLLOWER = 3
    STATE_WALL_STUCK = 4
    STATE_DICT = {STATE_ADVANCER:"advancer",STATE_SPREADER:"spreader",STATE_FRONT_WAITER:"front-waiter",STATE_FOLLOWER:"follower",STATE_WALL_STUCK:"wall-stuck"}

    def __init__(self,id,startPos,plotSize,obstacleList = []):
        self.neighbours = {} #dict of {id:[state,posx,posy,posz]}
        
        self.state = self.STATE_ADVANCER
        self.velocity = np.array([0.0,0.0,0.0])
        self.position = np.array(startPos)
        self.id = id
        self.plotSize = plotSize
        self.obstacleList = obstacleList
        self.bondForce = np.array([0.0,0.0,0.0])
        self.movementForce = np.array([0.0,0.0,0.0])
        self.gravityOn = False
        self.collisionDamping = 0.8
        self.deltaTime = 0.2
    
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
                if self.neighbours[i][0] != self.STATE_FOLLOWER and self.neighbours[i][0] != self.STATE_WALL_STUCK:
                    totalforce -= deltaforce
                else:
                    if self.state == self.STATE_WALL_STUCK:
                        totalforce -= deltaforce

            if dist < target_dist +- 0.1:
                totalforce += deltaforce
        self.bondForce = totalforce
        return 
    
    def switchState(self,state):
        if state != self.state:
            #print(f'{self.id}: from {self.STATE_DICT[self.state]} to {self.STATE_DICT[state]}')
            self.state = state
        return
    
    def nearEdge(self):
        margin = 0.1
        if abs(self.position[0] - 0.0) < margin or abs(self.position[0] - self.plotSize[0]) < margin:
            return True
        return False
            
    
    def forwardMove(self):
        movementForce = np.array([0.0,0.0,0.0])
        has_left_neighbour = False
        has_right_neighbour = False
        has_top_neighbour = False
        has_wait_spreader_neighbour = False
        # Find shortest and longest left/right neighbours
        rightest_dist = float('inf')
        leftest_dist = -float('inf')
        rightest_id = -1
        leftest_id = -1
        for neighbour in self.neighbours:
            if self.neighbours[neighbour][1] < self.position[0] and abs(self.neighbours[neighbour][2] - self.position[1]) < 5.0:  
                has_left_neighbour = True
            if self.neighbours[neighbour][1] > self.position[0] and abs(self.neighbours[neighbour][2] - self.position[1]) < 5.0:  
                has_right_neighbour = True
            if self.neighbours[neighbour][2] > self.position[1] and abs(self.neighbours[neighbour][1] - self.position[0]) < 0.5:  
                #if self.id ==3:
                    #print(f'{self.id}: top neighbour is {neighbour}')
                has_top_neighbour = True
            if self.neighbours[neighbour][0] == self.STATE_ADVANCER or self.neighbours[neighbour][0] == self.STATE_SPREADER or self.neighbours[neighbour][0] == self.STATE_FRONT_WAITER:
                deltax =  self.neighbours[neighbour][1]  - self.position[0]
                if deltax > 0:
                    if deltax < rightest_dist:
                        rightest_dist = deltax
                        rightest_id = neighbour
                if deltax < 0:
                    if deltax > leftest_dist:
                        leftest_dist = deltax
                        leftest_id = neighbour
            if self.neighbours[neighbour][0] == self.STATE_FRONT_WAITER or self.neighbours[neighbour][0] == self.STATE_SPREADER:
                has_wait_spreader_neighbour = True
        if has_top_neighbour: # not a frontier particle
            self.switchState(self.STATE_FOLLOWER)
            self.movementForce = movementForce
            return                
        if not has_left_neighbour:
            if self.id == observation_id:
                print(f'{self.id}: no left neighbour')
            if self.nearEdge():
                    self.switchState(self.STATE_ADVANCER)
                    movementForce[1] = forward_movement
            else:
                self.switchState(self.STATE_SPREADER)
                movementForce = np.array([-side_movement,0.0,0.0])
        else:
            if not has_right_neighbour:
                if self.id == observation_id:
                    print(f'{self.id}: no right neighbour')
                if self.nearEdge():
                    self.switchState(self.STATE_ADVANCER)
                    movementForce[1] = forward_movement
                else:
                    self.switchState(self.STATE_SPREADER)
                    movementForce = np.array([side_movement,0.0,0.0])
            else: #has both neighbours
                self.switchState(self.STATE_ADVANCER)
                movementForce = np.array([0.0,forward_movement,0.0])
            
        # If we're in state 2 (advancer), adjust movement to balance distances
        if self.state == self.STATE_ADVANCER:
            if (leftest_id != -1 or rightest_id != -1) and leftest_id != rightest_id:
                target = (leftest_dist + rightest_dist)/2
                movementForce[0] += (target)*0.2
                
                if self.id == observation_id:
                    print(f'adjusting movement to be {movementForce}')
                    print(f'{self.id}: leftest id is {leftest_id}, rightest id is {rightest_id}')
                    print(f'{self.id}: leftest is {leftest_dist}, rightest is {rightest_dist}')
                    print(f'{self.id}: target is {target}, movementForce is {movementForce}')
        
        # if self.state == self.STATE_ADVANCER and has_wait_spreader_neighbour:
        #     if self.id == observation_id:
        #         print(f'has waiting neighbour')
        #     self.switchState(self.STATE_FRONT_WAITER)
        #     movementForce[1] = 0.0
        self.movementForce = movementForce
        return
    
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
            if dist < force_radius - break_margin:
                return True
        return False
        
    def lineIntersects(self, x1, y1, x2, y2, x3, y3, x4, y4):
            # Returns True if line segments (x1,y1)->(x2,y2) and (x3,y3)->(x4,y4) intersect
            denominator = ((x2 - x1) * (y4 - y3)) - ((y2 - y1) * (x4 - x3))
            if denominator == 0:
                return False
                
            ua = (((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3))) / denominator
            ub = (((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3))) / denominator
            
            return (ua >= 0 and ua <= 1) and (ub >= 0 and ub <= 1)
        
    def checkOcclusion(self, idx):
        
        # Get line between self and idx
        x1, y1 = self.position[0], self.position[1]
        x2, y2 = idx[2], idx[3]
        
        # Check intersection with each obstacle
        for obstacle in self.obstacleList:
            # Check each edge of the obstacle rectangle
            # Top edge
            if self.lineIntersects(x1, y1, x2, y2, 
                                    obstacle[0][0], obstacle[0][1], 
                                    obstacle[1][0], obstacle[1][1]):
                return False
            # Right edge
            if self.lineIntersects(x1, y1, x2, y2,
                                    obstacle[1][0], obstacle[1][1],
                                    obstacle[2][0], obstacle[2][1]):
                return False
            # Bottom edge
            if self.lineIntersects(x1, y1, x2, y2,
                                    obstacle[2][0], obstacle[2][1],
                                    obstacle[3][0], obstacle[3][1]):
                return False
            # Left edge
            if self.lineIntersects(x1, y1, x2, y2,
                                    obstacle[3][0], obstacle[3][1],
                                    obstacle[0][0], obstacle[0][1]):
                return False
        return True
    
        
        

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
                if self.checkOcclusion(i) or dist < 0.5:
                    self.addNeighbour(i)
                else:
                    self.removeNeighbour(i)    
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
                self.switchState(self.STATE_WALL_STUCK)
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
        self.forwardMove() #calculate movement forces
        if self.state != self.STATE_FOLLOWER and self.state!=self.STATE_FOLLOWER: # not follower or stuck
            predictedPos = self.position + self.movementForce * movement_factor * self.deltaTime
            if not self.checkNeighbours(predictedPos): #will not have any neighbours
                #self.movementForce = -self.bondForce
                #print("will not have neighbour")
                xx = 1
            else:
                #need to maintain right neighbour if going left and vice versa
                if self.state == self.STATE_SPREADER or self.state == self.STATE_ADVANCER:
                    self.bondForce[0] = 0.0
        
        self.velocity = self.movementForce * movement_factor + self.bondForce * bond_factor
        if self.id == observation_id:
            print(f'{self.id} velocity is {self.velocity} = {self.movementForce} + {self.bondForce}')
        if self.id == observation_id:
            print(f'at {self.position}')
        if self.id == observation_id:
            print(f'try to move to {self.position + self.velocity * self.deltaTime}')

        self.position = self.resolveCollisions(self.position + self.velocity * self.deltaTime) #predict pos and resolve collision

        if self.id == observation_id:
            print(f'moved to {self.position}')
        return