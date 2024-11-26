import math
import numpy as np



#FBD-based chain
class Model:
    def __init__(self,id,startPos,plotSize,collision_buffer, obstacleList = [],target_dist = 1.2,movement_factor = 0.05,bond_factor = 0.4,
                 observation_id = -4,force_radius = 1.4,deltaTime = 0.02):
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
        self.deltaTime = deltaTime
        self.state = 0
        self.target_dist = target_dist
        self.force_radius = force_radius
        self.movement_factor = movement_factor
        self.bond_factor = bond_factor
        self.observation_id = observation_id
        self.collision_buffer = collision_buffer
    
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
            if dist > self.target_dist + 0.1:
                totalforce -= deltaforce
            if dist <self.target_dist - 0.1:
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
            if dist < self.force_radius - 1.5:
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
            if dist < self.force_radius:
                if deltax > 0.0:
                    has_left_neighbour = True
                if deltax < 0.0:
                    has_right_neighbour = True
        return has_left_neighbour,has_right_neighbour
    
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

    def scanSurroundingsOccluded(self,globalComms):
        #globalComms is an array [id,state,posx,posy,poz]
        for i in globalComms:
            if i[0] == self.id:
                continue
            deltax = self.position[0] - i[2]
            deltay = self.position[1] - i[3]
            deltaz = self.position[2] - i[4]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < self.force_radius:
                if self.checkOcclusion(i) or dist < 0.5:
                    self.addNeighbour(i)
                else:
                    self.removeNeighbour(i)    
            if dist > self.force_radius:
                self.removeNeighbour(i)
        return
    
    def scanSurroundings(self,globalComms):
        #globalComms is an array [id,state,posx,posy,poz]
        for i in globalComms:
            if i[0] == self.id:
                continue
            deltax = self.position[0] - i[2]
            deltay = self.position[1] - i[3]
            deltaz = self.position[2] - i[4]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < self.force_radius:
                self.addNeighbour(i)
            if dist > self.force_radius:
                self.removeNeighbour(i)
        return
    
    def scanSurroundingsDynamic(self,globalComms,radius_percentage):
        #alternative, scan surroundings will change the radius to introduce manufacturing defects
        #globalComms is an array [id,state,posx,posy,poz]
        for i in globalComms:
            if i[0] == self.id:
                continue
            deltax = self.position[0] - i[2]
            deltay = self.position[1] - i[3]
            deltaz = self.position[2] - i[4]
            dist = math.sqrt(deltax*deltax + deltay*deltay + deltaz*deltaz)
            if dist < self.force_radius * radius_percentage:
                self.addNeighbour(i)
            if dist > self.force_radius * radius_percentage:
                self.removeNeighbour(i)
        return
    
    def resolveCollisions(self,predictedPos):
        collision_buffer = self.collision_buffer
        #collision_buffer = 0.2
        #collision_buffer = 0
        # Check for collision with side walls
        if predictedPos[0] > self.plotSize[0] or predictedPos[0]<0.0:
            predictedPos[0] = self.position[0]
            self.velocity[0] = 0.0
        # Check for collision with ground and top
        if (predictedPos[1]) < 0.0 or predictedPos[1] > self.plotSize[1]:
            predictedPos[1] = self.position[1]
            self.velocity[1] = 0.0
        for i in self.obstacleList:
            precollisionx = self.position[0]>i[0][0]-collision_buffer and self.position[0]<i[1][0] + collision_buffer
            precollisiony = self.position[1]<i[0][1] +collision_buffer and self.position[1]>i[3][1] -collision_buffer
            collisionx = predictedPos[0]>i[0][0]-collision_buffer and predictedPos[0]<i[1][0] + collision_buffer
            collisiony = predictedPos[1]<i[0][1] +collision_buffer and predictedPos[1]>i[3][1] -collision_buffer
            if collisionx and collisiony:
                #inside rectangle
                if not precollisionx and precollisiony:
                #if collisionx:
                    if self.gravityOn:
                        self.velocity = (-self.velocity[0]* self.collisionDamping,self.velocity[1]) 
                        predictedPos[0] = self.position[0] + self.velocity[0] * self.deltaTime
                    else:
                        predictedPos[0] = self.position[0]
                        self.velocity[0] = 0.0
                if precollisionx and not precollisiony:
                #if collisiony:
                    if self.gravityOn:
                        self.velocity = (self.velocity[0],-self.velocity[1] * self.collisionDamping)        
                        predictedPos[1] = self.position[1] + self.velocity[1] * self.deltaTime
                    else:
                        predictedPos[1] = self.position[1]
                        self.velocity[1] = 0.0
        return np.array([predictedPos[0],predictedPos[1],0])
    
    def resolveCollisionswithMag(self,predictedPos):
        # Check for collision with side walls
        if predictedPos[0] > self.plotSize[0] or predictedPos[0]<0.0:
            predictedPos[0] = self.position[0]
            self.velocity[0] = 0.0
        # Check for collision with ground and top
        if (predictedPos[1]) < 0.0 or predictedPos[1] > self.plotSize[1]:
            predictedPos[1] = self.position[1]
            self.velocity[1] = 0.0
        for i in self.obstacleList:
            precollisionx = self.position[0]>i[0][0] and self.position[0]<i[1][0]
            precollisiony = self.position[1]<i[0][1] and self.position[1]>i[3][1]
            collisionx = predictedPos[0]>i[0][0] and predictedPos[0]<i[1][0]
            collisiony = predictedPos[1]<i[0][1] and predictedPos[1]>i[3][1]
            if collisionx and collisiony:
                vel_mag = np.linalg.norm(self.velocity)
                #inside rectangle
                if not precollisionx and precollisiony:
                #if collisionx:
                    if self.gravityOn:
                        self.velocity = (-self.velocity[0]* self.collisionDamping,self.velocity[1]) 
                        predictedPos[0] = self.position[0] + self.velocity[0] * self.deltaTime
                    else:
                        sign = np.sign(self.velocity[1])
                        self.velocity[1] = vel_mag
                        predictedPos[1] = self.position[1] + vel_mag * self.deltaTime * sign
                        predictedPos[0] = self.position[0]
                        self.velocity[0] = 0.0
                if precollisionx and not precollisiony:
                #if collisiony:
                    if self.gravityOn:
                        self.velocity = (self.velocity[0],-self.velocity[1] * self.collisionDamping)        
                        predictedPos[1] = self.position[1] + self.velocity[1] * self.deltaTime
                    else:
                        sign = np.sign(self.velocity[0])
                        self.velocity[0] = vel_mag
                        predictedPos[0] = self.position[0] + vel_mag * self.deltaTime * sign
                        predictedPos[1] = self.position[1]
                        self.velocity[1] = 0.0
        return np.array([predictedPos[0],predictedPos[1],0])
        
    def calculateEdgeNormals(self):
        res = []
        for obstacle in self.obstacleList:
            edgenormals = []
            for vertices in range(len(obstacle)):
                if vertices != len(obstacle)-1:
                    length = math.sqrt(((obstacle[vertices+1][1] - obstacle[vertices][1])*(obstacle[vertices+1][1] - obstacle[vertices][1]))+((obstacle[vertices+1][0] - obstacle[vertices][0])*(obstacle[vertices+1][0] - obstacle[vertices][0])))
                    edgenormals.append((-(obstacle[vertices+1][1] - obstacle[vertices][1])/length,(obstacle[vertices+1][0] - obstacle[vertices][0])/length))
                    #edgenormals.append(((obstacle[vertices+1][0] - obstacle[vertices][0]),(obstacle[vertices+1][1] - obstacle[vertices][1])))
                else:
                    length = math.sqrt(((obstacle[0][1] - obstacle[vertices][1])*(obstacle[0][1] - obstacle[vertices][1]))+((obstacle[0][0] - obstacle[vertices][0])*(obstacle[0][0] - obstacle[vertices][0])))
                    edgenormals.append((-(obstacle[0][1] - obstacle[vertices][1])/length,(obstacle[0][0] - obstacle[vertices][0])/length))
                    #edgenormals.append(((obstacle[0][0] - obstacle[vertices][0]),(obstacle[0][1] - obstacle[vertices][1])))
            res.append(edgenormals)
        #print(res)
        return res
    
    def detectCollision(self,edgenormals):
        res = []
        for i in range(len(edgenormals)): #for each obstacle
            collision = True
            #print(f'checking {len(edgenormals[i])} normals')
            for j in range(len(edgenormals[i])): #for each obstacle normal
                mindot = float('inf')
                maxdot = float('-inf')
                positiondot = np.dot(self.position,edgenormals[i][j])
                for k in range(len(self.obstacleList[i])): #for each vertice
                    edgedot = np.dot(self.obstacleList[i][k],edgenormals[i][j])
                    if maxdot<edgedot:
                        maxdot = edgedot
                    if mindot > edgedot:
                        mindot = edgedot
                edgeAnalyse = j+1 if j != len(edgenormals[i])-1 else 0
                # print(f'edgedot is {edgedot} for vertice {self.obstacleList[i][k]}')
                # print(f'obs {i} edge {j}:normal is {edgenormals[i][j]}')
                # print(f'obs {i} edge {j}:edge {self.obstacleList[i][j]} and {self.obstacleList[i][edgeAnalyse]} self.positiondot is {self.positiondot}, maxdot is {maxdot}, mindot is {mindot}')
                if not(positiondot<maxdot and positiondot>mindot):
                    #print(f'no collision between {self.position} and {self.obstacleList[i]}')
                    collision = False
                    break
            if collision:
                #print(f'COLLISION between {self.position} and {self.obstacleList[i]}')
                res.append(i)
        #print(res)
        return res

    def findCollisionEdge(self,obstacleIdxs):
        min_distance = float('inf')
        nearest_edges = []
        nearest_edge = -1
        for obstacleIdx in obstacleIdxs: #for each obstacle
            min_distance = float('inf')
            for j in range(len(self.obstacleList[obstacleIdx])): #for each obstacle edge
                edge1 = self.obstacleList[obstacleIdx][j]
                if j == len(self.obstacleList[obstacleIdx])-1:
                    edge2 = self.obstacleList[obstacleIdx][0]
                else:
                    edge2 = self.obstacleList[obstacleIdx][j+1]
                # print(edge1)
                # print(edge2)
                parallelogram_area = abs((edge2[0]-edge1[0]) * (self.position[1]-edge1[1]) - (edge2[1]-edge1[1]) * (self.position[0]-edge1[0]))
                base = math.sqrt((edge1[0]-edge2[0])*(edge1[0]-edge2[0])+(edge1[1]-edge2[1])*(edge1[1]-edge2[1]))
                distance = parallelogram_area/base
                #print(f'distance:{distance} edge:{j} between {edge1} and {edge2}')
                if distance < min_distance:
                    min_distance = distance
                    nearest_edge = j
            nearest_edges.append((obstacleIdx,nearest_edge))
        #print(f'nearest edges are {nearest_edges}')
        return nearest_edges
    
    def resolveCollisionsSAT(self,predictedPos):
        # Check for collision with side walls
        if predictedPos[0] > self.plotSize[0] or predictedPos[0]<0.0:
            predictedPos[0] = self.position[0]
        # Check for collision with ground and top
        if (predictedPos[1]) < 0.0 or predictedPos[1] > self.plotSize[1]:
            predictedPos[1] = self.position[1]

        collisionIdxs = self.detectCollision(predictedPos)
        if collisionIdxs:
            collisionEdges = self.findCollisionEdge(collisionIdxs)[0] #just taking the first collided edge for now
            collisionEdgeNormal = self.calculateEdgeNormals()[collisionIdxs[0]][collisionEdges[1]]
            collisionEdgeParallel = (-collisionEdgeNormal[1],collisionEdgeNormal[0])
            # Project velocity onto normal and parallel components
            v_normal = (self.velocity[0] * collisionEdgeNormal[0] + self.velocity[1] * collisionEdgeNormal[1])
            v_parallel = (self.velocity[0] * collisionEdgeParallel[0] + self.velocity[1] * collisionEdgeParallel[1])
            #print(f'particle {posIdx} ori velocity is {self.velocities[posIdx][0]:.4f},{self.velocities[posIdx][1]:.4f}')
            # Update velocity based on collision response
            if self.gravityOn: 
                # Reflect normal component with damping, preserve parallel component
                new_v_normal = -v_normal * self.collisionDamping
                self.velocity = (new_v_normal * collisionEdgeNormal[0] + v_parallel * collisionEdgeParallel[0],
                                            new_v_normal * collisionEdgeNormal[1] + v_parallel * collisionEdgeParallel[1])
            else:
                # Zero out normal component, preserve parallel component
                self.velocity = (v_parallel * collisionEdgeParallel[0],
                                            v_parallel * collisionEdgeParallel[1])
            #print(f'particle {posIdx} new velocity is {self.velocities[posIdx][0]:.4f},{self.velocities[posIdx][1]:.4f}')
            # Update position
            pos = self.position + self.velocity * self.deltaTime
        return pos
    
    def sendComms(self):
        return [self.id,self.state,self.position[0],self.position[1],self.position[2]]