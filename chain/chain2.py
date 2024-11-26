import sys
import os
sys.path.append(os.path.abspath('../sph/'))

import math
import numpy as np
from model import Model
import time

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
#FBD-based chain
class Chain2(Model):
    def __init__(self,id,startPos,plotSize,obstacleList = [],target_dist = 1.2,bond_factor = 0.4,
                 observation_id = -5,mass = 1,spring_constant=200,gravity=9.81,updateForceMode = 'updateForces',scanSurroundingsMode='scanSurroundings',collision_buffer=1.2):
        self.mass = mass
        self.spring_constant = spring_constant
        self.gravity = gravity * 2
        if updateForceMode == "updateForcesSquare":
            force_radius = 1.5* target_dist
        else:
            force_radius = 1.2* target_dist
        self.terminal_velocity = 5.0
        self.green_neighbours = []
        self.had_green_left = False
        movement_factor = ((force_radius-target_dist) * self.spring_constant)*0.9
        #movement_factor = ((self.force_radius) * self.spring_constant)*0.2
        #movement_factor = 0
        #self.gravity = 0.0
        self.lastknownvector = np.zeros(0)
        self.min_movement_fraction = 0.5
        self.modes=(scanSurroundingsMode,updateForceMode)
        collision_buffer = collision_buffer
        super().__init__(id,startPos,plotSize,collision_buffer,obstacleList,target_dist,movement_factor,bond_factor,
                 observation_id,force_radius=force_radius,deltaTime=0.02)
    
    def calculate_FBD(self,forceArr):
        force = np.zeros(3)
        for i in self.neighbours.keys():
            neighbour_force = forceArr[self.id][i]
            if abs(neighbour_force.any())> 0.01:
                force += neighbour_force
        return force
    
    def resolveSpringForce(self,predictedPos,spring_force): 
        #spring forces are not decomposed but projected with same magnitude in free direction
        for i in self.obstacleList:
            precollisionx = self.position[0]>i[0][0] and self.position[0]<i[1][0]
            precollisiony = self.position[1]<i[0][1] and self.position[1]>i[3][1]
            collisionx = predictedPos[0]>i[0][0] and predictedPos[0]<i[1][0]
            collisiony = predictedPos[1]<i[0][1] and predictedPos[1]>i[3][1]
            if collisionx and collisiony:
                #inside rectangle
                if not precollisionx and precollisiony:
                #collide side of obstacle
                    spring_force[1] = np.linalg.norm(spring_force) * np.sign(spring_force[1])
                    spring_force[0] = 0.0
                if precollisionx and not precollisiony:
                #collide top/bottom of obstacle
                    spring_force[0] = np.linalg.norm(spring_force) * np.sign(spring_force[0])
                    spring_force[1] = 0.0
        return spring_force
    
    def move(self,spring_force):
        damping_coefficient = 2*math.sqrt(self.mass * self.spring_constant) #critical damping
        gravity_force = np.array([0.0, self.gravity*self.mass, 0.0])
        total_force = spring_force + gravity_force - damping_coefficient * self.velocity
        left_right_neighbours = self.hasLeftRightNeighbours()
        scaled_movement_factor = self.movement_factor

        #state machine here
        #if have both neighbors
        if left_right_neighbours[0] and left_right_neighbours[1]: 
            self.state = 0
        else:
            #exclude special case of only one blue neighbour, or no neighbours, or brown state
            if not((len(self.neighbours) == 1 and list(self.neighbours.values())[0][0] == 1) \
                or len(self.neighbours) == 0 or self.state == 2): 
                
                if not left_right_neighbours[0] and left_right_neighbours[1]: #only right neighbour
                    self.had_green_left = False #if had left neighbour in front when green
                    for i in self.green_neighbours:
                        if i[1] < self.position[0] and i[2]>self.position[1]:
                            lastknownvector = np.array(i[1:])-self.position
                            self.lastknownvector = lastknownvector/np.linalg.norm(lastknownvector)
                            self.had_green_left = True
                            #print(f'{self.id}: updating lastknown vector {self.lastknownvector}')
                            break
                    if self.had_green_left:
                        self.state = 2
                        #total_force -= scaled_movement_factor * len(self.neighbours) * 3.5 * self.lastknownvector 
                    else:
                        self.state = 1
                        total_force[0] -= scaled_movement_factor * len(self.neighbours)

                        #scale according to how far from edge
                        #total_force[0] -= scaled_movement_factor * len(self.neighbours) * (1.0 + (self.position[0])/self.plotSize[0])

                        if not self.nearEdge(): #if not near edge, dont apply gravity
                            total_force -= gravity_force

                elif not left_right_neighbours[1] and left_right_neighbours[0]: #only left neighbour
                    self.had_green_right = False
                    for i in self.green_neighbours:
                        if i[1] > self.position[0] and i[2]>self.position[1]:
                            lastknownvector = np.array(i[1:])-self.position 
                            self.lastknownvector = lastknownvector/np.linalg.norm(lastknownvector)
                            self.had_green_right = True
                            #print(f'{self.id}: updating lastknown vector {self.lastknownvector}')
                            break
                    if self.had_green_right:
                        self.state = 2
                        #total_force += scaled_movement_factor * len(self.neighbours) * 1.5 * self.lastknownvector
                    else:
                        self.state = 1
                        total_force[0] += scaled_movement_factor * len(self.neighbours)

                        #scale according to how far from edge
                        #total_force[0] += scaled_movement_factor * len(self.neighbours) * (1.0 + (self.plotSize[0] - self.position[0])/self.plotSize[0])

                        if not self.nearEdge():
                            total_force -= gravity_force
                #else: do nothing
            else:
                #if brown, move towards last known neighbour vector
                if self.state == 2:
                    left_factor = 0.7
                    right_factor = 0.4
                    if self.had_green_left:
                        total_force += scaled_movement_factor * max(1,len(self.neighbours)) * left_factor * self.lastknownvector
                        # print(f'{self.id}: going left {scaled_movement_factor * min(1,len(self.neighbours)) * left_factor * self.lastknownvector}')
                        # print(f'{self.id}: going left {scaled_movement_factor} * {min(1,len(self.neighbours))} * left_factor * {self.lastknownvector}')
                    elif self.had_green_right:
                        total_force += scaled_movement_factor * max(1,len(self.neighbours)) * right_factor * self.lastknownvector
                        # print(f'{self.id}: going right {scaled_movement_factor * min(1,len(self.neighbours)) * right_factor * self.lastknownvector}')
                # elif self.state == 1 and list(self.neighbours.values())[0][0] != 1: #the case for no neighbour as a blue
                #     elif not left_right_neighbours[1] and left_right_neighbours[0]: #only left neighbour
                #     total_force[0] += scaled_movement_factor * len(self.neighbours)

                else:
                    self.state = 0



                #else: continue falling if no left and right  neighbours
        self.acceleration = total_force/self.mass
        self.velocity = np.minimum(self.velocity + self.acceleration * self.deltaTime, np.full(3,self.terminal_velocity))
        
        
        predictedPos =  self.position + self.velocity * self.deltaTime
        self.position = self.resolveCollisions(predictedPos)
        #self.position = self.resolveCollisionswithMag(predictedPos)
        
        #end of move if green, record down neighbour positions
        if self.state == 0:
            self.green_neighbours = []
            neighbour_values = list(self.neighbours.values())
            for i in neighbour_values:
                self.green_neighbours.append(i)

    def updateForces(self,forceArr):
        for i in self.neighbours.keys():
            neighbourpos = np.array(self.neighbours[i][1:])
            direction = neighbourpos - self.position
            distance = np.linalg.norm(direction)
            if distance > 0:
                unit_direction = direction / distance
                force = unit_direction * (distance - self.target_dist) * self.spring_constant
                forceArr[self.id][i] = force
                forceArr[i][self.id] = -force
            
    def updateForcesSquare(self,forceArr): #attempt at making a square formation
        for i in self.neighbours.keys():
            neighbourpos = np.array(self.neighbours[i][1:])
            direction = neighbourpos - self.position
            distance = np.linalg.norm(direction)
            real_target_dist = self.target_dist if i%2!=self.id%2 else self.target_dist * math.sqrt(2)
            if distance > 0:
                unit_direction = direction / distance
                if distance < real_target_dist:
                    force = unit_direction * (distance - real_target_dist) * self.spring_constant * 3
                else:
                    force = unit_direction * (distance - real_target_dist) * self.spring_constant
                forceArr[self.id][i] = force
                forceArr[i][self.id] = -force
    
    def updateForcesString(self,forceArr):
        for i in self.neighbours.keys():
            neighbourpos = np.array(self.neighbours[i][1:])
            direction = neighbourpos - self.position
            distance = np.linalg.norm(direction)
            if distance > 0:
                unit_direction = direction / distance
                if distance > self.target_dist:
                    force = unit_direction * (distance - self.target_dist) * self.spring_constant
                    forceArr[self.id][i] = force
                    forceArr[i][self.id] = -force
                elif distance < self.target_dist/3:
                    force = unit_direction * (distance - self.target_dist/3) * self.spring_constant
                    forceArr[self.id][i] = force
                    forceArr[i][self.id] = -force

    def step(self,forceArr,globalComms):
        ###Select scanning method##########################################
        if self.modes[0]=='scanSurroundings':
            self.scanSurroundings(globalComms)
        elif self.modes[0]=='scanSurroundingsOccluded':
            self.scanSurroundingsOccluded(globalComms)
        elif self.modes[0]=='scanSurroundingsDynamic':
            self.scanSurroundingsDynamic(globalComms,1.0-(self.id)/600)
        elif self.modes[0]=='scanSurroundingsOccludedDynamic':
            self.scanSurroundingsOccludedDynamic(globalComms,1.0-(self.id)/600)
        elif self.modes[0]=='scanSurroundingsOccludedDynamicString':
            self.scanSurroundingsOccludedDynamicString(globalComms,1.0-(self.id)/600)
        else:
            print("Wrong scan surroundings mode!")
        ###################################################################

        ###Calculate spring forces
        spring_force = self.calculate_FBD(forceArr)
        ###################################################################

        ###Move
        self.move(spring_force)
        ###################################################################

        ###Update forces based on new position  
        if self.modes[1]=='updateForces':
            self.updateForces(forceArr)
        elif self.modes[1]=='updateForcesSquare':
            self.updateForcesSquare(forceArr)
        elif self.modes[1]=='updateForcesString':
            self.updateForcesString(forceArr)
        else:
            print("Wrong update force mode!")
        ###################################################################
        return        
   
