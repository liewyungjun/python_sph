import sys
import os
sys.path.append(os.path.abspath('../sph/'))
print(sys.path)

import math
import numpy as np
from model import Model
import time

np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
#FBD-based chain
class Chain2(Model):
    def __init__(self,id,startPos,plotSize,obstacleList = [],target_dist = 1.2,bond_factor = 0.4,
                 observation_id = -5,mass = 1,spring_constant=200,gravity=9.81):
        self.mass = mass
        self.spring_constant = spring_constant
        self.gravity = gravity * 1.5
        force_radius = 1.2* target_dist
        self.terminal_velocity = 5.0
        self.green_neighbours = []
        self.had_green_left = False
        movement_factor = ((force_radius-target_dist) * self.spring_constant)*0.95
        #movement_factor = ((self.force_radius) * self.spring_constant)*0.2
        #movement_factor = 0
        #self.gravity = 0.0
        self.min_movement_fraction = 0.5
        super().__init__(id,startPos,plotSize,obstacleList,target_dist,movement_factor,bond_factor,
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

        #if have both neighbors
        if left_right_neighbours[0] and left_right_neighbours[1]: 
            self.state = 0
        else:
            if not((len(self.neighbours) == 1 and list(self.neighbours.values())[0][0] == 1) or len(self.neighbours) == 0 or self.state == 2) : #exclude special case of only one or no neighbours
                scaled_movement_factor = self.movement_factor
                if not left_right_neighbours[0] and left_right_neighbours[1]: #only right neighbour
                    self.had_green_left = False
                    for i in self.green_neighbours:
                        if i[1] < self.position[0] and i[2]>self.position[1]:
                            lastknownvector = np.array(i[1:])
                            lastknownvector = lastknownvector/np.linalg.norm(lastknownvector)
                            self.had_green_left = True
                            break
                    if self.had_green_left:
                        self.state = 2
                        total_force -= scaled_movement_factor * len(self.neighbours) * 3.5 * lastknownvector + gravity_force
                        print("red left")
                    else:
                        self.state = 1
                        total_force[0] -= scaled_movement_factor * len(self.neighbours)
                        if not self.nearEdge():
                            total_force -= gravity_force
                elif not left_right_neighbours[1] and left_right_neighbours[0]: #only left neighbour
                    self.had_green_right = False
                    for i in self.green_neighbours:
                        if i[1] > self.position[0] and i[2]>self.position[1]:
                            lastknownvector = np.array(i[1:])
                            self.had_green_right = True
                            break
                    if self.had_green_right:
                        self.state = 2
                        total_force += scaled_movement_factor * len(self.neighbours) * 1.5 * lastknownvector  + gravity_force
                    else:
                        self.state = 1
                        total_force[0] += scaled_movement_factor * len(self.neighbours)
                        if not self.nearEdge():
                            total_force -= gravity_force
                if self.state == 2:
                    if self.had_green_left:
                        total_force -= scaled_movement_factor * len(self.neighbours) * 3.5 * lastknownvector + gravity_force
                    elif self.had_green_right:
                        total_force += scaled_movement_factor * len(self.neighbours) * 1.5 * lastknownvector + gravity_force
                #else: continue falling if no left and right  neighbours
        self.acceleration = total_force/self.mass
        self.velocity = np.minimum(self.velocity + self.acceleration * self.deltaTime, np.full(3,self.terminal_velocity))
        
        
        predictedPos =  self.position + self.velocity * self.deltaTime
        self.position = self.resolveCollisions(predictedPos)
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
        self.scanSurroundingsOccluded(globalComms)
        #self.scanSurroundings(globalComms)
        #self.scanSurroundingsDynamic(globalComms,1.0-(self.id)/600)
        ###################################################################

        ###Calculate spring forces
        spring_force = self.calculate_FBD(forceArr)
        ###################################################################

        ###Move
        self.move(spring_force)
        ###################################################################

        ###Update forces based on new position  
        self.updateForces(forceArr)
        #self.updateForcesSquare(forceArr)
        #self.updateForcesString(forceArr)
        ###################################################################
        return        
   
