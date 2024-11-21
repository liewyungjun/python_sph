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
        #force = np.array([0.0, self.gravity*self.mass, 0.0])  # gravity in y direction
        force = np.zeros(3)
        for i in self.neighbours.keys():
            neighbour_force = forceArr[self.id][i]
            # if self.id == self.observation_id:
            #     print(f'reading spring force from {self.id} to {i} is {neighbour_force}')
            if abs(neighbour_force.any())> 0.01:
                force += neighbour_force
        # if self.id == self.observation_id:
        #     print(f'total spring force is {force}')
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
        ##Attempt at only dampening velocity along the normalised vector of spring force
        # norm = np.linalg.norm(spring_force)
        # if norm >0:
        #     unit_spring_vector = spring_force / norm
        #     damping_force = np.dot(self.velocity,unit_spring_vector) * unit_spring_vector * damping_coefficient
        # else:
        #     damping_force = np.zeros(3)
        # damping_force = np.zeros(3)
        total_force = spring_force + gravity_force - damping_coefficient * self.velocity
        #total_force = spring_force + gravity_force - damping_force
        if self.id == self.observation_id:
            print(f'total force {total_force} = spring force {spring_force} + grav - damp_coeff {damping_coefficient} * vel {self.velocity}')
        left_right_neighbours = self.hasLeftRightNeighbours()
        if self.id == self.observation_id:
                    print(f"Left/Right: {left_right_neighbours}")
        if left_right_neighbours[0] and left_right_neighbours[1]: #has both neighbors
            self.state = 0
            if self.id == self.observation_id:
                print("state is 0")
        else:
            if not((len(self.neighbours) == 1 and list(self.neighbours.values())[0][0] == 1) or len(self.neighbours) == 0 or self.state == 2) : #exclude special case of only one or no neighbours
                # scaled_movement_factor = (self.movement_factor - self.movement_factor * self.min_movement_fraction) * \
                #                         min(self.position[0] - 0.0,self.plotSize[0] - self.position[0])/(self.plotSize[0]/2) + self.movement_factor * self.min_movement_fraction
                scaled_movement_factor = self.movement_factor
                if not left_right_neighbours[0] and left_right_neighbours[1]: #only right neighbour
                    self.had_green_left = False
                    if self.id == self.observation_id:
                        print(f'checking for green left neighbour {self.green_neighbours}')
                    for i in self.green_neighbours:
                        if i[1] < self.position[0] and i[2]>self.position[1]:
                            if self.id == self.observation_id:
                                print(f'there is a green left neighbour')
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
                    if self.id == self.observation_id:
                        print("no left")
                        print(f'total force {total_force} after {-scaled_movement_factor} * {-self.movement_factor}')
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
                        print("red brown")
                    else:
                        self.state = 1
                        total_force[0] += scaled_movement_factor * len(self.neighbours)
                        if not self.nearEdge():
                            total_force -= gravity_force
                        if self.id == self.observation_id:
                            print("no right")
                            print(f'total force {total_force} after {scaled_movement_factor} scaled from {self.movement_factor}')
                if self.state == 2:
                    if self.had_green_left:
                        total_force -= scaled_movement_factor * len(self.neighbours) * 3.5 * lastknownvector + gravity_force
                    elif self.had_green_right:
                        total_force += scaled_movement_factor * len(self.neighbours) * 1.5 * lastknownvector + gravity_force
                #else: continue falling if no left and right  neighbours

        # try to use spring force to go around obstacle with magnitude projection    
        # self.acceleration = total_force/self.mass
        # self.velocity = np.minimum(self.velocity + self.acceleration * self.deltaTime, np.full(3,self.terminal_velocity))
        # predictedPos =  self.position + self.velocity * self.deltaTime
        # spring_force = self.resolveSpringForce(predictedPos,spring_force)
        # total_force = spring_force + gravity_force - damping_coefficient * self.velocity
        # if left_right_neighbours[0] and left_right_neighbours[1]: #has both neighbors
        #     self.state = 0
        #     if self.id == self.observation_id:
        #         print("state is 0")
        # else:
        #     if not((len(self.neighbours) == 1 and list(self.neighbours.values())[0][0] == 1) or len(self.neighbours) == 0) : #exclude special case of only one or no neighbours
        #         scaled_movement_factor = (self.movement_factor - self.movement_factor * self.min_movement_fraction) * \
        #                                 min(self.position[0] - 0.0,self.plotSize[0] - self.position[0])/(self.plotSize[0]/2) + self.movement_factor * self.min_movement_fraction
        #         scaled_movement_factor = self.movement_factor
        #         if not left_right_neighbours[0] and left_right_neighbours[1]: #only right neighbour
        #             total_force[0] -= scaled_movement_factor * len(self.neighbours)
        #             if not self.nearEdge():
        #                 total_force -= gravity_force
        #             if self.id == self.observation_id:
        #                 print("no left")
        #                 print(f'total force {total_force} after {-scaled_movement_factor} * {-self.movement_factor}')
        #         elif not left_right_neighbours[1] and left_right_neighbours[0]: #only left neighbour
        #             total_force[0] += scaled_movement_factor * len(self.neighbours)
        #             if not self.nearEdge():
        #                 total_force -= gravity_force
        #             if self.id == self.observation_id:
        #                 print("no right")
        #                 print(f'total force {total_force} after {scaled_movement_factor} scaled from {self.movement_factor}')
        #     self.state = 1

        self.acceleration = total_force/self.mass
        self.velocity = np.minimum(self.velocity + self.acceleration * self.deltaTime, np.full(3,self.terminal_velocity))
        
        
        predictedPos =  self.position + self.velocity * self.deltaTime
        self.position = self.resolveCollisions(predictedPos)
        # if self.id == self.observation_id:
        #     # print(f'resultant force {total_force} = spring force {spring_force} + grav {gravity_force}')
        #     # print(f'spring direction {spring_direction}')
        #     # print(f'damped_spring_force {damped_spring_force} = spring_force {spring_force} - {damping_coefficient} * {velocity_parallel}')
        #     # print(f'total_force {total_force} = damped_spring_force {damped_spring_force} + grav {gravity_force}')
        #     print(f'acceleration is {self.acceleration}')
        #     print(f'velocity is {self.velocity}')
        #     print(f'predictedPos is {predictedPos}')
        #     print(f'position is {self.position}')
        if self.state == 0:
            self.green_neighbours = []
            neighbour_values = list(self.neighbours.values())
            for i in neighbour_values:
                self.green_neighbours.append(i)
            if self.id == self.observation_id:
                print(f'added green neighbours {self.green_neighbours}')

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
                if self.id == self.observation_id:
                    print(f'spring force between {self.id} and {i} is {force}')
                    print(f' = unit_dir {unit_direction} * (dist {distance} - target_dist {self.target_dist}) * spring_const {self.spring_constant}')
            
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
                if self.id == self.observation_id:
                    print(f'spring force between {self.id} and {i} is {force}')
                    print(f' = unit_dir {unit_direction} * (dist {distance} - target_dist {self.target_dist}) * spring_const {self.spring_constant}')
    
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
                    if self.id == self.observation_id:
                        print(f'spring force between {self.id} and {i} is {force}')
                        print(f' = unit_dir {unit_direction} * (dist {distance} - target_dist {self.target_dist}) * spring_const {self.spring_constant}')
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

        if self.id == self.observation_id:
            print(f'{self.id} Spring Force: {spring_force}')
            print(f'{self.id} is at : {self.position}')

        ###Move
        self.move(spring_force)
        ###################################################################

        if self.id == self.observation_id:
            print(f'{self.id} move to  : {self.position}')

        ###Update forces based on new position  
        self.updateForces(forceArr)
        #self.updateForcesSquare(forceArr)
        #self.updateForcesString(forceArr)
        ###################################################################
        return        
   
