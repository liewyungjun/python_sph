import math
import numpy as np
from model import Model
import time


#FBD-based chain
class Chain2(Model):
    def __init__(self,id,startPos,plotSize,obstacleList = [],target_dist = 1.2,movement_factor = 0.05,bond_factor = 0.4,
                 observation_id = 0):
        super().__init__(id,startPos,plotSize,obstacleList,target_dist,movement_factor,bond_factor,
                 observation_id)
        self.mass = 1
        self.spring_constant = 20.0
        self.gravity = 9.81
        self.force_radius = 1.2* self.target_dist
    
    def calculate_FBD(self,forceArr):
        force = np.array([0.0, self.gravity*self.mass, 0.0])  # gravity in y direction
        for i in self.neighbours.keys():
            neighbour_force = forceArr[self.id][i]
            if abs(neighbour_force.any())> 0.01:
                force += neighbour_force * 0.8
        return force

    def move(self,resultant_force):
        damping_coefficient = 2*math.sqrt(self.mass * self.spring_constant) #critical damping
        resultant_force -= damping_coefficient * self.velocity
        self.acceleration = resultant_force/self.mass
        self.velocity = self.velocity + self.acceleration * self.deltaTime
        predictedPos =  self.position + self.velocity * self.deltaTime
        self.position = self.resolveCollisions(predictedPos)
        if self.id == self.observation_id:
            print(f'acceleration is {self.acceleration}')
            print(f'velocity is {self.velocity}')
            print(f'predictedPos is {predictedPos}')
            print(f'position is {self.position}')

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
        

    def step(self,forceArr,globalComms):
        self.scanSurroundings(globalComms)
        resultant_force = self.calculate_FBD(forceArr)
        if self.id == self.observation_id:
            print(f'{self.id} Resultant Force: {resultant_force}')
            print(f'{self.id} is at : {self.position}')
        self.move(resultant_force)
        if self.id == self.observation_id:
            print(f'{self.id} move to  : {self.position}')
        self.updateForces(forceArr)
        # if self.id == self.observation_id:
        #     time.sleep(0.5)
        return        
   
