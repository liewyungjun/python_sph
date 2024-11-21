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
class Damping(Model):
    def __init__(self,id,startPos,plotSize,obstacleList = [],target_dist = 1.2,bond_factor = 0.4,
                 observation_id = [0,1],mass = 1,spring_constant=200,gravity=9.81):
        self.mass = mass
        self.spring_constant = spring_constant
        self.gravity = gravity
        force_radius = 1.2* target_dist
        self.terminal_velocity = 5.0
        self.green_neighbours = []
        self.damping_coefficient = 2*math.sqrt(self.mass * self.spring_constant) #critical damping
        self.gravity_force = np.array([0.0, self.gravity*self.mass, 0.0])
        movement_factor = ((force_radius-target_dist) * self.spring_constant)*0.95
        self.min_movement_fraction = 0.5
        super().__init__(id,startPos,plotSize,obstacleList,target_dist,movement_factor,bond_factor,
                 observation_id,force_radius=force_radius,deltaTime=0.02)
        
    def move(self,forceArr,objectList):
        spring_force = np.zeros(3)
        damping_force = np.zeros(3)
        projected_vel_magnitude = 0
        spring_normal = np.zeros(3)
        relative_vel = np.zeros(3)

        for i in self.neighbours.keys():
            neighbour_force = forceArr[self.id][i]
            if self.id in self.observation_id:
                print(f'reading {neighbour_force} from neighbour {i}')
            if abs(neighbour_force.any())> 0.01:
                spring_force += neighbour_force
                relative_vel = objectList[i].velocity - self.velocity
                spring_normal = (np.array(self.neighbours[i][1:]) - self.position)/np.linalg.norm((np.array(self.neighbours[i][1:]) - self.position))
                projected_vel_magnitude = np.dot(relative_vel,spring_normal)
                damping_force +=self.damping_coefficient * projected_vel_magnitude * spring_normal
        
        #total_force = spring_force + self.gravity_force - self.damping_coefficient * self.velocity
        if self.id in self.observation_id:
            print(f'from velocity {self.velocity}')
            print(f'from position {self.position}')

        total_force = spring_force + self.gravity_force + damping_force
        self.acceleration = total_force/self.mass
        self.velocity = np.minimum(self.velocity + self.acceleration * self.deltaTime, np.full(3,self.terminal_velocity))
        predictedPos =  self.position + self.velocity * self.deltaTime
        self.position = self.resolveCollisions(predictedPos)

        if self.id in self.observation_id:
            print(f'projected vel mag {projected_vel_magnitude} = relative vel {relative_vel} * spring normal {spring_normal}')
            print(f'damping {damping_force} = dc {self.damping_coefficient} * proj vel mag {projected_vel_magnitude} * spring norm {spring_normal}')
            print(f'resultant force {total_force} = spring force {spring_force} + grav {self.gravity_force} + damping {damping_force}')
            print(f'acceleration is {self.acceleration}')
            print(f'velocity is {self.velocity}')
            print(f'predictedPos is {predictedPos}')
            print(f'position is {self.position}')

        for i in self.neighbours.keys():
            neighbourpos = np.array(self.neighbours[i][1:])
            direction = neighbourpos - self.position
            distance = np.linalg.norm(direction)
            if distance > 0:
                unit_direction = direction / distance
                force = unit_direction * (distance - self.target_dist) * self.spring_constant
                forceArr[self.id][i] = (force)
                forceArr[i][self.id] = (-force)
                if self.id in self.observation_id:
                    print(f'force {force} = unit direction {unit_direction} * (dist {distance} - self.target_dist {self.target_dist}) * self.spring_constant {self.spring_constant}')



    def step(self,forceArr,globalComms,objectList):
        ###Select scanning method##########################################
        self.scanSurroundings(globalComms)
        ###Move
        self.move(forceArr,objectList)
        return        
   
