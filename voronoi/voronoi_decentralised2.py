import sys
import os
sys.path.append(os.path.abspath('../sph/'))
from model import Model
import numpy as np
import math
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import Polygon


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class Voronoi_Decentralised2(Model):
    def __init__(self,id,position,deltaTime,plotSize,plotFigSize,movement_factor,comms_radius,collision_buffer,obstacleList,numAgents):
        self.plotSize = plotSize #allowable movement space
        self.plotFigSize = plotFigSize #total fig size
        self.movement_factor = movement_factor
        self.neighbours = np.array([])
        self.velocity = np.array([0.0,0.0,0.0])
        self.centroid_target = np.array([0.0,0.0,0.0])
        self.regionSize = 0.0
        self.numAgents = numAgents
        self.region = []
        super().__init__(id,position,plotSize=plotSize,collision_buffer=collision_buffer,movement_factor=movement_factor, 
                         force_radius=comms_radius,deltaTime=deltaTime,obstacleList=obstacleList)

    def getVoronoiCentroid(self):
        neighbourList = [x[1:] for x in self.neighbours.values()]
        if len(neighbourList) != 0:
            points = np.vstack([self.position, neighbourList])
        else: 
             points = np.array([self.position])
        reflected_points = np.vstack([
            points,
            points * [-1, 1,1],  # reflect across x=0
            [[x[0],2 * self.plotFloor-x[1],x[2]] for x in points ], #reflect across y  = floor
            [[2 * self.plotSize[0] -x[0],x[1],x[2]] for x in points ], #reflect across x = max
            [[x[0],2 * self.plotFigSize[1]-x[1],x[2]] for x in points ] # reflect across y = max
        ])
        reflected_points = np.delete(reflected_points, [2], axis=1)
        vor = Voronoi(reflected_points)
        region_index = vor.point_region[0]
        region_points = vor.regions[region_index]
        for i in region_points: #for each vertice in each region
                if vor.vertices[i][0] <-1e-6 or vor.vertices[i][0]>self.plotSize[0]+1e6 or \
                    vor.vertices[i][1] <-1e-6 or vor.vertices[i][1]>self.plotSize[1]+1e6: #filter out of bounds regions
                    #print(f'Point {j} has region vertice {vor.vertices[i]} outside of area')
                    curX = vor.vertices[i][0]
                    curY = vor.vertices[i][1]
                    vor.vertices[i][0] = min(max(curX,0),self.plotSize[0])
                    vor.vertices[i][1] = min(max(curY,0),self.plotFigSize[1])
        polygon = [vor.vertices[i] for i in region_points]
        self.region = polygon
        if len(polygon) > 0:
            shapely_polygon = Polygon(polygon)
            shapely_centroid = shapely_polygon.centroid
            self.regionSize = shapely_polygon.area
            #print(f'{self.id} area is {self.regionSize}')
            #print(shapely_centroid)
            centroid = np.array([shapely_centroid.x,shapely_centroid.y])
            #print(f'{self.id} centroid is {centroid}')
        return centroid

    def step(self,comms):
        self.scanSurroundingsOccluded(comms)
        #self.scanSurroundings(comms)
        #centroid_target = [[x[0],x[1],0] for x in self.getVoronoiCentroid()]
        self.centroid_target =self.getVoronoiCentroid()
        self.centroid_target = np.array([self.centroid_target[0],min(self.plotSize[1],self.centroid_target[1]+0.2),0])
        # print(centroid_target)
        # print(self.position)
        self.velocity = (self.centroid_target - self.position) * self.movement_factor * self.deltaTime
        self.position = self.resolveCollisions(self.position + self.velocity)
        #comms[self.id] = self.sendComms()
        
        #TODO: refine the level movement heuristic###################
        neighbour_y = [x[2] for x in self.neighbours.values()]
        has_lower_neighbors = False
        if len(neighbour_y) > 0:
            lowest_neighbour = min(neighbour_y)
            if lowest_neighbour < self.position[1]:
                has_lower_neighbors = True
            highest_neighbour = max(neighbour_y)

        #rear guard code####################
        if not has_lower_neighbors and self.position[1] > self.plotFloor + 0.2:
            self.plotFloor =min(7.0,self.plotFloor+0.1)
        
        #frontline code######################
        at_front = False
        density_reached = False
        if (self.plotSize[1] - self.position[1]) < 1.0:
            at_front = True
            #print(f'{self.id} at front')
            if len(self.neighbours.keys()) > 0:
                density_value = sum((np.linalg.norm(self.position - neighbor_pos[1:]) + 1e-6) \
                                    for neighbor_pos in self.neighbours.values())/len(self.neighbours.keys())
            else:
                density_value = float('inf')
            #print(f'{self.id}:density value is {density_value}')
            if density_value < 4.0:
                density_reached = True
                self.plotSize[1] = min(self.plotFigSize[1],self.plotSize[1]+0.1)
                #print(f'{self.id} increasing')

        
        if len(neighbour_y) > 0:
            self.plotSize[1] = min(10,max(self.plotSize[1],highest_neighbour))
        if has_lower_neighbors:
            self.plotFloor = min(7.0,max(self.plotFloor, lowest_neighbour))
        ##############################################################

        if not has_lower_neighbors:
            self.state = 1
        elif at_front:
            if density_reached:
                self.state = 3
            else:
                self.state = 2
        else:
            self.state = 0