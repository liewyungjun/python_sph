import sys
import os
sys.path.append(os.path.abspath('../sph/'))
from model import Model
import numpy as np
import math
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import Polygon


np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})

class Voronoi_Decentralised(Model):
    def __init__(self,id,position,deltaTime,plotBounds,movement_factor,comms_radius,collision_buffer,obstacleList,numAgents):
        self.plotBounds = plotBounds
        self.movement_factor = movement_factor
        self.neighbours = np.array([])
        self.velocity = np.array([0.0,0.0,0.0])
        self.regionSize = 0.0
        self.numAgents = numAgents
        self.region = []
        super().__init__(id,position,plotSize=plotBounds,collision_buffer=collision_buffer,movement_factor=movement_factor, 
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
            [[2 * self.plotBounds[0] -x[0],x[1],x[2]] for x in points ], #reflect across x = max
            [[x[0],2 * self.plotBounds[1]-x[1],x[2]] for x in points ] # reflect across y = max
        ])
        reflected_points = np.delete(reflected_points, [2], axis=1)
        vor = Voronoi(reflected_points)
        region_index = vor.point_region[0]
        region_points = vor.regions[region_index]
        for i in region_points: #for each vertice in each region
                if vor.vertices[i][0] <-1e-6 or vor.vertices[i][0]>self.plotBounds[0]+1e6 or \
                    vor.vertices[i][1] <-1e-6 or vor.vertices[i][1]>self.plotBounds[1]+1e6: #filter out of bounds regions
                    #print(f'Point {j} has region vertice {vor.vertices[i]} outside of area')
                    curX = vor.vertices[i][0]
                    curY = vor.vertices[i][1]
                    vor.vertices[i][0] = min(max(curX,0),self.plotBounds[0])
                    vor.vertices[i][1] = min(max(curY,0),self.plotBounds[1])
        polygon = [vor.vertices[i] for i in region_points]
        self.region = polygon
        if len(polygon) > 0:
            shapely_polygon = Polygon(polygon)
            shapely_centroid = shapely_polygon.centroid
            self.regionSize = shapely_polygon.area
            print(f'{self.id} area is {self.regionSize}')
            #print(shapely_centroid)
            centroid = np.array([shapely_centroid.x,shapely_centroid.y])
            print(f'{self.id} centroid is {centroid}')
        return centroid

    def step(self,comms):
        self.scanSurroundingsOccluded(comms)
        #centroid_target = [[x[0],x[1],0] for x in self.getVoronoiCentroid()]
        centroid_target =self.getVoronoiCentroid()
        centroid_target = np.array([centroid_target[0],centroid_target[1],0])
        # print(centroid_target)
        # print(self.position)
        self.velocity = (centroid_target - self.position) * self.movement_factor * self.deltaTime
        self.position = self.resolveCollisions(self.position + self.velocity)
        comms[self.id] = self.sendComms()
        if self.regionSize < 2 * (self.plotBounds[1]-self.plotFloor) * self.plotBounds[0] / self.numAgents and \
            self.position[1] > self.plotFloor + 0.2:
            self.plotBounds[1] +=0.01
            self.plotFloor +=0.01
            print(self.plotBounds[1])
            print(self.plotFloor)

