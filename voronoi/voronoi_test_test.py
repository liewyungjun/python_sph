from matplotlib.collections import LineCollection
from scipy.spatial import Voronoi, voronoi_plot_2d
from voronoi_decentralised import Voronoi_Decentralised
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys
import os
from datetime import datetime
from utils.helperfunctions import readMap,readStartingPos
from shapely.geometry import Polygon

if __name__ == '__main__':
    try:
        numModel = int(sys.argv[1])
    except IndexError:
        numModel = 6
    points = []
    plotsize = (10,10)
    axesScaling = 0.75 #size of plot
    deltaTime = 0.02
    movement_factor = 0.5
    comms_radius = 5.0
    collision_buffer = 0.5
    
    frame_length = 1500

    rowlength = 5

    trails = False

    for i in range(numModel):
        points.append([i%rowlength*1*0.75+plotsize[0]/4,i/2])
    points = np.array(points)
    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 0.1 * points_per_unit  # 5 axes unit marker diameter

    # Set up the figure and axis
    connections_lines = []
    texts = []
    trajs = []
    traj_hist = []
    modelaxs = []
    
    state_colors = ['green', 'blue', 'brown','red','purple']

    print(points)
    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 0.1 * points_per_unit  # 5 axes unit marker diameter
    # fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
    # ax.grid(True)
    # ax.set_xticks(range(int(x_limits[0]), int(x_limits[1])+1))
    # ax.set_yticks(range(int(y_limits[0]), int(y_limits[1])+1))    
    # ax.set_xlim(x_limits)
    # ax.set_ylim(y_limits)
    
    
    def voronoiPlot(points):
        reflected_points = np.vstack([
        points,
        points * [-1, 1],  # reflect across x=0
        points * [1, -1],  # reflect across y=0
        [[10 + 10-x[0],x[1]] for x in points ],
        [[x[0],10 + 10-x[1]] for x in points ]
        ])


        vor = Voronoi(reflected_points)
        fig2, ax = plt.subplots(figsize=figsize)
        voronoi_plot_2d(vor, ax=ax)        
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        #plt.figure(figsize=figsize)        
        centroids = [[] for i in range(numModel)]
        global areas
        for j in range(numModel): #check the index for its associated point
            regionIdx = vor.point_region[j]
            region = vor.regions[regionIdx]
            for i in region: #for each vertice in each region
                if vor.vertices[i][0] <-1e-6 or vor.vertices[i][0]>10+1e6 or \
                    vor.vertices[i][1] <-1e-6 or vor.vertices[i][1]>10+1e6: #filter out of bounds regions
                    print(f'Point {j} has region vertice {vor.vertices[i]} outside of area')
                    curX = vor.vertices[i][0]
                    curY = vor.vertices[i][1]
                    vor.vertices[i][0] = min(max(curX,0),10)
                    vor.vertices[i][1] = min(max(curY,0),10)
            polygon = [vor.vertices[i] for i in region]
            if len(polygon) > 0:
                shapely_polygon = Polygon(polygon)
                shapely_centroid = shapely_polygon.centroid
                shapely_area = shapely_polygon.area
                print(shapely_centroid)
                centroid = np.array([shapely_centroid.x,shapely_centroid.y])
                #centroid = np.mean(polygon, axis=0)
                plt.plot(centroid[0], centroid[1], 'ko')
                label = j
                plt.text(centroid[0], centroid[1],str(label), fontsize=12, ha='right')
                print(f'Point {j} centroid: {centroid}')
                centroids[j] = centroid
                areas[j].append(shapely_area)


        # Add vertex index labels
        for i, vertex in enumerate(vor.points):
            if not(vertex[0] <0 or vertex[0]>10 or vertex[1]<0 or vertex[1]>10):
                plt.text(vertex[0], vertex[1], str(i), fontsize=8, ha='right')

        for i, vertex in enumerate(vor.vertices):
            if not(vertex[0] <0 or vertex[0]>10 or vertex[1] <0 or vertex[1]>10):
                plt.text(vertex[0], vertex[1], str(i), fontsize=8, ha='right')

        return  centroids

steps = 10
energyValueArr = [0.0 for x in range(steps)]
areas = [[] for i in range(numModel)]
for i in range(steps):
    newPoints = np.array(voronoiPlot(points))
    energyValue = 0
    
    for j in range(numModel):
        energyValue += np.linalg.norm(newPoints - points)
    print(f'energyValue: {energyValue}')
    energyValueArr[i] = energyValue
    points = newPoints
    plt.show()
    print(f'{i}--------------------------------------------')

plt.figure()
plt.plot([x for x in range(steps)], energyValueArr, 'b-')
plt.xlabel('Iteration')
plt.ylabel('Energy Value')
plt.title('Energy Value vs Iteration')
plt.grid(True)
plt.show()

for i in range(len(areas)):
    print(f'{i} final area: {areas[i][-1]}')

plt.figure()
for i in range(numModel):
    plt.plot([x for x in range(steps)], areas[i], label=f'Point {i}')
plt.xlabel('Iteration')
plt.ylabel('Area')
plt.title('Area vs Iteration')
plt.grid(True)
plt.legend()
plt.show()
