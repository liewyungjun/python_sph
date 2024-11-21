import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

import numpy as np
import sys
import os

def read_map_and_get_clicks():
    # Read the map file
    obstacles = []

    def readMap(mapname,loadpath):
        with open(f'{loadpath}/{mapname}.txt', 'r') as f:
            content = f.read()
            # Convert string representation of list to actual list
            points = eval(content)
            #print(points)
            # Since we have a single obstacle, we'll return it in the obstacleList format
            obstacleList = points
            #obstacleList = [[(x, y) for x, y in points]]
            return obstacleList
        
    mapname = 'basic_1010'
    loadpath = "demo_maps"
    obstacles = readMap(mapname,loadpath)
    plotsize = (10,10)
    axesScaling = 0.5
    savepath = "demo_starting_pos"
    saveName = "edgepull_single"



    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 5 * points_per_unit  # 5 axes unit marker diameter

    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
    ax.set_xlim(x_limits)
    ax.set_ylim(y_limits)


    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)

    # Initialize empty lists for click coordinates
    count = 0
    click_points = []

    def onclick(event):
        if event.button == 1:  # Left click
            click_points.append((event.xdata, event.ydata))
            ax.scatter(event.xdata, event.ydata, c='red')
            ax.text(event.xdata, event.ydata + 0.2, f'{len(click_points)-1}', ha='center')
            fig.canvas.draw()
    
    def on_close(event):
        if click_points:
            with open(f'{savepath}/{saveName}.txt', 'a') as f:
                f.write('[')
                for i in range(len(click_points)-1):
                    f.write(str(click_points[i]) + ',\n')
                f.write(str(click_points[-1]) + ']\n')
    
    def onkey(event):
        global points
        if event.key == 'home':  # Reset plot when home key is pressed
            ax.clear()
            ax.set_xlim(x_limits)
            ax.set_ylim(y_limits)
            points = []
            global  click_points
            click_points = []
            plt.draw()        

    # Connect the click event to the figure
    cid = fig.canvas.mpl_connect('button_press_event', onclick)
    fig.canvas.mpl_connect('close_event', on_close)
    fig.canvas.mpl_connect('key_press_event', onkey)



    plt.grid(True)
    plt.title('Click to select positions')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

    return click_points

if __name__ == '__main__':
    positions = read_map_and_get_clicks()
    print("Selected positions:", positions)
