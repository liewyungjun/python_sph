from spring import Spring
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys
import os
from datetime import datetime

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
    

if __name__ == '__main__':
    try:
        numSprings = int(sys.argv[1])
    except IndexError:
        numSprings = 6
    #numSprings = 6
    springs = []
    obstacles = [[(2,6),(4,6),(4,4),(2,4)],[(5,8),(6,8),(6,6),(5,6)]]

    frame_length  = 2000

    loadmap = True
    mapname = 'maze_spring'
    loadpath = "demo_maps"

    save = True
    results_path = "demo_results"
    savename = "maze_spring.mp4"

    if loadmap:
        obstacles = readMap(mapname,loadpath)

    #obstacles = [[(4,8),(6,8),(6,5),(4,5)]]
    #obstacles = []
    plotsize = (10,10)
    scaling_factor = 0.5
    for i in range(numSprings):
        springs.append(Spring(i,np.array([i%5+3,i//10,0],dtype='float64'),plotSize=plotsize,obstacleList=obstacles))
    springs[0].position = np.array([7,2,0])
    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*scaling_factor*0.8), (y_limits[1] - y_limits[0]) / (2.54*scaling_factor*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 5 * points_per_unit  # 5 axes unit marker diameter

    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
    ax.set_xlim(x_limits)
    ax.set_ylim(y_limits)
    springaxs = []
    for i in range(numSprings):
        springaxs.append(ax.plot([], [], 'b^')[0])

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)
        #polygons.append(polygon)

    globalcomms = [np.zeros(4) for i in range(numSprings)]
    connections_lines = []
    texts = []

    def update(frame):
        global globalcomms
        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numSprings):
            springs[i].sendComms(globalcomms)
        #print(globalcomms)
        # Clear previous texts
        global texts
        for text in texts:
            text.remove()
        texts = []
        
        for i in range(numSprings):
            springs[i].step(globalcomms)
            springaxs[i].set_data([springs[i].position[0]],[springs[i].position[1]])
            text = ax.text(springs[i].position[0], springs[i].position[1]+0.2, str(springs[i].id), ha='center', va='bottom')        
            texts.append(text)
            # Change color dynamically based on frame number or any other condition
            colors = ['blue', 'red']
            springaxs[i].set_color(colors[springs[i].state])    
        print(f'{frame}-------------')
        #for i in range(numSprings):
            #print(f'{i} is at {springs[i].position}')
        
        #Draw lines between neighbours
        for i in range(numSprings):
            for neighbour in springs[i].neighbours:
                if neighbour >= len(springs):
                    continue
                try:
                    line= ax.plot([springs[i].position[0], springs[neighbour].position[0]], 
                        [springs[i].position[1], springs[neighbour].position[1]], 
                        'g-', alpha=0.3)
                except IndexError:
                    print(f'{i} or {neighbour} out of index for length {len(springs)}')
                connections_lines.append(line[0])
        # if frame == 150:
        #     springs[3].neighbours = [4]
        #     springs[2].neighbours = [1]
        #time.sleep(0.2)    
        return springaxs + texts + connections_lines
    
    anim = animation.FuncAnimation(fig, update, frames=frame_length, interval=0.02 * 1000, blit=True,repeat=False)
    
    if save:
        # Create timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Create folder name with simulation name and timestamp
        folder_name = f"{results_path}/{timestamp}_{savename.split('.')[0]}"
        
        # Create folders if they don't exist
        os.makedirs(folder_name, exist_ok=True)
        
        # Update save path to include new folder
        save_path = os.path.join(folder_name, savename)
        
        # Save animation to the new folder
        writervideo = animation.FFMpegWriter(fps=60)
        anim.save(save_path, writer=writervideo)
    
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()
