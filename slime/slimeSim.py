from slime import Slime
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys
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
        numSlimes = int(sys.argv[1])
    except IndexError:
        numSlimes = 6
    #numSlimes = 6
    slimes = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    #obstacles = []
    plotsize = (10,10)
    
    #slimes[-1].position = np.array([9.5,9.5,0])

    save = False
    results_path = "demo_results"
    savename = "slime.mp4"
    frame_length = 2000

    loadmap = False
    mapname = 'sparse_spring'
    loadpath = "demo_maps"

    if loadmap:
        obstacles = readMap(mapname,loadpath)

    for i in range(numSlimes):
            slimes.append(Slime(i,[i%5+1,i//5,0],plotSize=plotsize,obstacleList=obstacles))
            
    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*0.5*0.8), (y_limits[1] - y_limits[0]) / (2.54*0.5*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 5 * points_per_unit  # 5 axes unit marker diameter

    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
    ax.set_xlim(x_limits)
    ax.set_ylim(y_limits)
    slimeaxs = []
    for i in range(numSlimes):
        slimeaxs.append(ax.plot([], [], 'b^')[0])

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)
        #polygons.append(polygon)

    globalcomms = [[] for i in range(numSlimes)]
    connections_lines = []
    texts = []
    #for i in range(numSlimes):
            #print(f'{i} is at {slimes[i].position}')
    def update(frame):
        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numSlimes):
            globalcomms[i] = slimes[i].sendComms()
        for i in range(numSlimes):
            slimes[i].scanSurroundings(globalcomms)
        for i in range(numSlimes):
            #print(f'{slimes[i].id} neighbours: {slimes[i].neighbours}')
            slimes[i].step()
        global texts
        for text in texts:
            text.remove()
        texts = []
        for i in range(numSlimes):
            slimeaxs[i].set_data([slimes[i].position[0]],[slimes[i].position[1]])
            text = ax.text(slimes[i].position[0], slimes[i].position[1]+0.2, str(slimes[i].id), ha='center', va='bottom')        
            texts.append(text)
            # Change color dynamically based on frame number or any other condition
            colors = ['green', 'blue', 'brown','red','purple']
            slimeaxs[i].set_color(colors[slimes[i].state])    
        print(f'{frame}-------------')
        #for i in range(numSlimes):
            #print(f'{i} is at {slimes[i].position}')
        # Draw lines between neighbours
        for i in range(numSlimes):
            for neighbour in slimes[i].neighbours:
                line= ax.plot([slimes[i].position[0], slimes[neighbour].position[0]], 
                        [slimes[i].position[1], slimes[neighbour].position[1]], 
                        'g-', alpha=0.3)
                connections_lines.append(line[0])
        
        #time.sleep(0.2)    
        return slimeaxs + connections_lines + texts
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