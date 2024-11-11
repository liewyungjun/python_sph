from slime import Slime
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys

if __name__ == '__main__':
    try:
        numSlimes = int(sys.argv[1])
    except IndexError:
        numSlimes = 6
    #numSlimes = 6
    slimes = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    obstacles = []
    plotsize = (10,10)
    for i in range(numSlimes):
        slimes.append(Slime(i,[i%5+1,i//5+2,0],plotSize=plotsize,obstacleList=obstacles))
    #slimes[-1].position = np.array([9.5,9.5,0])

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
    anim = animation.FuncAnimation(fig, update, frames=2000, interval=0.2, blit=True,repeat=False)

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.show()