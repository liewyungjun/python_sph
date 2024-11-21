from chain import Chain
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys
import sys
import os
from datetime import datetime

if __name__ == '__main__':
    try:
        numChains = int(sys.argv[1])
    except IndexError:
        numChains = 6
    #numChains = 6
    Chains = []
    #obstacles = [[(2,6),(6,6),(6,2),(2,2)]]
    obstacles = []
    plotsize = (10,10)
    for i in range(numChains):
        Chains.append(Chain(i,[i%5+1.1,i//5,0],plotSize=plotsize,obstacleList=obstacles))
    #Chains[-1].position = np.array([9.5,9.5,0])

    save = False
    results_path = "demo_results"
    savename = "Chain.mp4"
    frame_length = 2000

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
    Chainaxs = []
    for i in range(numChains):
        Chainaxs.append(ax.plot([], [], 'b^')[0])

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)
        #polygons.append(polygon)

    globalcomms = [[] for i in range(numChains)]
    connections_lines = []
    texts = []
    #for i in range(numChains):
            #print(f'{i} is at {Chains[i].position}')
    def update(frame):
        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numChains):
            globalcomms[i] = Chains[i].sendComms()
        for i in range(numChains):
            Chains[i].scanSurroundings(globalcomms)
        for i in range(numChains):
            #print(f'{Chains[i].id} neighbours: {Chains[i].neighbours}')
            Chains[i].step()
        global texts
        for text in texts:
            text.remove()
        texts = []
        for i in range(numChains):
            Chainaxs[i].set_data([Chains[i].position[0]],[Chains[i].position[1]])
            text = ax.text(Chains[i].position[0], Chains[i].position[1]+0.2, str(Chains[i].id), ha='center', va='bottom')        
            texts.append(text)
            # Change color dynamically based on frame number or any other condition
            colors = ['green', 'blue', 'brown','red','purple']
            Chainaxs[i].set_color(colors[Chains[i].state])    
        print(f'{frame}-------------')
        #for i in range(numChains):
            #print(f'{i} is at {Chains[i].position}')
        # Draw lines between neighbours
        for i in range(numChains):
            for neighbour in Chains[i].neighbours:
                line= ax.plot([Chains[i].position[0], Chains[neighbour].position[0]], 
                        [Chains[i].position[1], Chains[neighbour].position[1]], 
                        'g-', alpha=0.3)
                connections_lines.append(line[0])
        
        #time.sleep(0.2)    
        return Chainaxs + connections_lines + texts
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