from chain2 import Chain2
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
    
def generateParticleGrid(numParticles,plotSize,gridSpacing):#particle generation
    particleList = []
    x_cap = plotSize[0] / gridSpacing
    if x_cap > numParticles:
        for i in range(numParticles):
            x = i * gridSpacing
            y = gridSpacing
            particleList.append((x,y))
    else:
        y_offset = 1
        rownum = 0
        for i in range(numParticles):
            if i // x_cap > rownum:
                y_offset +=1
                rownum +=1
            x = i%x_cap * gridSpacing
            y = y_offset * gridSpacing
            particleList.append((x,y))
    return particleList


if __name__ == '__main__':
    try:
        numModel = int(sys.argv[1])
    except IndexError:
        numModel = 6
    chain2 = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    #obstacles = []
    plotsize = (10,10)
    axesScaling = 0.5 #size of plot
    
    save = False
    results_path = "demo_results"
    savename = "chain2.mp4"
    frame_length = 2500

    loadmap = False
    mapname = 'sparse_spring'
    loadpath = "demo_maps"

    mass = 1
    spring_constant = 200
    gravity = 9.81

    trails = False
    if loadmap:
        obstacles = readMap(mapname,loadpath)

    for i in range(numModel):
            chain2.append(Chain2(i,[i%5+1,i//5,0],plotSize=plotsize,obstacleList=obstacles,mass=mass,spring_constant=spring_constant,gravity=gravity))
            
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
    plt.title(f"Simulation\nParticles: {numModel}")
    
    globalcomms = [[] for i in range(numModel)]
    connections_lines = []
    texts = []
    trajs = []
    traj_hist = []
    modelaxs = []
    forceArr  = [[np.array([0.0,0.0,0.0]) for i in range(numModel)] for j in range(numModel)]

    for i in range(numModel):
        modelaxs.append(ax.plot([], [], 'b^')[0])
        trajs.append(ax.plot([], [], 'c.', markersize=markersize, alpha = 0.1)[0])
        traj_hist.append([])

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)
    
    def update(frame):
        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numModel):
            globalcomms[i] = chain2[i].sendComms()
        # for i in range(numModel):
        #     chain2[i].scanSurroundings(globalcomms)
        for i in range(numModel):
            chain2[i].step(forceArr,globalcomms)
        
        #particle label
        global texts
        for text in texts:
            text.remove()
        texts = []
        for i in range(numModel):
            modelaxs[i].set_data([chain2[i].position[0]],[chain2[i].position[1]])
            text = ax.text(chain2[i].position[0], chain2[i].position[1]+0.2, str(chain2[i].id), ha='center', va='bottom')        
            texts.append(text)
            # Change color dynamically based on frame number or any other condition
            colors = ['green', 'blue', 'brown','red','purple']
            modelaxs[i].set_color(colors[chain2[i].state])  
            if trails:
                if frame == 0:
                    traj_hist[i] = [[chain2[i].position[0]],[chain2[i].position[1]]]
                else:
                    traj_hist[i][0].append(chain2[i].position[0])
                    traj_hist[i][1].append(chain2[i].position[1])
                #print(traj_hist)
                trajs[i].set_data([traj_hist[i][0]],[traj_hist[i][1]])
        print(f'{frame}-------------')
        # Draw lines between neighbours
        for i in range(numModel):
            for neighbour in chain2[i].neighbours:
                line= ax.plot([chain2[i].position[0], chain2[neighbour].position[0]], 
                        [chain2[i].position[1], chain2[neighbour].position[1]], 
                        'g-', alpha=0.3)
                connections_lines.append(line[0])
        
        #time.sleep(0.2)    
        return modelaxs + connections_lines + texts + trajs
    
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
            params_file = os.path.join(folder_name, "parameters.txt")
            with open(params_file, "w") as f:
                f.write(f"Simulation Parameters:\n")
                f.write(f"------------------------------------\n")
                f.write(f"Simulation Steps: {frame_length}\n")
                f.write(f"Spring Constant: {spring_constant}\n")
                f.write(f"Gravity: {gravity}\n")
                f.write(f"Force Radius: {chain2[0].force_radius}\n")
                f.write(f"Target Distance: {chain2[0].target_dist}\n")
                

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.show()