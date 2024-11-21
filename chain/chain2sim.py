from matplotlib.collections import LineCollection
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

def readStartingPos(positionName,loadpath):
    with open(f'{loadpath}/{positionName}.txt', 'r') as f:
        content = f.read()
        # Convert string representation of list to actual list
        points = eval(content)
        #print(points)
        # Since we have a single obstacle, we'll return it in the obstacleList format
        startingPos = points
        #obstacleList = [[(x, y) for x, y in points]]
        return startingPos

if __name__ == '__main__':
    try:
        numModel = int(sys.argv[1])
    except IndexError:
        numModel = 6
    chain2 = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    #obstacles = []
    plotsize = (10,10)
    axesScaling = 0.75 #size of plot
    
    save = False
    results_path = "results"
    savename = "opt_test.mp4"
    frame_length = 3000

    loadmap = True
    mapname = 'basic_1010'
    loadpath = "demo_maps"

    loadStartPos = False
    loadStartingPath = 'demo_starting_pos'
    positionName = "edgepull"

    mass = 1
    spring_constant = 200
    gravity = 9.81
    target_dist = 1.2

    trails = False
    if loadmap:
        obstacles = readMap(mapname,loadpath)

    if loadStartPos:
        startingPoints = readStartingPos(positionName,loadStartingPath)
        numModel = len(startingPoints)

    for i in range(numModel):
            if loadStartPos:
                chain2.append(Chain2(i,[startingPoints[i][0],startingPoints[i][1],0],plotSize=plotsize,
                                     obstacleList=obstacles,mass=mass,spring_constant=spring_constant,
                                     gravity=gravity,target_dist=target_dist))
            else:
                chain2.append(Chain2(i,[i%5+3,i//5/2,0],plotSize=plotsize,obstacleList=obstacles,mass=mass,
                                     spring_constant=spring_constant,gravity=gravity,target_dist=target_dist))
            
    x_limits = (0, plotsize[0])
    y_limits = (0, plotsize[1])
    figsize = ((x_limits[1]-x_limits[0]) / (2.54*axesScaling*0.8), (y_limits[1] - y_limits[0]) / (2.54*axesScaling*0.8))
    inches_per_unit = figsize[0] / (x_limits[1] - x_limits[0])  # how many inches per axis unit
    points_per_unit = inches_per_unit * 72  # convert to points
    markersize = 0.1 * points_per_unit  # 5 axes unit marker diameter

    # Set up the figure and axis
    fig, ax = plt.subplots(figsize=(figsize),dpi=122)  # e.g. 30 units = 30cm/2 = 15cm size graph
    ax.grid(True)
    ax.set_xticks(range(int(x_limits[0]), int(x_limits[1])+1))
    ax.set_yticks(range(int(y_limits[0]), int(y_limits[1])+1))    
    ax.set_xlim(x_limits)
    ax.set_ylim(y_limits)
    plt.title(f"Simulation\nParticles: {numModel}")
    
    globalcomms = [[] for i in range(numModel)]
    connections_lines = []
    texts = []
    trajs = []
    traj_hist = []
    modelaxs = []
    velocity_arrows = []
    forceArr  = [[np.array([0.0,0.0,0.0]) for i in range(numModel)] for j in range(numModel)]

    for i in range(numModel):
        modelaxs.append(ax.plot([], [], 'b^')[0])
        trajs.append(ax.plot([], [], 'c.', markersize=markersize, alpha = 0.1)[0])
        traj_hist.append([[],[]])
        velocity_arrows.append(ax.quiver([], [], [], [], scale=1))

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        ax.add_patch(polygon)
    
    state_colors = ['green', 'blue', 'brown','red','purple']

    def update(frame):
        start_time = time.time()

        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numModel):
            globalcomms[i] = chain2[i].sendComms()
        # for i in range(numModel):
        #     chain2[i].scanSurroundings(globalcomms)
        comms_time = time.time()
        for i in range(numModel):
            chain2[i].step(forceArr,globalcomms)
        steps_time = time.time()
        #particle label
        global texts
        for text in texts:
            text.remove()
        texts = []
        
        positions_x = [chain2[i].position[0] for i in range(numModel)]
        positions_y = [chain2[i].position[1] for i in range(numModel)]
        
        # Update all particle positions at once
        for i in range(numModel):
            modelaxs[i].set_data([positions_x[i]], [positions_y[i]])
            modelaxs[i].set_color(state_colors[chain2[i].state])
        
        posState_time = time.time()
        
        # Create all text labels at once
        texts = [ax.text(x, y+0.2, str(chain2[i].id), ha='center', va='bottom') 
                for i, (x, y) in enumerate(zip(positions_x, positions_y))]
        text_time = time.time()
        if trails:
            for i in range(numModel):
                
                traj_hist[i][0].append(positions_x[i])
                traj_hist[i][1].append(positions_y[i])
                trajs[i].set_data(traj_hist[i][0], traj_hist[i][1])
        
        # Create all connection lines at once
        neighbor_pairs = [(i, n) for i in range(numModel) for n in chain2[i].neighbours]
        segments = np.array([[(positions_x[i], positions_y[i]), 
                            (positions_x[j], positions_y[j])] for i, j in neighbor_pairs])
        connections_lines = [LineCollection(segments, colors='g', alpha=0.3)]
        ax.add_collection(connections_lines[0])        

        lines_time = time.time()
        end_time = time.time()
        # print(f'Time taken: {end_time - start_time} seconds')
        # print(f'Comms time: {comms_time - start_time} seconds')
        # print(f'Steps time: {steps_time - comms_time} seconds')
        # print(f'Draw time: {end_time - steps_time} seconds')
        # print(f'posState time: {posState_time - steps_time} seconds')
        # print(f'text time: {text_time - posState_time} seconds')
        # print(f'lines time: {lines_time - text_time} seconds')
        #time.sleep(0.2)    
        #return modelaxs + connections_lines + texts + trajs + velocity_arrows    
        print(f'{frame}-------------')
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