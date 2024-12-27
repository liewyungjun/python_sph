from matplotlib.collections import LineCollection
from voronoi_decentralised2 import Voronoi_Decentralised2
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Polygon
import numpy as np
import sys
import os
from datetime import datetime
from utils.helperfunctions import readMap,readStartingPos

if __name__ == '__main__':
    try:
        numModel = int(sys.argv[1])
    except IndexError:
        numModel = 6
        
    VoronoiDecentralised2 = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    obstacles = []
    plotfigsize = [10,10] #for plotting purposes
    plotSize = [10,2] #voronoi consideration bounds (aka bound level)
    axesScaling = 0.75 #size of plot
    deltaTime = 0.02
    movement_factor = 0.5
    comms_radius = 12.0
    #comms_radius = 2.0
    collision_buffer = 0.02
    
    save = False
    results_path = "results"
    savename = "Voronoi_6drones_global_sparse.mp4"
    frame_length = 2500

    loadmap = True
    mapname = 'sparse_chain'
    loadpath = "maps"

    loadStartPos = False
    loadStartingPath = 'demo_starting_pos'
    positionName = "30_middle"

    rowlength = 5

    mass = 1

    trails = False
    voronoi_markings = True

    if loadmap:
        obstacles = readMap(mapname,loadpath)

    if loadStartPos:
        startingPoints = readStartingPos(positionName,loadStartingPath)
        numModel = len(startingPoints)

    
    for i in range(numModel):
            if loadStartPos:
                VoronoiDecentralised2.append(Voronoi_Decentralised2(i,[startingPoints[i][0],startingPoints[i][1],0],deltaTime=deltaTime,
                                                                  plotSize=[plotSize[0],plotSize[1]],plotFigSize = plotfigsize,movement_factor=movement_factor,
                                                                  comms_radius=comms_radius,collision_buffer=collision_buffer,obstacleList=obstacles,numAgents=numModel))
            else:
                VoronoiDecentralised2.append(Voronoi_Decentralised2(i,[i%rowlength*1*0.75+plotfigsize[0]/4,i//rowlength/2 + 0.2,0],deltaTime=deltaTime,
                                                                  plotSize=[plotSize[0],plotSize[1]],plotFigSize = plotfigsize,movement_factor=movement_factor,
                                                                  comms_radius=comms_radius,collision_buffer=collision_buffer,obstacleList=obstacles,numAgents=numModel))
            
    x_limits = (0, plotfigsize[0])
    y_limits = (0, plotfigsize[1])
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
    regions = []
    floorLines,boundLines = [],[]
    centroidtargets = []
    if voronoi_markings:
        floorLines = [ax.axhline(y=x.plotFloor, color='red', linestyle='-',alpha=0.2) for x in VoronoiDecentralised2]
        boundLines = [ax.axhline(y=x.plotSize[1], color='blue', linestyle='-',alpha=0.2) for x in VoronoiDecentralised2]
        for i in range(numModel):
            centroidtargets.append(ax.plot([], [], 'o', mfc='none', mec='b')[0])
    voronoi_polygons = [[] for i in range(numModel)]

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
        polygon = Polygon([[0,0]], fill=True, alpha=0.2,edgecolor='black', linewidth=1)
        ax.add_patch(polygon)
        voronoi_polygons[i] = polygon

    for obstacle in obstacles:
        polygon = Polygon(obstacle, fill=True, facecolor='gray')
        inflation = [[obstacle[0][0]-collision_buffer, obstacle[0][1]+collision_buffer], [obstacle[1][0]+collision_buffer, obstacle[1][1]+collision_buffer], 
                 [obstacle[2][0]+collision_buffer, obstacle[2][1]-collision_buffer], [obstacle[3][0]-collision_buffer, obstacle[3][1]-collision_buffer]]
        outline = Polygon(inflation, fill=False, facecolor='gray')
        ax.add_patch(polygon)
        ax.add_patch(outline)
    
    
    
    state_colors = ['green','blue','red','brown','purple']

    def update(frame):
        start_time = time.time()

        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numModel):
            globalcomms[i] = VoronoiDecentralised2[i].sendComms()

        comms_time = time.time()

        for i in range(numModel):
            VoronoiDecentralised2[i].step(globalcomms)

        steps_time = time.time()
        
        #particle label
        global texts
        for text in texts:
            text.remove()
        texts = []
        
        positions_x = [VoronoiDecentralised2[i].position[0] for i in range(numModel)]
        positions_y = [VoronoiDecentralised2[i].position[1] for i in range(numModel)]
        centroid_positions_x = [VoronoiDecentralised2[i].centroid_target[0] for i in range(numModel)]
        centroid_positions_y = [VoronoiDecentralised2[i].centroid_target[1] for i in range(numModel)]
        
        # Update all particle positions at once
        global floorLines
        global boundLines
        global voronoi_polygons
        for i in range(numModel):
            modelaxs[i].set_data([positions_x[i]], [positions_y[i]])
            modelaxs[i].set_color(state_colors[VoronoiDecentralised2[i].state])
            #TODO: use enumerate to do all of this at once
            if voronoi_markings:
                centroidtargets[i].set_data([centroid_positions_x[i]], [centroid_positions_y[i]])
                floorLines[i].set_ydata([VoronoiDecentralised2[i].plotFloor])
                boundLines[i].set_ydata([VoronoiDecentralised2[i].plotSize[1]])
            # texts.append(ax.text(0.1, VoronoiDecentralised2[i].plotFloor+0.1, str(i), ha='center', va='bottom'))        
            # texts.append(ax.text(0.1, VoronoiDecentralised2[i].plotSize[1]+0.1, str(i), ha='center', va='bottom'))        
            # Plot Voronoi region as polygon
            if VoronoiDecentralised2[i].region is not None and len(VoronoiDecentralised2[i].region) > 0 and voronoi_markings:
                # fill=True, alpha=0.2, color=state_colors[VoronoiDecentralised2[i].state])
                voronoi_polygons[i].set_xy(VoronoiDecentralised2[i].region)
            
        posState_time = time.time()
        
        # Create all text labels at once
        texts = [ax.text(x, y+0.2, str(VoronoiDecentralised2[i].id), ha='center', va='bottom') 
                for i, (x, y) in enumerate(zip(positions_x, positions_y))]
        
        text_time = time.time()

        if trails:
            for i in range(numModel):
                
                traj_hist[i][0].append(positions_x[i])
                traj_hist[i][1].append(positions_y[i])
                trajs[i].set_data(traj_hist[i][0], traj_hist[i][1])
        
        # Create all connection lines at once
        neighbor_pairs = [(i, n) for i in range(numModel) for n in VoronoiDecentralised2[i].neighbours]
        segments = np.array([[(positions_x[i], positions_y[i]), 
                            (positions_x[j], positions_y[j])] for i, j in neighbor_pairs])
        connections_lines = [LineCollection(segments, colors='g', alpha=0.3)]
        ax.add_collection(connections_lines[0])        

        end_time = time.time()
   
        print(f'{frame}-------------')
        # print(f'Time taken: {end_time - start_time} seconds')
        # print(f'Comms time: {comms_time - start_time} seconds')
        # print(f'Steps time: {steps_time - comms_time} seconds')
        # print(f'Draw time: {end_time - steps_time} seconds')
        # print(f'posState time: {posState_time - steps_time} seconds')
        # print(f'text time: {text_time - posState_time} seconds')
        #print(f'lines time: {lines_time - text_time} seconds')
        #time.sleep(0.2)    
        return modelaxs + connections_lines + texts + trajs + floorLines + boundLines + voronoi_polygons + centroidtargets
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
                if loadmap:
                    f.write(f"Map: {loadpath}/{mapname}\n")
                if loadStartPos:
                    f.write(f"Starting POs: {loadStartingPath}/{positionName}\n")
                f.write(f"Comms Radius: {VoronoiDecentralised2[0].force_radius}\n")
                # f.write(f"Target Distance: {VoronoiDecentralised2[0].target_dist}\n")
                # f.write(f'Modes: {VoronoiDecentralised2[0].modes}\n')
                

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.show()