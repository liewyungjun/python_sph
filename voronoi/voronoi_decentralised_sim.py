from matplotlib.collections import LineCollection
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

if __name__ == '__main__':
    try:
        numModel = int(sys.argv[1])
    except IndexError:
        numModel = 6
        
    VoronoiDecentralised = []
    obstacles = [[(2,6),(6,6),(6,4),(2,4)]]
    obstacles = []
    plotsize = [10,10]
    plotBounds = [10,3]
    axesScaling = 0.75 #size of plot
    deltaTime = 0.02
    movement_factor = 0.5
    comms_radius = 2.0
    collision_buffer = 0.5
    
    save = False
    results_path = "demo_results"
    savename = "H.mp4"
    frame_length = 1500

    loadmap = True
    mapname = 'basic_1010'
    loadpath = "maps"

    loadStartPos = False
    loadStartingPath = 'demo_starting_pos'
    positionName = "30_middle"

    rowlength = 5

    mass = 1

    trails = False

    if loadmap:
        obstacles = readMap(mapname,loadpath)

    if loadStartPos:
        startingPoints = readStartingPos(positionName,loadStartingPath)
        numModel = len(startingPoints)

    
    for i in range(numModel):
            if loadStartPos:
                VoronoiDecentralised.append(Voronoi_Decentralised(i,[startingPoints[i][0],startingPoints[i][1],0],deltaTime=deltaTime,
                                                                  plotBounds=[plotBounds[0],plotBounds[1]],movement_factor=movement_factor,
                                                                  comms_radius=comms_radius,collision_buffer=collision_buffer,obstacleList=obstacles,numAgents=numModel))
            else:
                VoronoiDecentralised.append(Voronoi_Decentralised(i,[i%rowlength*1*0.75+plotsize[0]/4,i/2 + 0.2,0],deltaTime=deltaTime,
                                                                  plotBounds=[plotBounds[0],plotBounds[1]],movement_factor=movement_factor,
                                                                  comms_radius=comms_radius,collision_buffer=collision_buffer,obstacleList=obstacles,numAgents=numModel))
            
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
    regions = []
    floorLines = [ax.axhline(y=x.plotFloor, color='red', linestyle='-') for x in VoronoiDecentralised]
    boundLines = [ax.axhline(y=x.plotBounds[1], color='blue', linestyle='-') for x in VoronoiDecentralised]
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
    
    
    
    state_colors = ['green', 'blue', 'brown','red','purple']

    def update(frame):
        start_time = time.time()

        global connections_lines
        for lines in connections_lines:
            lines.remove()
        connections_lines = []
        for i in range(numModel):
            globalcomms[i] = VoronoiDecentralised[i].sendComms()
        # for i in range(numModel):
        #     VoronoiDecentralised[i].scanSurroundings(globalcomms)
        comms_time = time.time()
        for i in range(numModel):
            VoronoiDecentralised[i].step(globalcomms)
        steps_time = time.time()
        #particle label
        global texts
        for text in texts:
            text.remove()
        texts = []
        
        positions_x = [VoronoiDecentralised[i].position[0] for i in range(numModel)]
        positions_y = [VoronoiDecentralised[i].position[1] for i in range(numModel)]
        
        # Update all particle positions at once
        global floorLines
        global boundLines
        global voronoi_polygons
        for i in range(numModel):
            modelaxs[i].set_data([positions_x[i]], [positions_y[i]])
            modelaxs[i].set_color(state_colors[VoronoiDecentralised[i].state])
            #TODO: use enumerate to do all of this at once
            floorLines[i].set_ydata([VoronoiDecentralised[i].plotFloor])
            boundLines[i].set_ydata([VoronoiDecentralised[i].plotBounds[1]])
            # texts.append(ax.text(0.1, VoronoiDecentralised[i].plotFloor+0.1, str(i), ha='center', va='bottom'))        
            # texts.append(ax.text(0.1, VoronoiDecentralised[i].plotBounds[1]+0.1, str(i), ha='center', va='bottom'))        
            # Plot Voronoi region as polygon
            if VoronoiDecentralised[i].region is not None and len(VoronoiDecentralised[i].region) > 0:
                # fill=True, alpha=0.2, color=state_colors[VoronoiDecentralised[i].state])
                voronoi_polygons[i].set_xy(VoronoiDecentralised[i].region)
            
        posState_time = time.time()
        
        # Create all text labels at once
        texts = [ax.text(x, y+0.2, str(VoronoiDecentralised[i].id), ha='center', va='bottom') 
                for i, (x, y) in enumerate(zip(positions_x, positions_y))]
        text_time = time.time()
        if trails:
            for i in range(numModel):
                
                traj_hist[i][0].append(positions_x[i])
                traj_hist[i][1].append(positions_y[i])
                trajs[i].set_data(traj_hist[i][0], traj_hist[i][1])
        
        # Create all connection lines at once
        neighbor_pairs = [(i, n) for i in range(numModel) for n in VoronoiDecentralised[i].neighbours]
        segments = np.array([[(positions_x[i], positions_y[i]), 
                            (positions_x[j], positions_y[j])] for i, j in neighbor_pairs])
        connections_lines = [LineCollection(segments, colors='g', alpha=0.3)]
        ax.add_collection(connections_lines[0])        

        
   
        print(f'{frame}-------------')
        return modelaxs + connections_lines + texts + trajs + floorLines + boundLines + voronoi_polygons
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
                f.write(f"Spring Constant: {spring_constant}\n")
                f.write(f"Gravity: {gravity}\n")
                f.write(f"Force Radius: {VoronoiDecentralised[0].force_radius}\n")
                f.write(f"Target Distance: {VoronoiDecentralised[0].target_dist}\n")
                f.write(f'Modes: {VoronoiDecentralised[0].modes}\n')
                

    plt.xlabel("X")
    plt.ylabel("Y")

    plt.show()