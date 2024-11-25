

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
