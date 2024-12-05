import numpy as np

points = np.random.uniform(0, 10, size=(9, 2))
points = np.array([[1,5],[1.5,2],[1.8,8],[2.5,5],[4,2],[5,4],[6.5,2],[7,7],[9,2.5]])
points = np.array([[5,4],[6.5,2],[7,7],[9,2.5]])
reflected_points = np.vstack([
    points,
    points * [-1, 1],  # reflect across x=0
    points * [1, -1],  # reflect across y=0
    [[10 + 10-x[0],x[1]] for x in points ],
    [[x[0],10 + 10-x[1]] for x in points ]
    # points + [10, 0],  # reflect across x=10
    # points + [0, 10],  # reflect across y=10
    # points + [10, 10], # reflect across (10,10)
    # points + [-10, 0], # reflect across x=-10
    # points + [0, -10], # reflect across y=-10
])#points = [[1,2],[3,4],[2,4],[2,3],[2.5,2]]
from scipy.spatial import Voronoi, voronoi_plot_2d

vor = Voronoi(reflected_points)
import matplotlib.pyplot as plt


fig = voronoi_plot_2d(vor)
plt.xlim(0, 10)
plt.ylim(0, 10)

# Calculate and plot centroids
for region in range(len(vor.regions)):
    if not -1 in vor.regions[region] and len(vor.regions[region]) > 0:
        for i in vor.regions[region]:
            if vor.vertices[i][0] <0 or vor.vertices[i][0]>10 or vor.vertices[i][1] <0 or vor.vertices[i][1]>10:
                break
            else:
                polygon = [vor.vertices[i] for i in vor.regions[region]]
                if len(polygon) > 0:
                    centroid = np.mean(polygon, axis=0)
                    plt.plot(centroid[0], centroid[1], 'ko')
                    label = ''
                    for j in range(len(vor.point_region)):
                        if vor.point_region[j] == region:
                            label = j
                    plt.text(centroid[0], centroid[1],str(label), fontsize=12, ha='right')
                    #plt.text(centroid[0], centroid[1],str(region), fontsize=12, ha='right')

# Add vertex index labels
for i, vertex in enumerate(vor.points):
    if not(vertex[0] <0 or vertex[0]>10 or vertex[1]<0 or vertex[1]>10):
        plt.text(vertex[0], vertex[1], str(i), fontsize=8, ha='right')

for i, vertex in enumerate(vor.vertices):
    if not(vertex[0] <0 or vertex[0]>10 or vertex[1] <0 or vertex[1]>10):
        plt.text(vertex[0], vertex[1], str(i), fontsize=8, ha='right')

plt.show()
