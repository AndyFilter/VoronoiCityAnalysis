from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import matplotlib.pyplot as plt

NUM_POINTS = 100

points = np.random.rand(NUM_POINTS, 2) * 20 - 10

# Do the thing
vor = Voronoi(points)

fig = voronoi_plot_2d(vor)
plt.show()

regions = []

# filter out empty and "outside" cells
for r in vor.regions:
    if len(r) > 0 and -1 not in r:
        regions.append(r)

print(vor.regions)
print(vor.ridge_points)
print(regions)

out_points = open("points.dat", "w+")
out_vtx = open("vertices.dat", "w+")
out_edges = open("edges.dat", "w+")
out_regions = open("regions.dat", "w+")

out_points.write("%d\n" % len(points))
out_vtx.write("%d\n" % len(vor.vertices))
out_edges.write("%d\n" % len(vor.ridge_points))
out_regions.write("%d\n" % len(regions))

out_points.writelines(f"{p[0]}\t{p[1]}\n" for p in points)
out_vtx.writelines(f"{p[0]}\t{p[1]}\n" for p in vor.vertices)
out_edges.writelines(f"{e[0]}\t{e[1]}\n" for e in vor.ridge_points)
out_regions.writelines("\t".join(str(x) for x in r) + "\n" for r in regions)
