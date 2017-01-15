# show the lidar recording in each txt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
import re
import glob
def scancoodtrans(lidar_dep):
	lidar_deg = np.arange(30,150)*np.pi/180
	# print lidar_dep
	x_cood = np.zeros(110)
	y_cood = np.zeros(110)
	for i in range(len(lidar_dep)):
		x_cood[i] = lidar_dep[i]*np.cos(lidar_deg[i])
		y_cood[i] = lidar_dep[i]*np.sin(lidar_deg[i])
	return x_cood,y_cood
# print lidar_np.shape

lidar_deps = np.loadtxt("3predict.txt")
print lidar_deps.shape
for i in range(lidar_deps.shape[0]):
	x,y = scancoodtrans(lidar_deps[i,:])
	plt.figure(i)
	#scan range
	circle=plt.Circle((0,0),6,color='steelblue',alpha=0.3)
	ax=plt.gca()
	ax.add_artist(circle)
	#scan points
	plt.scatter(x,y,color='red',alpha=0.5)
	plt.xlim((-6,6))
	plt.ylim((-6,6))
	# plt.scatter(x,y, color='b',alpha=0.3)
	# plt.show()
	figname = "3_" + str(i) + ".png"
	print figname
	plt.savefig(figname)
	plt.close(i)


