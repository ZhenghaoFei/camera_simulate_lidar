# show the lidar recording in each txt
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
import re
import glob
def scancoodtrans(filename):
	lidar_dep = np.loadtxt(filename,delimiter=",")
	lidar_deg = np.arange(0,360)*np.pi/180
	# print lidar_dep
	x_cood = np.zeros(360)
	y_cood = np.zeros(360)
	for i in range(len(lidar_dep)):
		x_cood[i] = lidar_dep[i]*np.cos(lidar_deg[i])
		y_cood[i] = lidar_dep[i]*np.sin(lidar_deg[i])
	return x_cood,y_cood
# print lidar_np.shape


for i,txtname in enumerate(sorted(glob.iglob('*.txt'))):
	x,y = scancoodtrans(txtname)
	x_label, y_label = x[30:150], y[30:150] 
	plt.figure(i)
	#scan range
	circle=plt.Circle((0,0),6,color='steelblue',alpha=0.3)
	ax=plt.gca()
	ax.add_artist(circle)
	#scan points
	plt.scatter(x,y,color='steelblue',alpha=0.5)
	#label range
	plt.plot([0, 6],[0, 6*np.tan(np.pi/6)],color="r")
	plt.plot([0,-6],[0, 6*np.tan(np.pi/6)],color="r")
	wedge = Wedge((0, 0), 6, 30, 150,color='indianred',alpha=0.5)
	ax.add_patch(wedge)
	#label points
	plt.scatter(x_label,y_label,color='r',alpha=0.9)
	plt.xlim((-6,6))
	plt.ylim((-6,6))
	# plt.scatter(x,y, color='b',alpha=0.3)
	# plt.show()
	figname = str(re.findall("\d+",txtname)[0]) + ".png"
	print figname
	plt.savefig(figname)
	plt.close(i)