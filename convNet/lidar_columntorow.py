import glob
import os
import numpy as np
from os import listdir
from os.path import isfile, join

mypath = './data/data1/'
data_lidar = sorted(glob.glob(mypath + '*txt'), key=os.path.basename)

for i in range(len(data_lidar)):
	data = np.loadtxt(data_lidar[i])
	data = data.reshape(1, -1)

	np.savetxt(data_lidar[i],data, delimiter=',')

# filename = './data/data1/5911492232.txt'
# data = np.loadtxt(filename)
# data = data.reshape(1, -1)
# print data.shape

# np.savetxt('haha.txt', data, delimiter=',')