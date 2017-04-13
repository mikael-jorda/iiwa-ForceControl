#!/usr/bin/env python

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np

# data files
folder = 'debug_ori_controller'
header = ''
position_file = folder + '/' + header + 'cartesian_positions.txt';
orientation_file = folder + '/' + header + 'orientation.txt';

positions = np.loadtxt(position_file,skiprows=1)
orientations = np.loadtxt(orientation_file,skiprows=1)

time = positions[:,0]

x_desired = positions[:,1:4]
x = positions[:,4::]

quat_desired = orientations[:,1:5]
quat = orientations[:,5::]

plt.figure(1)
plt.plot(time,x,'r',label="X current")
plt.plot(time,x_desired,'k', label="X desired")
# plt.axis([time[0], time[-1], 0.6, 0.61])
plt.legend()

plt.figure(2)
plt.plot(time,quat[:,2:4],'r',label="q current")
plt.plot(time,quat_desired[:,2:4],'k', label="q desired")
plt.legend()

plt.show()