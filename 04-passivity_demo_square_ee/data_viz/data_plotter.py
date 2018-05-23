#!/usr/bin/env python

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
# if len(sys.argv) < 2:
# 	print "Give the name of the file to read as an argument\n"
# 	exit()

# file = np.loadtxt(sys.argv[1] ,skiprows=1)

file_force = np.loadtxt("debug_robot/tests_force.txt", skiprows=1)
file_moment = np.loadtxt("debug_robot/tests_moment.txt", skiprows=1)

time = file_force[:,0]
force = file_force[:,1:4]
desired_force = file_force[:,4:7]
Rc_force = file_force[:,7]

moment = file_moment[:,1:4]
desired_moment = file_moment[:,4:7]
Rc_moment = file_moment[:,7]

plt.figure(1)
# plt.plot(time, force[:,2],'r',label="Force")
plt.plot(force[:,2],'r',label="Force")
# plt.plot(time, desired_force[:,2],'k', label="Desired Force")
plt.plot(desired_force[:,2],'k', label="Desired Force")
plt.legend()

plt.figure(2)
plt.plot(Rc_force)
plt.axis([0, np.size(Rc_force), -0.05, 1.05])

plt.figure(3)
# plt.plot(time, force[:,2],'r',label="Force")
plt.plot(moment[:,0:2],'r',label="moment")
# plt.plot(time, desired_force[:,2],'k', label="Desired Force")
plt.plot(desired_moment[:,0:2],'k', label="Desired moment")
plt.legend()

plt.figure(4)
plt.plot(Rc_moment)
plt.axis([0, np.size(Rc_moment), -0.05, 1.05])

plt.show()