#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

radius = 10#2.5
 
length = 2*np.pi*radius
velocity = 2 

tmax = length/velocity
temps = np.linspace(0,tmax,400)
theta = np.linspace(0,2*np.pi,400)
x = radius*np.cos(theta)
y = radius*np.sin(theta)
yaw = np.round(theta,5)
yaw[0] =  0.00001
#yaw = np.ones(400)*0.0
pitch = np.zeros(400)+0
roll = np.zeros(400)+0
z = np.ones(400)*0.99 * -1
vel = np.ones(400)*2

with open("/home/crismer/Try_tfe/Trajectories/circular_traj.txt", "w") as f:
    for i in range(400):
        f.write(str(x[i]) + " " + str(y[i]) + " " + str(z[i]) + " " + str(roll[i]) + " " + str(pitch[i]) + " " + str(yaw[i]) + " " + str(vel[i]) + "\n")
