import matplotlib.pyplot as plt
import numpy as np
import csv

x_fit = np.array([0,30,50,70,100,130,160,180,220,250,280,300,350]) 
yfit = np.array([0.4,0.3,0.3,0.2,0.1,0.3,0.4,0.2,0.2,0.3,0.1,0.2,0.3]) *7
polyn = np.polyfit(x_fit,yfit,11)
c = np.linspace(0,300,301)
bbbb = np.polyval(polyn,c)
plt.plot(c,bbbb)
plt.show()
with open('/home/crismer/Try_tfe/Trajectories/blabla.csv', 'r') as csvfile:
        reader = csv.reader(csvfile)
        data_list = list(reader)

data_array = np.array(data_list[1:-1], dtype=np.float32)

x = data_array[0:-1:5,1]*5
y = data_array[0:-1:5,2]*5
z = np.zeros(len(x)) - 0.99

yaw = data_array[0:-1:5,3]
pitch = np.zeros(len(x))
roll = np.zeros(len(x))
sss = np.linspace(0,len(x),len(x))
speed = np.polyval(polyn,sss)

plt.plot(x,y)
plt.show()

a = 5
with open("/home/crismer/Try_tfe/Trajectories/full_traj.txt", "w") as f:
    for i in range(300):
        f.write(str(x[i]) + " " + str(y[i]) + " " + str(z[i]) + " " + str(roll[i]) + " " + str(pitch[i]) + " " + str(yaw[i]) + " " + str(speed[i]) + "\n")

a = 5
