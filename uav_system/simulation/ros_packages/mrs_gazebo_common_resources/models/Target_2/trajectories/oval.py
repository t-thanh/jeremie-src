import matplotlib.pyplot as plt
import numpy as np

#### cercle 1 
x1 = np.linspace(3,5,50)
y1 = np.sqrt(4 - (x1-3)**2)
derivative1 = -(x1-3) / np.sqrt(4- (x1-3)**2)

angle1 = np.arctan2(derivative1,1) + np.pi/2

###### straight line 
x2 = np.linspace(-2,2.999999999,10)
y2 = np.ones(10)*2
angle2 = np.ones(10)*np.pi/2

### ellipse
x3 = np.linspace(-4.99999999999,-1.999999999999,50)
y3 = np.sqrt(((1 - ((x3 +2)**2)/9)) *4 )
deriv3 = -2*(x3 +2)/(3*np.sqrt(-x3**2 - 4*x3 +5))
angle3 = np.arctan2(deriv3,1) + np.pi/2

y4 = -np.sqrt(((1 - ((x3 +2)**2)/9))* 4 )
deriv4 = 2*(x3 +2)/(3*np.sqrt(-x3**2 - 4*x3 +5))
angle4 = np.arctan2(deriv4,1) +3* np.pi/2



#### straight 2
x5 = np.linspace(-2,(5-np.sqrt(2)/2)-0.0000000001,20)
y5 = np.ones(20)*-2
angle5 = np.ones(20)* 3*np.pi/2 
######circle2 
x6 = np.linspace((5-np.sqrt(2)/2),4.99999999,20)
y6 = -np.sqrt((0.5-(x6-(5-np.sqrt(2)/2))**2)) - (2 - np.sqrt(2)/2)
deriv6 = (x6 + 1/np.sqrt(2) - 5 ) / (np.sqrt(-x6**2 + 8.588579*x6 - 17.9289))
angle6 = np.arctan2(deriv6,1) + 3*np.pi/2 


####### straight 3 
x7 = np.ones(10)*5
y7 = np.linspace(- (2 -np.sqrt(2)/2),0,10)
angle7 = np.ones(10)*2*np.pi

#####
#plt.plot(x1,angle1*360/(2*np.pi))
#plt.plot(x2,angle2*360/(2*np.pi))
#plt.plot(x3,angle3*360/(2*np.pi))

#plt.plot(x3,angle4*360/(2*np.pi))

#plt.plot(x5,angle5*360/(2*np.pi))
#plt.plot(x6,angle6*360/(2*np.pi))
#plt.plot(x7,angle7*360/(2*np.pi))
#plt.show()
yaw =  np.round(np.concatenate((np.flip(angle1),np.flip(angle2),np.flip(angle3),angle4,angle5 ,angle6 ,angle7)),3)
pitch = np.zeros(len(yaw))
roll = np.zeros(len(yaw))
x =  np.round(np.concatenate((np.flip(x1),np.flip(x2),np.flip(x3),x3,x5 ,x6 ,x7)),4)
y = np.round(np.concatenate((np.flip(y1),np.flip(y2),np.flip(y3),y4,y5 ,y6 ,y7)),4)
z = np.ones(len(yaw))* - 0.99
v = np.ones(len(yaw))*1

with open("/home/crismer/Try_tfe/Trajectories/oval_traj.txt", "w") as f:
    for i in range(len(yaw)):
        if i > 0 and i < len(yaw) - len(angle7) +1 : 
            if x[i-1] == x[i]: 
                continue  
            
        f.write(str(x[i]) + " " + str(y[i]) + " " + str(z[i]) + " " + str(roll[i]) + " " + str(pitch[i]) + " " + str(yaw[i]) + " " + str(v[i]) + "\n")



#####
plt.plot(x1,y1)
plt.plot(x2,y2)
plt.plot(x3,y3)
plt.plot(x3,y4)
plt.plot(x5,y5)
plt.plot(x6,y6)
plt.plot(x7,y7)
plt .show()
a = 5