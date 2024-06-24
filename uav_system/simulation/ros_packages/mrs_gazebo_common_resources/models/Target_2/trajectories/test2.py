import matplotlib.pyplot as plt
import numpy as np



a = np.linspace(0,5,100)
b = np.linalg.norm(a)

for i in range(5):
    img_number = str(i)
    im = "image_"
    name_image = im+img_number
    print(name_image)
    print(type(name_image))
    



########
tmax = 15
temps = np.linspace(0,tmax,100)
t = 2*np.pi/tmax*temps

#scale = 5* 2 / (3 - np.cos(2*t))
#x = np.round(scale * np.cos(t),5)
#y = np.round(scale * np.sin(2*t) / 2,5)
x = 5*np.cos(t)
dx_dt = -5*np.sin(t)

y = 4*np.sin(2*t)/2
dy_dt = 4*np.cos(2*t)

plt.plot(x,y)
plt.show()
z = np.ones(100)*0.99 * -1
#v_x = 4*np.pi/3 * (np.sin(t)*(np.cos(2*t-3)-3)-2*(np.sin(2*t)*np.cos(t)))/(3*(3-np.cos(2*t))**2)
#v_y = -4*np.pi*(np.sin(2*t)**2 + np.cos(2*t)**2-3*np.cos(2*t))/(3*(3-np.cos(2*t))**2)
#theta = np.arctan(v_x/v_y)*180/np.pi
theta = np.arctan2(dx_dt,dy_dt)

plt.plot(t,dx_dt)
plt.plot(t,dy_dt)
plt.show()

plt.plot(t,theta*360/(2*np.pi))
plt.show()

#vel = np.round(np.sqrt(v_x**2+v_y**2),4)





pitch = np.zeros(100)
roll = np.zeros(100)
speed = np.ones(100)

theta_prime = np.abs(theta)

for i in range(len(theta)):
   if t[i] > np.pi : 
       theta_prime[i] = theta_prime[i]+ np.pi
    

plt.plot(temps,(theta_prime))      
yaw = np.round(-theta,3)
x  =np.round(x,5)
y = np.round(y,5)

plt.show()




a = 5
with open("/home/crismer/Try_tfe/Trajectories/infinty_traj.txt", "w") as f:
    for i in range(100):
        f.write(str(x[i]) + " " + str(y[i]) + " " + str(z[i]) + " " + str(roll[i]) + " " + str(pitch[i]) + " " + str(yaw[i]) + " " + str(speed[i]) + "\n")

a = 5