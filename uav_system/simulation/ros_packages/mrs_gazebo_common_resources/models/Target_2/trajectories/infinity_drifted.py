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

x = 5*np.sin(t)
y = 5*np.sin(t)*np.cos(t)
plt.plot(x,y)
plt.show()
z = np.ones(100)*0.99 * -1
v_x = 5*np.cos(t)
v_y = 5*(np.cos(t)**2 - np.sin(t)**2)
theta = np.arctan(v_x/v_y)*180/np.pi
vel = np.round(np.sqrt(v_x**2+v_y**2),4)
pitch = np.zeros(100)
roll = np.zeros(100)
speed = np.ones(100)
#plt.plot(temps,v_x)
#plt.plot(temps,v_y)
ix = np.linspace(0,1,100)
plt.plot(ix,(theta))
theta_prime = theta
for i in range(len(theta)):
    if i > 12:
        theta_prime[i] = theta_prime[i] + 180

    if i > 37:
        theta_prime[i] = theta_prime[i] + 180
    if i > 61:
        theta_prime[i] = theta_prime[i] -180
    if i > 86:
        theta_prime[i] = theta_prime[i] -180

theta_prime = -theta_prime

# plt.plot(temps,(theta_prime))      
yaw = np.round(theta_prime/360*2*np.pi,3)
plt.plot(ix,(theta_prime))
plt.show()




a = 5
with open("trajectory_infbbmlfsqlfsdkjlmfsqjlkfqs.txt", "w") as f:
    for i in range(100):
        f.write(str(x[i]) + " " + str(y[i]) + " " + str(z[i]) + " " + str(roll[i]) + " " + str(pitch[i]) + " " + str(yaw[i]) + " " + str(speed[i]) + "\n")

a = 5