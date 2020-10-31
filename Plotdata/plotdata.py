import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
mag = open('mag_calibrated3.txt','r')
mag_max = [-32767, -32767, -32767]
mag_min = [32767, 32767, 32767]

time = []
mag_x = []
mag_y = []
mag_z = []

for l in mag.readlines():  # readlines đọc từng dòng nhưng nó lưu mỗi dòng là 1 phần tử trong list
    temp1 = l.replace('\n','').split()
    mag_x.append(temp1[0])
    mag_y.append(temp1[1])
    mag_z.append(temp1[2])
    mag_x = list(map(float,mag_x))
    mag_y = list(map(float,mag_y))
    mag_z = list(map(float,mag_z)) 
for x in range(len(mag_x)):
    if mag_x[x] > mag_max[0]:
        mag_max[0] = mag_x[x]
    if mag_x[x] < mag_min[0]:
        mag_min[0] = mag_x[x]
print("max_x : ",mag_max[0])
print("min_x : ",mag_min[0])        

for y in range(len(mag_y)):
    if mag_y[y] > mag_max[1]:
        mag_max[1] = mag_y[y]
    if mag_y[y] < mag_min[1]:
        mag_min[1] = mag_y[y]
print("max_y : ",mag_max[1])
print("min_y : ",mag_min[1])        

for z in range(len(mag_z)):
    if mag_z[z] > mag_max[2]:
        mag_max[2] = mag_z[z]
    if mag_z[z] < mag_min[2]:
        mag_min[2] = mag_z[z]
print("max_z : ",mag_max[2])
print("min_z : ",mag_min[2])        
#t = 0
#tb =0
#for i in range(900):
#    t += gyro_z[i]

#tb = t/900
#print(tb)
#k = 0
#for i in range(900):
#    k += (gyro_z[i]-tb)**2
#var = k/900
#print(var**2)
#print(MAG_X)
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter(mag_x, mag_y, mag_z,c='green',marker = 'o')
plt.figure()
plt.scatter(mag_x, mag_z,color = 'green',marker = '*')
plt.scatter(mag_y, mag_z,color = 'blue',marker = 'o')
plt.scatter(mag_x, mag_y,color = 'red',marker = '^')
#ax.set_xlabel('Mag x')
#ax.set_ylabel('Mag y')
#ax.set_zlabel('May z')
#plt.xlabel('Mag Y')
#plt.ylabel('Mag Z')
#plt.title('ZY')
plt.show()