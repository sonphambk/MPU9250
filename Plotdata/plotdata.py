import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
Mag_x_file = open('Mag_x_data.txt','r')
Mag_y_file = open('Mag_y_data.txt','r')
Mag_z_file = open('Mag_z_data.txt','r')


MAG_X = []
MAG_Y = []
MAG_Z = []

for l in Mag_x_file.readlines():  # readlines đọc từng dòng nhưng nó lưu mỗi dòng là 1 phần tử trong list
    temp1 = l.replace('\n','')
    MAG_X.append(temp1)
MAG_X =list(map(float,MAG_X))


for l1 in Mag_y_file.readlines(): # readlines
    temp2 = l1.replace('\n','')
    MAG_Y.append(temp2)
MAG_Y =list(map(float,MAG_Y))

for l2 in Mag_z_file.readlines(): # readlines
    temp3 = l2.replace('\n','')
    MAG_Z.append(temp3)
MAG_Z =list(map(float,MAG_Z))
#print(MAG_X)
#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.scatter(MAG_X, MAG_Y, MAG_Z,c='green',marker = 'o')
plt.scatter(MAG_Y, MAG_Z,color = 'green',marker = 'd')
plt.scatter(MAG_X, MAG_Z,color = 'blue',marker = 'o')
plt.scatter(MAG_X, MAG_Y,color = 'red',marker = '^')
#ax.set_xlabel('Mag x')
#ax.set_ylabel('Mag y')
#ax.set_zlabel('May z')
#plt.xlabel('Mag Y')
#plt.ylabel('Mag Z')
#plt.title('ZY')
plt.show()