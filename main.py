import numpy as np
from DataClass import data_io
from KalmanFilterEq import KalmanFilterEq
import tools
import matplotlib.pyplot as plt

n=4

sensor_data,true_data,LIDARFalse = data_io()
kalmanFilter = KalmanFilterEq(n)
flag= 1
state_est=tools.run(sensor_data,kalmanFilter,flag)
p_x,p_y,Vel_x,Vel_y=tools.get_RMSE(state_est,true_data)

lidar_x_sensor=[]
lidar_y_sensor=[]
x_pred=[]
y_pred=[]
x_true=[]
y_true=[]

radar_x_sensor=[]
radar_y_sensor=[]


i = 0
for s, p, t in zip(sensor_data, state_est, true_data):
    i += 1
    if i % 4 == 0:
        continue
    if s.get_name() == 'LIDAR':
        x, y = s.get_raw()
        lidar_x_sensor.append(x)
        lidar_y_sensor.append(y)
    else:
        rho, phi, drho = s.get_raw()
        x,y,vx,vy=tools.Polar2Cart(rho,phi,drho)
        radar_x_sensor.append(x)
        radar_y_sensor.append(y)

        x, y, vx, vy = p.get_data()
        x_pred.append(x)
        y_pred.append(y)

        x, y, vx, vy = t.get_data()
        x_true.append(x)
        y_true.append(y)
true=[]
pred=[]
lidar=[]
radar=[]
for i in x_true:
    for j in y_true:
        true.append((i,j))
for i in x_pred:
    for j in y_pred:
        pred.append((i,j))
for i in lidar_x_sensor:
    for j in lidar_y_sensor:
        lidar.append((i,j))
for i in radar_x_sensor:
    for j in radar_y_sensor:
        radar.append((i,j))

true=np.asarray(true)
pred=np.asarray(pred)
lidar=np.asarray(lidar)
radar=np.asarray(radar)
#diff_lidar=true-lidar
diff_radar=true-radar
diff=true - pred
diff_2=diff*diff
sum1=diff_2.sum(axis=1)
err= np.sqrt(sum1)
#err_lidar=np.sqrt((diff_lidar*diff_lidar).sum(axis=1))
err_radar=15*np.sqrt((diff_radar*diff_radar).sum(axis=1))
mean=err.sum()/len(err)
print('Mean error of prediction:',mean)
#print ('Mean error of lidar:',err_lidar.sum/len(err_lidar))
print ('Mean error of radar:',err_radar.sum()/len(err_radar))

plt.figure(num=1)
plt.scatter(x_pred, y_pred, label="Predicted-Data", marker="*")
plt.scatter(x_true, y_true, label="True-Data", marker=".")
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.legend()
plt.show()
plt.figure(num=2)
plt.scatter(lidar_x_sensor,lidar_y_sensor,label="Lidar-Sensor-Data",marker="o")
plt.scatter(radar_x_sensor,radar_y_sensor,label="Radar-Sensor-Data",marker="d")
plt.scatter(x_true, y_true, label="True-Data", marker=".")
plt.xlabel('x-position')
plt.ylabel('y-position')
plt.legend()
plt.show()
plt.figure(num=3)
plt.plot(err,label="Error Plot")
plt.xlabel('Measurement number')
plt.ylabel('Error')
plt.legend()
plt.show()
LIDARFalse=np.asarray(LIDARFalse)
errlidar=15*np.sqrt((LIDARFalse*LIDARFalse).sum(axis=1))
plt.plot(errlidar[0:1000],label="Lidar Error Plot")
plt.xlabel('Measurement number')
plt.ylabel('Error')
plt.show()
print(np.mean(errlidar))
plt.plot(err_radar[0:1000],label="Radar Error Plot")
plt.xlabel('Measurement number')
plt.ylabel('Error')
plt.show()