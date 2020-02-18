from math import sin, cos, sqrt
from numpy import arctan2
import numpy as np
import DataClass
from KalmanFilterEq import KalmanFilterEq

P = np.matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1000, 0],
               [0, 0, 0, 1000]])

Q = np.matrix(np.zeros([4, 4]))
F = np.matrix(np.eye(4))
LIDAR_R=np.matrix([[0.001, 0], [0, 0.001]])
RADAR_R=np.matrix([[0.01, 0, 0],
                     [0, 1.0e-6, 0],
                     [0, 0, 0.01]])

LIDAR_H = np.matrix([[1, 0, 0, 0],
                     [0, 1, 0, 0]])


def start(data_pt,kalmanFilter,flag):
    timestamp = data_pt.get_time()
    x = np.matrix([data_pt.get_data()]).T
    kalmanFilter.x = x
    kalmanFilter.P = P
    kalmanFilter.F = F
    kalmanFilter.Q = Q
    flag=0
    return timestamp


def Q_Update(dt):
    dt_2 = pow(dt, 2)
    dt_3 = pow(dt, 3)
    dt_4 = pow(dt, 4)

    Q = np.array(
        [[dt_4 / 4, 0, dt_3 / 2, 0], [0, dt_4 / 4, 0, dt_3 / 2], [dt_3 / 2, 0, dt_2, 0], [0, dt_3 / 2, 0, dt_2]])

    return Q*5


def kalman(data_pt,kalmanFilter,timestamp):
    dt=time_dif(timestamp,data_pt.get_time())
    timestamp=data_pt.get_time()

    kalmanFilter.F[0,2],kalmanFilter.F[1,3]=dt,dt
    kalmanFilter.Q=Q_Update(dt)
    kalmanFilter.predict()
    x=kalmanFilter.x
    z=np.matrix(data_pt.get_raw()).T

    if DataClass.data1.get_name()=='RADAR':
        p_x,p_y,Vel_x,Vel_y=  x[0, 0], x[1, 0], x[2, 0], x[3, 0]
        radius, arg, velocity= Cart2Polar(p_x,p_y,Vel_x,Vel_y)
        H=jacobian(p_x,p_y,Vel_x,Vel_y)
        Hx=np.matrix([[radius, arg, velocity]]).T
        R=RADAR_R
    elif DataClass.data1.get_name()=='LIDAR':
        H=LIDAR_H
        Hx=LIDAR_H*x
        R=LIDAR_R

    kalmanFilter.update(z,H,Hx,R)
    return timestamp

def run(data, kalmanFilter,flag):
    state_estimations = []
    for data_pt in data:
        if flag:
            timestamp = start(data_pt, kalmanFilter,flag)
        else:
            timestamp = kalman(data_pt, kalmanFilter, timestamp)
        x = kalmanFilter.x

        p_x, p_y, Vel_x, Vel_y = x[0, 0], x[1, 0], x[2, 0], x[3, 0]

        g = {'time': data_pt.get_time(),
             'sensor': 'estimate',
             'x': p_x,
             'y': p_y,
             'Vel_x': Vel_x,
             'Vel_x': Vel_x,
             'Vel_y': Vel_y}

        state_estimation = DataClass.data1(g)
        state_estimations.append(state_estimation)

    return state_estimations


def Polar2Cart(radius, arg, velocity):
    x = radius * cos(arg)
    y = radius * sin(arg)
    vel_x = velocity * cos(arg)
    vel_y = velocity * sin(arg)

    return x, y, vel_x, vel_y


def Cart2Polar(x, y, vel_x, vel_y):
    radius = sqrt(pow(x, 2) + pow(y, 2))
    arg = arctan2(y, x)

    if (radius < 0.001):
        return 0, 0, 0
    velocity = ((x * vel_x) + (y * vel_y)) / radius

    return radius, arg, velocity


def jacobian(pos_x,pos_y,vel_x,vel_y):
    threshold = 0.001
    dsquare= pow(pos_x,2) + pow(pos_y,2)
    d= sqrt(dsquare)
    dcube=d * dsquare

    # if denominator tends to zero return zero matrix to avoid the divide by zero error
    if (dsquare<threshold):
        H= np.ndarray(np.zeros([3,4]))
    else :
        H00= pos_x/d
        H01= pos_y/d
        H10= -pos_y/dsquare
        H11= pos_x/dsquare
        H20= pos_y * (vel_x*pos_y- vel_y*pos_x)/ dcube
        H21= pos_x * (vel_y*pos_x- vel_x*pos_y)/ dcube
        H= np.matrix([[H00, H01,0,0],
                      [H10,H11,0,0],
                      [H20,H21,H00,H01]])
    return H

#return time diffrence in microseconds
def time_dif(t1,t2):
    return (t2,t1)/1000000.0


def get_RMSE(predictions, ground_truth):

    pxs, pys, vxs, vys = [], [], [], []

    for p, t in zip(predictions, ground_truth):
        ppx, ppy, pvx, pvy = p.get_data()
        tpx, tpy, tvx, tvy = t.get_data()

        pxs += [(ppx - tpx) * (ppx - tpx)]
        pys += [(ppy - tpy) * (ppy - tpy)]
        vxs += [(pvx - tvx) * (pvx - tvx)]
        vys += [(pvy - tvy) * (pvy - tvy)]

    px, py = sqrt(np.mean(pxs)), sqrt(np.mean(pys))
    vx, vy = sqrt(np.mean(vxs)), sqrt(np.mean(vys))

    return px, py, vx, vy



