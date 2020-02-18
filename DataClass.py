from tools import Polar2Cart
import copy
import numpy as np
class data1:

    def __init__(self, data_point):
        self.name = data_point['sensor']
        self.time = data_point['time']
        self.data = []
        self.all = data_point

        if (self.name == 'LIDAR'):
            self.data = [data_point['x'], data_point['y'], 0, 0]
            self.raw = [data_point['x'], data_point['y']]

        elif (self.name == 'RADAR'):
            x, y, Vel_x, Vel_y = Polar2Cart(data_point['radius'], data_point['arg'], data_point['Velocity'])
            self.data = [x, y, Vel_x, Vel_y]
            self.raw = [data_point['radius'], data_point['arg'], data_point['Velocity']]

        else:
            self.data = [data_point['x'], data_point['y'], data_point['Vel_x'], data_point['Vel_y']]
            self.raw = copy.deepcopy(self.data)

    def get_name(self):
        return self.name

    def get_time(self):
        return self.time

    def get_raw(self):
        return self.raw

    def get_data(self):
        return self.data

    def get(self):
        return self.all

RadarFalse=[]
LIDARFalse=[]

def data_io():
    sensor_data = []
    true_data = []

    with open(r'C:\Users\Chinnu\Downloads\data-1.txt', 'r') as data:
        lines = data.readlines()

    for line in lines:
        sep = line.split()

        if sep[0] == 'L':
            LIDARFalse.append(((float(sep[1]))-float(sep[4]),float(sep[2])-float(sep[5])))
            sensor_data_pt = data1({'sensor': 'LIDAR', 'x':float(sep[1]), 'y': float(sep[2]), 'time':float(sep[3])})

            true_data_pt = data1(
                {'sensor': 'Ground', 'time': float(sep[3]), 'x': float(sep[4]), 'y': float(sep[5]), 'Vel_x': float(sep[6]), 'Vel_y': float(sep[7])})

        else:

            sensor_data_pt = data1(
                {'sensor': 'RADAR','time': float(sep[4]), 'radius': float(sep[1]), 'arg': float(sep[2]), 'Velocity': float(sep[3])})
            true_data_pt = data1(
                {'sensor': 'Ground', 'time': float(sep[4]), 'x': float(sep[5]), 'y': float(sep[6]), 'Vel_x': float(sep[7]), 'Vel_y': float(sep[8])})

        sensor_data.append(sensor_data_pt)
        true_data.append(true_data_pt)
    return sensor_data, true_data,LIDARFalse

