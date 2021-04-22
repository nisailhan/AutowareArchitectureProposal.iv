#!/usr/bin/env python

import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import roslib

def main(dimension, accel_or_brake):
    script_dir = roslib.packages.get_pkg_dir('raw_vehicle_cmd_converter')
    csv_dir = script_dir + '/data/default'

    vel_list = []
    stroke_list = []
    acc_list = []
    with open(csv_dir + '/{}_map.csv'.format(accel_or_brake)) as f:
        for l_idx, l in enumerate(f.readlines()):
            w = l.split(',')
            w[-1] = w[-1][:-1]
            if l_idx == 0:
                vel_list = [float(e) * 3600 / 1000 for e in w[1:]]
            else:
                stroke_list.append(float(w[0]))
                acc_list.append([float(e) for e in w[1:]])

    plt.rcParams["font.size"] = 30
    if dimension == 2:
        plt.title('{} map'.format(accel_or_brake))
        plt.xlabel('stroke')
        plt.ylabel('acceleration [m/s^2]')

        for vel_idx, vel in enumerate(vel_list):
            plt.plot(stroke_list, np.array(acc_list).T[vel_idx], label='vel={}[km/h]'.format(vel))
        plt.legend()
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title("{} map".format(accel_or_brake))
        ax.set_xlabel('stroke')
        ax.set_ylabel('vel [km/h]')
        ax.set_zlabel('acceleration [m/s^2]')

        print([stroke_list for e in vel_list])
        print([vel_list for e in stroke_list])
        surf = ax.plot_surface(np.array([stroke_list for e in vel_list]).T, [vel_list for e in stroke_list], np.array(acc_list), cmap='bwr')
        fig.colorbar(surf, shrink=0.5, aspect=10)

    plt.show()

if __name__ == '__main__':
    dimension = 2 if '2d' in sys.argv else 3
    accel_or_brake = 'accel' if 'accel' in sys.argv else 'brake'

    main(dimension, accel_or_brake)
