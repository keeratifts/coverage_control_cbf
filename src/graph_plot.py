#! /usr/bin/env python

import numpy as np
from math import *
import matplotlib.pyplot as plt

DATA_PATH = '/home/'

def trajectories_plot(log_file_dir):
    plt.style.use('seaborn-ticks')
    plt.figure(1)
    data = np.genfromtxt(log_file_dir+'/test.csv', delimiter=' , ', names=True)
    num = int(len(data[0])/2) #no. of robots
    for i in range(num):
        plt.plot(data['x_traj_'+str(i)], data['y_traj_'+str(i)],marker = 's', markevery = len(data['x_traj_'+str(i)]) + 1, label = 'ROBOT '+ str(i + 1))
    plt.legend(ncol = 2, frameon=True)

    ###### PLOT COORDINATE RESPONSE ######
    # plt.figure(2)
    # plt.subplot(3,1,1)
    # plt.plot(data['x_goal_0'], color = 'red', label = 'reference')
    # plt.plot(data['x_traj_0'], color = 'blue', label ='actual')
    # plt.legend(loc = 4)
    # plt.title('X coordinate response')
    # plt.xlabel('sample time')
    # plt.ylabel('x [m]')
    # plt.xlim(0, len(data['x_goal_0']) + 3)
    # plt.ylim(np.min(data['x_traj_0']) - 0.1, np.max(data['x_traj_0']) + 0.1)
    # plt.grid()

    # plt.figure(2)
    # plt.subplot(3,1,2)
    # plt.plot(y_goal_1, color = 'red', label = 'reference')
    # plt.plot(y_traj_1, color = 'blue', label = 'actual')
    # plt.legend(loc = 4)
    # plt.title('Y coordinate response')
    # plt.xlabel('sample time')
    # plt.ylabel('y [m]')
    # plt.xlim(0, len(y_goal_1) + 3)
    # plt.ylim(np.min(y_traj_1) - 0.075, np.max(y_traj_1) + 0.075)
    # plt.grid()
    #plt.tight_layout()

    plt.show()

    ###### SAVE FIGURE ######
    # plt.savefig('/home/robolab/raspi_ws/src/coverage_control/Data/FIG_'+str(iter)+'.png')

if __name__ == '__main__':
    trajectories_plot(DATA_PATH)