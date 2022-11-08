#! /usr/bin/env python
import numpy as np
from cvxopt import matrix, sparse
from cvxopt.solvers import qp, options
from scipy.integrate import odeint
from math import *

options['show_progress'] = False
options['reltol'] = 1e-2 # was e-2
options['feastol'] = 1e-2 # was e-4
options['maxiters'] = 50 # default is 100

# same with controller in coverage control nonuniform
def si_position_controller(xi, positions, x_velocity_gain=0.5, y_velocity_gain=0.5, velocity_magnitude_limit=0.2):
    _,N = np.shape(xi)
    dxi = np.zeros((2, N))

        # Calculate control input
    dxi[0][:] = x_velocity_gain*(positions[0][:]-xi[0][:])
    dxi[1][:] = y_velocity_gain*(positions[1][:]-xi[1][:])

        # Threshold magnitude
    norms = np.linalg.norm(dxi, axis=0)
    idxs = np.where(norms > velocity_magnitude_limit)
    if norms[idxs].size != 0:
        dxi[:, idxs] *= velocity_magnitude_limit/norms[idxs]

    return dxi

'''
barrier_gain: double (controls how quickly agents can approach each other.  lower = slower)
safety_radius: double (how far apart the agents will stay. higher = bigger safety radius)
magnitude_limit: how fast the robot can move linearly.
'''
# same with controller in coverage control nonuniform
def si_barrier_cert(dxi, x, safety_radius, barrier_gain=100, magnitude_limit=0.1): # magnitude_limit=0.25
    N = dxi.shape[1]
    num_constraints = int(comb(N, 2))
    A = np.zeros((num_constraints, 2*N))
    b = np.zeros(num_constraints)
    H = sparse(matrix(2*np.identity(2*N)))

    count = 0
    for i in range(N-1):
        for j in range(i+1, N):
            error = x[:, i] - x[:, j]
            h = (error[0]*error[0] + error[1]*error[1]) - np.power(safety_radius, 2)

            A[count, (2*i, (2*i+1))] = -2*error
            A[count, (2*j, (2*j+1))] = 2*error
            b[count] = barrier_gain*np.power(h, 3)

            count += 1

        # Threshold control inputs before QP
    norms = np.linalg.norm(dxi, 2, 0)
    idxs_to_normalize = (norms > magnitude_limit)
    dxi[:, idxs_to_normalize] *= magnitude_limit/norms[idxs_to_normalize]

    f = -2*np.reshape(dxi, 2*N, order='F')
    result = qp(H, matrix(f), matrix(A), matrix(b))['x']

    return np.reshape(result, (2, -1), order='F')

def diff_equation(y_list, t, e, omega, c_dot):
    ki = 200 # 200
    sum_fu = 0
    coe = 0
    for i in range(1,len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            sum_fu += (y_list[i] + e*cos(coe*omega*t)) * cos(coe*omega*t)
        else:
            coe = int((i+1)/2)
            sum_fu += (y_list[i] + e*sin(coe*omega*t)) * sin(coe*omega*t)
    result = []
    result.append(-ki*e-sum_fu + 20*sin(pi*t))
    for i in range(1,len(y_list)):
        if i%2 == 0:
            coe = int(i/2)
            result.append((-e)*coe*e*omega*cos(coe*omega*t) + (ki*e+c_dot) *sin(coe*omega*t))
        else:
            coe = int((i+1)/2)
            result.append(e*coe*omega*sin(coe*omega*t) + (ki*e+c_dot) *cos(coe*omega*t))
    return np.array(result)

# update it for cbf and cvt program
def cal_tra_fatii_update(new_coords, new_centroids, old_centroids, gain = 3):
    T=gain
    t=np.linspace(0,T,num=1000)
    omega = pi*2/T
    point_lists = []
    #print("new_coords::: ", new_coords)
    #print("new_centroids::: ", new_centroids)
    for i in range(len(new_coords)):
        y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
        y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
        result_x = odeint(diff_equation, y_list_x, t, args=(new_coords[i][0]- new_centroids[i][0], omega, new_centroids[i][0] - old_centroids[i][0]))
        result_y = odeint(diff_equation, y_list_y, t, args=(new_coords[i][1]- new_centroids[i][1], omega, new_centroids[i][1] - old_centroids[i][1]))
        #result_x = odeint(diff_equation, y_list_x, t, args=(coords[i][0]-new_centroids[i][0], omega, new_centroids[i][0] - old_centroids[i][0]))
        #result_y = odeint(diff_equation, y_list_y, t, args=(coords[i][1]-new_centroids[i][1], omega, new_centroids[i][1] - old_centroids[i][1]))
        # output = _odepack.odeint(func, y0, t, args, Dfun, col_deriv, ml, mu,
        result_xt = result_x[:,0]
        result_yt = result_y[:,0]
        new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
        point_lists.append(list(new_result))

    # Here already have sorted for you by getting one of the point_list, rather than giving MANY arrays
    # refer to the original CVT program
    for i in range(len(new_coords)):
        new_coords[i] = point_lists[i][1] # point_lists[i][1] --> point_lists['rows']['column']
        #self.coords[i] = point_lists[i][0] # using this instead will achieve the same result, but it will 'space' out a little 
        
    return new_coords
