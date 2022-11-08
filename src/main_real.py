#! /usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from cvt_real import *
from raspi import *
import time
from transform import *
from controller import *
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area, _plot_polygon_collection_with_color

#Coverage control in certain area using Voronoi-based algorithm & Control barrier function for avoiding the collision.
#This script used for experimenting with real robot.
# This is CBF and CVT program (NON-UNIFORM)

N = 6
LOG_DIR = '/home/'

def setup_csv(N):
    x_traj = np.empty((0, N), float)
    y_traj = np.empty((0, N), float)
    t = ['time']
    data = []
    x, y = [], []
    for i in range(N):
        x.append('x_traj_'+str(i))
        y.append('y_traj_'+str(i))
    x_traj = np.append(x_traj, np.array([x]), axis=0)
    y_traj = np.append(y_traj, np.array([y]), axis=0)
    return x_traj, y_traj, t, data

if __name__ == '__main__':
    try:
        rospy.init_node('control_node', anonymous = False)
        rate = rospy.Rate(100)
        fig = plt.figure(figsize=(5.7,5))
        fig.subplots_adjust(wspace=0.5, hspace=0, top=0.95, bottom=0.15)
        ax1 = fig.add_subplot(121,projection='scatter_density')
        ax2 = fig.add_subplot(122,projection='scatter_density')
        ax1.axis('scaled')
        ax2.axis('scaled')
        norm = ImageNormalize(vmin=0., vmax=1000, stretch=LogStretch())

        ''' Initialize it first '''
        pose = get_robot_position(N)
        safety_radius = 0.36
        old_centroids = (np.zeros((N, 2))) # N = 10
        coords = np.column_stack((pose[0:2]))
        x_traj, y_traj, t, data = setup_csv(N)
        target = [0, 0]
        iter = 0
        end = 0
        start = time.time()

        # Voronoi partition
        (area, poly_shape, poly2pt, new_centroids, new_coords, x_unit, y_unit, ori_centroid) = gen_voronoi_first(coords, target) # Voronoi + density function

        while not rospy.is_shutdown():
            plotting_density(fig, ax2, 50, x_unit, y_unit, norm) # from here (putting the plotting of density function in) 
            plotting_voronoi(fig, ax1, iter, save_fig=True)
            plt.pause(0.001)
            ax1.clear()
            ax2.clear()
            pose = get_robot_position(N) # three value --> (x,y,z)
            pose_si = uni_to_si_states(pose) # transformer --> (x,y)

            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)

            coords = np.column_stack((pose[0:2]))

            # new_coords have to be update from the bottom
            # voronoi_partition
            nrm = (np.linalg.norm(np.column_stack((new_coords)) - pose_si))
            # print (nrm)
            print (coords)
            if(end - start > 60):
                target[0] = -0.25
                target[1] = 0.25
                if(end - start > 120):
                    target[0] = -0.25
                    target[1] = -0.25
                if(end - start > 180):
                    target[0] = -0.5
                    target[1] = -0.5
                if (end - start > 240):
                    end = time.time()
                    t.append(end - start)
                    data.append(t)
                    for i in range(N):
                        data.append(x_traj[:, i])
                        data.append(y_traj[:, i])
                    np.savetxt(LOG_DIR+'/test.csv', np.column_stack(data), delimiter=' , ', fmt='%s')
                    rospy.signal_shutdown('End of testing')
                    pass
            
            if(iter % 2 == 0):
                (area, poly_shape, poly2pt, new_centroids, new_coords, x_unit, y_unit, ori_centroid) = gen_voronoi_upd(coords, target, ori_centroid) # Voronoi + density function
            
            # CVT controller here
                new_coords = cal_tra_fatii_update(new_coords, new_centroids, old_centroids, gain=16) # already came in to cvt format --> (x,y)\
                old_centroids = new_centroids # the old_centroid must have the value of the previous iteration of the new_centroids

            plot_voronoi_polys_with_points_in_area(ax1, area, poly_shape, np.array(coords), poly2pt, voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=12, voronoi_and_points_cmap=None)

            plot_sensor_range(coords, ax1, safety_radius)

            x_goal = np.dstack(new_coords)[0] 
            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si)
            dxu = si_to_uni_dyn(dxi, pose) # transformer
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            end = time.time()
            t.append(end - start)
            iter += 1    

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
