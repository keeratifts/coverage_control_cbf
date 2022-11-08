#! /usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import numpy as np
from cvt_non_uniform import *
from raspi import *
import time
from transform import *
from controller import *
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area, _plot_polygon_collection_with_color

#Coverage control in certain area using Voronoi-based algorithm & Control barrier function for avoiding the collision.
#This script used for experimenting with simulation.
# This is CBF and CVT program (NON_UNIFORM)

N = 9
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
        pose = getposition(N)
        old_centroids = (np.zeros((N, 2))) # N = 10
        coords = np.column_stack((pose[0:2]))
        safety_radius = 1.2
        target = [0, 0]
        iter = 0
        start = time.time()
        x_traj, y_traj, t, data = setup_csv(N)

        # Voronoi partition
        (area, poly_shape, poly2pt, new_centroids, new_coords, x_unit, y_unit, ori_centroid) = gen_voronoi_first(coords, target) # Voronoi + density function

        while not rospy.is_shutdown():
            plotting_density(fig, ax2, 50, x_unit, y_unit, norm) # from here (putting the plotting of density function in) 
            plotting_voronoi(fig, ax1, iter)
            plt.pause(0.001)
            ax1.clear()
            ax2.clear()

            pose = getposition(N) # three value --> (x,y,z)
            pose_si = uni_to_si_states(pose) # transformer --> (x,y)

            x_traj = np.append(x_traj, pose[0:1], axis=0)
            y_traj = np.append(y_traj, pose[1:2], axis=0)

            coords = np.column_stack((pose[0:2]))


            if(iter % 8 == 0):
                target[0] += 0.08
                target[1] += 0.08
                if target[0] >= 3.4:
                    end = time.time()
                    t.append(end - start)
                    rate.sleep()

                    data.append(t)
                    for i in range(N):
                        data.append(x_traj[:, i])
                        data.append(y_traj[:, i])
                    rospy.signal_shutdown('End of testing')
                    np.savetxt(LOG_DIR+'/test.csv', np.column_stack(data), delimiter=' , ', fmt='%s')
                    pass
            
            if(iter % 3 == 0):
                (area, poly_shape, poly2pt, new_centroids, new_coords, x_unit, y_unit, ori_centroid) = gen_voronoi_upd(coords, target, ori_centroid) # Voronoi + density function

            # CVT controller here
                new_coords = cal_tra_fatii_update(new_coords, new_centroids, old_centroids) # already came in to cvt format --> (x,y)\
                old_centroids = new_centroids # the old_centroid must have the value of the previous iteration of the new_centroids

            plot_voronoi_polys_with_points_in_area(ax1, area, poly_shape, np.array(coords), poly2pt, voronoi_edgecolor='black', points_color='black', 
                                        points_markersize=12, voronoi_and_points_cmap=None)

            #plot_new_coords(new_coords, ax1) #plot goal of each robot
            plot_sensor_range(coords, ax1, safety_radius)

            x_goal = np.dstack(new_coords)[0] 
            dxi = si_position_controller(pose_si, x_goal)
            dxi = si_barrier_cert(dxi, pose_si, safety_radius)
            dxu = si_to_uni_dyn(dxi, pose) # transformer
            k = set_velocities(N, dxu)
            put_velocities(N, k)
            end = time.time()
            t.append(end - start)
            rate.sleep()

            iter += 1

            

    except rospy.ROSInterruptException:
        rospy.signal_shutdown('End of testing')
        pass
