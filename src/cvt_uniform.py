#! /usr/bin/env python
import numpy as np
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area, _plot_polygon_collection_with_color
from shapely.geometry import Polygon, Point
import matplotlib.pyplot as plt
from matplotlib.pyplot import MultipleLocator

from shapely import geometry
import math
from scipy.integrate import odeint
import random

''' Uniform only '''

def gen_voronoi_first(coords, outer): # As initialize
    #coords = np.column_stack((pose[0:2])) # This is coords
    area_shape = Polygon(outer)
    # No need to use for pts or points_to_coords(pts) 
    poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(coords, area_shape, accept_n_coord_duplicates=0)
    # density function
    #poly_centroids = density_function(poly_shapes) 
    poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes]) # if density function, the poly centroids have to be centroid calculation
    new_coords = reshape_coords(coords) # it has to be here
    new_centroids = match_pair(poly_shapes, list(new_coords), list(poly_centroids))

    return area_shape, poly_shapes, poly_to_pt_assignments, new_centroids, new_coords
    #return area_shape, poly_shapes, poly_to_pt_assignments

def gen_voronoi_upd(new_coords, outer): # In the while loop
    #coords = np.column_stack((pose[0:2])) # This is coords
    area_shape = Polygon(outer)
    # No need to use for pts or points_to_coords(pts) 
    poly_shapes, pts, poly_to_pt_assignments = voronoi_regions_from_coords(new_coords, area_shape, accept_n_coord_duplicates=0)
    # density function
    #poly_centroids = density_function(poly_shapes) 
    poly_centroids = np.array([p.centroid.coords[0] for p in poly_shapes]) # if density function, the poly centroids have to be centroid calculation
    # talor for the density function
    #new_centroids = match_pair(poly_shapes, new_coords, poly_centroids)
    #old_centroids = new_centroids
    new_coords = reshape_coords(new_coords)
    new_centroids = match_pair(poly_shapes, list(new_coords), list(poly_centroids))

    
    return area_shape, poly_shapes, poly_to_pt_assignments, new_centroids, new_coords
    #return area_shape, poly_shapes, poly_to_pt_assignments

def density_function(poly_shapes):
    numbers = 5000
    target = [-4, -4]
    deviation = 1.5

    x_unit = np.random.normal(target[0], deviation, numbers) # Draw random samples from a normal (Gaussian) distribution.
    y_unit = np.random.normal(target[1], deviation, numbers) # np.random.normal("Mean (“centre”) of the distribution", "Standard deviation (spread or “width”) of the distribution", "Output shape = 10000")

    region_value = value_contribution(x_unit, y_unit, poly_shapes) # the region_value calculate how many 'point' in x-coordinate and y-coordinate
    poly_centroids = centroid_calculation(region_value, poly_shapes) # for the right plot density function, among the 10000 numbers, in the middle there is a centroid
    
    return poly_centroids

# This match pair have already included reshape centroids
# From main_coverage_with_cbf.py
def match_pair(poly_shapes, new_coords, centroids):
    new_centroids = []
    for n in centroids:
        m = Point(n)
        new_centroids.append(n)
    sorted_centroids = []
    points = coords_to_points(new_coords)
    for i, p in enumerate(points):
        for j, poly in enumerate(poly_shapes):
            if p.within(poly):
                pair = new_centroids[j]
                sorted_centroids.append(pair)
    return sorted_centroids # return in CVT format

def reshape_coords(pose):
    coords = []
    for n in pose:
        m = Point(n)
        coords.append(n)
    return coords

def plotting(fig, ax, iter, save_fig=False):
    major_locator=MultipleLocator(4)
    ax.set_xlim([-12,12])
    ax.set_ylim([-12,12])

    #change x,y labels
    y_ticks = np.arange(-4, 100, 4)
    x_ticks = np.arange(-4, 100, 4)
    ax.set_yticklabels(y_ticks)
    ax.set_xticklabels(x_ticks)

    font = {'size':13}
    ax.xaxis.set_major_locator(major_locator)
    ax.yaxis.set_major_locator(major_locator)
    ax.set_xlabel('x(m)', font, labelpad=15)
    ax.set_ylabel('y(m)', font, labelpad=15)    

    if save_fig:
        plt.savefig('/home/robolab/raspi_ws/src/CVT_CBF_gazebo/Data/FIG/FIG_'+str(iter)+'.png')
    #plt.title(str(j)  +"th itr")
    plt.tick_params(labelsize=13) #设置刻度字体大小
    plt.pause(0.001)
    ax.clear()

# it is fine I guess
def value_contribution(x_unit, y_unit, poly_shapes): # x_unit and y_unit is output 10000
    '''
    x_unit and y_unit is output 10000
    '''
    point_value = np.vstack((np.array(x_unit), np.array(y_unit))).T  # rearrange into (x coordinate, y coordinate) like a 10000 output value
    poly_nums = len(poly_shapes) # poly_nums = 8 
    region_value =[[] for i in range(poly_nums)] # region_value = [[], [], [], [], [], [], [], []] depend on how many agent
    for i, p in enumerate(point_value):
        for j, poly in enumerate(poly_shapes):
            point = geometry.Point(p) # turn x_unit and y_unit into Point
            if point.within(poly): # turn x_unit and y_unit (Point form) is within poly_shapes
                region_value[j].append(p) # put turn x_unit and y_unit (Point form) into region_value[j]
    return np.array(region_value, dtype=object)

# it is fine I guess
# for the right plot density function, among the 10000 numbers, in the middle there is a centroid
def centroid_calculation(region_value, poly_shapes):
    '''
    for the right plot density function, among the 10000 numbers, in the middle there is a centroid
    '''
    sum_value = []
    for i in range(len(poly_shapes)):
        init = [0,0]
        for j in region_value[i]:
            init += j
        sum_value.append(init)
    poly_centroids = []
    for i in range(len(poly_shapes)):
        poly_size = len(region_value[i])
        if poly_size == 0:
            #poly_centroids.append([0,0])
            ori_centroid = [p.centroid.coords[0] for p in poly_shapes]
            poly_centroids.append(list(ori_centroid[i]))
        else:
            poly_dense_x = sum_value[i][0]/poly_size
            poly_dense_y = sum_value[i][1]/poly_size
            poly_centroids.append([poly_dense_x,poly_dense_y])
    return poly_centroids # return 10 x-coordinate and y-coordinate for the right plot poly_centroid 

def plot_new_coords(new_coords, ax, size = 0.2):
    sensor_region = []
    for coord in list(new_coords):
        circ = geometry.Point(coord[0], coord[1]).buffer(size, cap_style=1)
        sensor_region.append(circ)
    
    _plot_polygon_collection_with_color(ax, sensor_region, color='blue', alpha=1, zorder=10)

def plot_sensor_range(new_coords, ax, radius):
    sensor_region = []
    radius = radius/2
    for coord in list(new_coords):
        circ = geometry.Point(coord[0], coord[1]).buffer(radius, cap_style=1)
        sensor_region.append(circ)
    
    _plot_polygon_collection_with_color(ax, sensor_region, color='red', alpha=0.3, zorder=10)
