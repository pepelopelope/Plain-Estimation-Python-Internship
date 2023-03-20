# -*- coding: utf-8 -*-
"""
Created on Sat Aug  7 13:30:53 2021

@author: pepei
"""

import open3d as o3d
import numpy as np
import matplotlib.cm as colorm
from statistics import mode
import random as ran

plane_array=np.zeros((20, 4))
list_IN=[]
list_IN_RECYCLE=[]
list_colors=np.zeros((20, 3))
pc_all=[]
final_ratio=10
float(final_ratio)

distance_threshold_table = {
        0: 0.06,
        1: 0.06,
        2: 0.025,
        3: 0.025,
        4: 0.025,
        5: 0.025,
        6: 0.025,
        7: 0.025,
        8: 0.025,
        9: 0.025,        
        10: 0.01,
        11: 0.01,        
        12: 0.01,
        13: 0.01,        
        14: 0.01,
        15: 0.01,
        }
def get_dt(decimal):
    return distance_threshold_table.get(decimal,'NA')

eps_table = {
        0: 0.08,
        1: 0.08,
        2: 0.08,
        3: 0.08,
        4: 0.02,
        5: 0.02,
        6: 0.02,
        7: 0.02,
        8: 0.02,
        9: 0.01,
        10: 0.01,
        11: 0.01,
        12: 0.01,
        13: 0.01,
        14: 0.01,
        15: 0.01,
        }
def get_eps(decimal):
    return eps_table.get(decimal,'NA')

eps_table_sec = {
        0: 0.02,
        1: 0.02,
        2: 0.02,
        3: 0.02,
        4: 0.02,
        5: 0.02,
        6: 0.02,
        7: 0.02,
        8: 0.02,
        9: 0.02,
        10: 0.01,
        11: 0.01,
        12: 0.01,
        13: 0.01,
        14: 0.01,
        15: 0.01,
        }
def get_eps_sec(decimal):
    return eps_table_sec.get(decimal,'NA')

"""hacer funcion con cluster"""

#   Read image

print("Read TUM dataset")
color_raw = o3d.io.read_image(
    "TUM_color.png")
depth_raw = o3d.io.read_image(
    "TUM_depth.png")
rgbd_image = o3d.geometry.RGBDImage.create_from_tum_format(color_raw, depth_raw)
print(rgbd_image)
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
        o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

"""
pcd = o3d.io.read_point_cloud("fragment.ply")"""

#   Create a "external points" cloud
outlier_cloud = pcd.select_by_index([], invert=True)
n=0
while (final_ratio>0.51):
    
    #   Start Color
    Color=n+1
    #   Start comparison
    comp=0
    
    #   Clean the Recycle list
    list_IN_RECYCLE=[]
    
    distance_threshold=get_dt(n)
    eps=get_eps(n)
    eps_sec=get_eps_sec(n+2)
    
    #Preprocess
    if (n < 2):
        #   Compute plane
        plane_array[n], inliers = outlier_cloud.segment_plane(distance_threshold,
                                                              ransac_n=3,
                                                              num_iterations=500)
        print(f"Plane equation {n+1}: {plane_array[n][0]}x + {plane_array[n][1]}y + {plane_array[n][2]}z + {plane_array[n][3]} = 0")
        #   Cloud plane
        inlier_cloud = outlier_cloud.select_by_index(inliers)
        centroid=inlier_cloud.get_center()
    
        #   I compare the plane I have with the ones I already scanned before in order to search if its equal to other one
        #   I do it in order to hace every plane in a same color
    
        """for f in range(0,n):
        comp=0
        for c in range(0,3):
            comp=comp+centroid[c]*plane_array[f][c]
        comp=comp+plane_array[f][3]
        if abs(comp) < 0.2:
            Color=f
        print(comp)"""
    
        
               
        #   DBSCAN plane
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
                labels = np.array(
                    inlier_cloud.cluster_dbscan(eps, min_points=10, print_progress=False))
                #   Clean labels
                for i in range(0,len(labels)-1):
                    if labels[i] == -1:
                       labels[i]=ran.random()*100000
                
                #   Search the biggest cluster by applying statistical fashion
                big_cluster=mode(labels)
                #   Extract the cluster into a "Internal points" list
                for i in range(0,len(labels)-1):
                    if labels[i] == big_cluster:
                        list_IN.append(inliers[i])
                        list_IN_RECYCLE.append(inliers[i])
                        
        inlier_cloud = outlier_cloud.select_by_index(list_IN_RECYCLE)
                


    else:
        plane_array[n], inliers = outlier_cloud.segment_plane(100,
                                                                  ransac_n=3,
                                                                  num_iterations=500)        
        #   DBSCAN
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            labels_med = np.array(
                outlier_cloud.cluster_dbscan(eps, min_points=30, print_progress=False))
            for i in range(0,len(labels_med)-1):
                    if labels_med[i] == -1:
                       labels_med[i]=ran.random()*100000
            big_cluster=mode(labels_med)
                #   Extract the cluster into a "Internal points" list
            for i in range(0,len(labels_med)-1):
                if labels_med[i] == big_cluster:
                    list_IN_RECYCLE.append(inliers[i])
                    
        inlier_cloud = outlier_cloud.select_by_index(list_IN_RECYCLE)
        
        list_IN_RECYCLE=[]
            
        plane_array[n], inliers = inlier_cloud.segment_plane(distance_threshold,
                                                                  ransac_n=3,
                                                                  num_iterations=500)
        print(f"Plane equation {n+1}: {plane_array[n][0]}x + {plane_array[n][1]}y + {plane_array[n][2]}z + {plane_array[n][3]} = 0")
        #   Cloud plane

        #  Add the cluster to the inlier cloud
        inlier_cloud = pcd.select_by_index(inliers)
        
        with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
            labels_small = np.array(
                inlier_cloud.cluster_dbscan(eps_sec, min_points=20, print_progress=False))
            for i in range(0,len(labels_small)-1):
                    if labels_small[i] == -1:
                       labels_small[i]=ran.random()*100000
            big_cluster=mode(labels_small)
                #   Extract the cluster into a "Internal points" list
            for i in range(0,len(labels_small)-1):
                if labels_small[i] == big_cluster:
                    list_IN.append(inliers[i])
                    list_IN_RECYCLE.append(inliers[i])
                    
        inlier_cloud = outlier_cloud.select_by_index(list_IN_RECYCLE)
        
    
    #   Paint the inlier cloud
    inlier_cloud.paint_uniform_color([colorm.tab20(Color/20)[0],colorm.tab20(Color/20)[1],colorm.tab20(Color/20)[2]])
    for colornum in range(0,3):
        list_colors[Color-1][colornum]=colorm.tab20(Color/20)[colornum]

    
    #   Update the outlier cloud
    outlier_cloud = pcd.select_by_index(list_IN, invert=True)
    
    #   Update ratio
    final_ratio=len(outlier_cloud.points)/len(pcd.points)

    #   Update the Full Pointcloud
    pc_all.append(inlier_cloud)
    
    #   Add another plane/cloud 
    n=n+1
    
            
#   At the end of the loop, include the outlier cloud
pc_all.append(outlier_cloud)

 #   Visualize the result
o3d.visualization.draw_geometries(pc_all,
                                  zoom=0.8,
                                  front=[-0.4999, -0.1659, -0.8499],
                                  lookat=[2.1813, 2.0619, 2.0999],
                                  up=[0.1204, -0.9852, 0.1215])

"""[outlier_cloud]"""