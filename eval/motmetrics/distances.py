# py-motmetrics - Metrics for multiple object tracker (MOT) benchmarking.
# https://github.com/cheind/py-motmetrics/
#
# MIT License
# Copyright (c) 2017-2020 Christoph Heindl, Jack Valmadre and others.
# See LICENSE file for terms.

"""Functions for comparing predictions and ground-truth."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import functools
import concurrent.futures

import os
import numpy as np
import cv2
import open3d as o3d
from shapely.geometry import Polygon, Point

from motmetrics import math_util
from shapely.vectorized import contains

def norm2squared_matrix(objs, hyps, max_d2=float('inf')):
    """Computes the squared Euclidean distance matrix between object and hypothesis points.

    Params
    ------
    objs : NxM array
        Object points of dim M in rows
    hyps : KxM array
        Hypothesis points of dim M in rows

    Kwargs
    ------
    max_d2 : float
        Maximum tolerable squared Euclidean distance. Object / hypothesis points
        with larger distance are set to np.nan signalling do-not-pair. Defaults
        to +inf

    Returns
    -------
    C : NxK array
        Distance matrix containing pairwise distances or np.nan.
    """

    objs = np.atleast_2d(objs).astype(float)
    hyps = np.atleast_2d(hyps).astype(float)

    if objs.size == 0 or hyps.size == 0:
        return np.empty((0, 0))

    assert hyps.shape[1] == objs.shape[1], "Dimension mismatch"

    delta = objs[:, np.newaxis] - hyps[np.newaxis, :]
    # print(hyps.shape[1])
    C = np.sum(delta ** 2, axis=-1)

    C[C > max_d2] = np.nan
    return C


def rect_min_max(r):
    min_pt = r[..., :2]
    size = r[..., 2:]
    max_pt = min_pt + size
    return min_pt, max_pt


def boxiou(a, b):
    """Computes IOU of two rectangles."""

    # print("a",a.shape,"b",b.shape)
    a_min, a_max = rect_min_max(a)
    b_min, b_max = rect_min_max(b)
    # Compute intersection.
    i_min = np.maximum(a_min, b_min)
    i_max = np.minimum(a_max, b_max)
    i_size = np.maximum(i_max - i_min, 0)
    i_vol = np.prod(i_size, axis=-1)
    # Get volume of union.
    a_size = np.maximum(a_max - a_min, 0)
    b_size = np.maximum(b_max - b_min, 0)
    a_vol = np.prod(a_size, axis=-1)
    b_vol = np.prod(b_size, axis=-1)
    u_vol = a_vol + b_vol - i_vol
    return np.where(i_vol == 0, np.zeros_like(i_vol, dtype=np.float64),
                    math_util.quiet_divide(i_vol, u_vol))




def count_points_in_polygon(points, polygon):
    points = np.array(points)
    return np.sum(contains(polygon, points[:, 0], points[:, 1]))

def calculate_iou(rect1_params, rect2_params, pcd):
    intersection_status, intersection_points = cv2.rotatedRectangleIntersection(tuple(rect1_params), tuple(rect2_params))

    poly1 = Polygon(cv2.boxPoints(rect1_params))
    poly2 = Polygon(cv2.boxPoints(rect2_params))

    area1 = poly1.area
    area2 = poly2.area

    if intersection_status == 1 and intersection_points is not None and len(intersection_points) > 2:
        flattened_intersection_points = [tuple(point[0]) for point in intersection_points]
        intersection_polygon = Polygon(flattened_intersection_points)
    elif intersection_status == 2:
        intersection_polygon = poly1 if area1 < area2 else poly2
    else:
        return 0.0

    # Automatically detect the number of CPU cores
    cpu_count = os.cpu_count() or 4  

    # Use up to 80% of available cores to avoid overloading the system
    cpu_count = int(cpu_count * 0.8)
    chunk_size = max(len(pcd) // cpu_count, 1000) 
    chunks = [pcd[i:i + chunk_size] for i in range(0, len(pcd), chunk_size)]

    with concurrent.futures.ProcessPoolExecutor(max_workers=cpu_count) as executor:
        points_in_poly1 = sum(executor.map(functools.partial(count_points_in_polygon, polygon=poly1), chunks))
        points_in_poly2 = sum(executor.map(functools.partial(count_points_in_polygon, polygon=poly2), chunks))
        points_in_intersection = sum(executor.map(functools.partial(count_points_in_polygon, polygon=intersection_polygon), chunks))

    union_count = points_in_poly1 + points_in_poly2 - points_in_intersection
    return points_in_intersection / union_count if union_count > 0 else 0.0




def rotated_boxiou(a, b,frame,pcd_path):
    # print(a.shape, "and", b.shape)

    filename = f"{frame:03}.pcd"
    pcd_path = os.path.join(pcd_path, filename)


    
    

    pcd = o3d.io.read_point_cloud(pcd_path)
    pcd_points = np.asarray(pcd.points)


    a_broadcast, b_broadcast = np.broadcast_arrays(a, b)
    
    ious = np.zeros(a_broadcast.shape[:-1])

    it = np.nditer(ious, flags=['multi_index'], op_flags=['writeonly'])
    while not it.finished:
        idx = it.multi_index
        rect1 = ((a_broadcast[idx][0], a_broadcast[idx][1]), (a_broadcast[idx][2], a_broadcast[idx][3]),a_broadcast[idx][4]/3.14159*180)

        rect2 = ((b_broadcast[idx][0], b_broadcast[idx][1]), (b_broadcast[idx][2], b_broadcast[idx][3]), b_broadcast[idx][4]/3.14159*180)


        width = b_broadcast[idx][2]
        height = b_broadcast[idx][3]
        depth = 5
        rotation = [0, 0, b_broadcast[idx][4]/3.14159*180]  
        translation = [b_broadcast[idx][0], b_broadcast[idx][1], 0] 

        int_pts = cv2.rotatedRectangleIntersection(rect1, rect2)[1]
        if int_pts is not None:


            it[0] = calculate_iou(rect1, rect2, pcd_points[:, :2]) 

        else:
            it[0] = 0.0

        it.iternext()

    return ious



def iou_matrix(objs, hyps,pcd_path,frame, max_iou=1.):
    """Computes 'intersection over union (IoU)' distance matrix between object and hypothesis rectangles.

    The IoU is computed as

        IoU(a,b) = 1. - isect(a, b) / union(a, b)

    where isect(a,b) is the area of intersection of two rectangles and union(a, b) the area of union. The
    IoU is bounded between zero and one. 0 when the rectangles overlap perfectly and 1 when the overlap is
    zero.

    Params
    ------
    objs : Nx4 array
        Object rectangles (x,y,w,h) in rows
    hyps : Kx4 array
        Hypothesis rectangles (x,y,w,h) in rows

    Kwargs
    ------
    max_iou : float
        Maximum tolerable overlap distance. Object / hypothesis points
        with larger distance are set to np.nan signalling do-not-pair. Defaults
        to 0.5

    Returns
    -------
    C : NxK array
        Distance matrix containing pairwise distances or np.nan.
    """


    # print("frame:",frame)


    if np.size(objs) == 0 or np.size(hyps) == 0:
        return np.empty((0, 0))

    objs = np.asfarray(objs)
    hyps = np.asfarray(hyps)

    iou = rotated_boxiou(objs[:, None], hyps[None, :],frame,pcd_path)
    dist = 1 - iou
    return np.where(dist > 1-max_iou, np.nan, dist)
