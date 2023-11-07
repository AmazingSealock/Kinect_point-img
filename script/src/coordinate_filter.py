import numpy as np
import os

def filter_points_Z(points,thre):
    points_new = []
    for p in points:
        if (p[...,2])<=thre:
            points_new.append(p)
    return points_new

def filter_points_Y(points,thre):
    points_new = []
    for p in points:
        if abs(p[...,1])<=thre:
            points_new.append(p)
    return points_new

def filter_points_X(points,thre):
    points_new = []
    for p in points:
        if abs(p[...,0])<=thre:
            points_new.append(p)
    return points_new

pcd_dir = r'C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\BIN_Files_Full'
save_path=r'C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\BIN Files New'
for name in os.listdir(pcd_dir):
    pcd_path = os.path.join(pcd_dir,name)
    points = np.fromfile(pcd_path, dtype=np.float32)
    points = points.reshape((-1, 4))
    coordinate = points
    
    print("MAX X:",max(coordinate[...,0])," MIN X:",min(coordinate[...,0]))
    print("MAX Y:",max(coordinate[...,1])," MIN Y:",min(coordinate[...,1]))
    print("MAX Z:",max(coordinate[...,2])," MIN Z:",min(coordinate[...,2]))
    
    
    coordinate = filter_points_Z(coordinate, thre=2)
    coordinate = filter_points_X(coordinate, thre=15)
    coordinate = filter_points_Y(coordinate, thre=15)
    coordinate = np.array(coordinate)
    coordinate.reshape((-1))
    coordinate.tofile(os.path.join(save_path,name))