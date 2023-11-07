from pygroundsegmentation import GroundPlaneFitting
import numpy as np
ground_estimator = GroundPlaneFitting() #Instantiate one of the Estimators

xyz_pointcloud_path = r'C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\src\example3\pcd\1695129277503548.bin'
xyz_pointcloud = np.fromfile(xyz_pointcloud_path, dtype=np.int32)
xyz_pointcloud = xyz_pointcloud.reshape((-1, 4))
xyz_pointcloud = xyz_pointcloud[:, 0:3]
ground_idxs = ground_estimator.estimate_ground(xyz_pointcloud)
# ground_pcl = xyz_pointcloud[ground_idxs]
label = np.zeros(len(xyz_pointcloud),dtype=np.int32)
label[ground_idxs]=1
# print(label)
label=label.astype(np.int32)
label.tofile(r'C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\src\example3\label\1695129277503548.bin')
