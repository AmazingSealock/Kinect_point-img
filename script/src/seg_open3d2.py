import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
# 读取点云
pcd = o3d.io.read_point_cloud(r"C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\PCD Files\1695129277503548.pcd")

# pcd = o3d.io.read_point_cloud(sample_ply_data.path)
# # Flip it, otherwise the pointcloud will be upside down.
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw([pcd])