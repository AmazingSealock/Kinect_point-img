import open3d as o3d

# 读取点云
pcd = o3d.io.read_point_cloud(r"C:\Users\maser\Documents\GitHub\SemanticKITTI-visualizer\PCD Files\1695129277503548.pcd")
print(pcd)
# 可视化点云
o3d.visualization.draw_geometries([pcd])