import open3d as o3d

# 读取点云
pcd = o3d.io.read_point_cloud(r"/home/action/Documents/pcdfile/11.6/pcd/1699261980888196.pcd")
print(pcd)
# 可视化点云
o3d.visualization.draw_geometries([pcd])