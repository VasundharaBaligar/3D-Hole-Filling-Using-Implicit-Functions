import open3d as o3d
import numpy as np

#The following is a bounding box

ply_file_path = "D:\occupancy_networks\PUGAN_occ\camel\iou_00.ply"
point_cloud = o3d.io.read_point_cloud(ply_file_path)

points = np.asarray(point_cloud.points)
print(points.shape)

#The following is

data = np.load('D:\occupancy_networks\PUGAN_occ\camel\points_iou_00.npz')
print(data.keys())

points1 = data['points']
occupancies = data['occupancies']
print(points1.shape, occupancies.shape)


pc = o3d.geometry.PointCloud()
pc.points = o3d.utility.Vector3dVector(points1)

# Save the point cloud as a PLY file
path = "D:\occupancy_networks\PUGAN_occ\camel\camel.ply"
o3d.io.write_point_cloud(path, pc)
