import numpy as np
import open3d as o3d
import os
from matplotlib import pyplot as plt
from plyfile import PlyData


data = np.load('D:\occupancy_networks\cbc946b4f4c022305e524bb779a94481\pointcloud.npz')

# Load the data from each .npy file
points_data = data["points"]
normals_data = data["normals"]

# Create a point cloud object
pcd = o3d.geometry.PointCloud()

# Set point coordinates (adjust dimensions if needed)
pcd.points = o3d.utility.Vector3dVector(points_data)

# Optionally set normals if available
if normals_data is not None:
    pcd.normals = o3d.utility.Vector3dVector(normals_data)
    #print(normals_data.shape)

output_folder = "D:\occupancy_networks\cbc946b4f4c022305e524bb779a94481"


# Save the combined point cloud as a .ply file in the specified folder
output_file_path = os.path.join(output_folder, "pointcloud_car.ply")
o3d.io.write_point_cloud(output_file_path, pcd)

print("Successfully converted and saved the merged.ply!")

# ***************************************************************************************************************************************
# Create hole
# ***************************************************************************************************************************************



##def savePC(pc_xyz, path, save_name):
##    pc_obj = o3d.geometry.PointCloud()
##    pc_obj.points = o3d.utility.Vector3dVector(pc_xyz)
##    pc_obj.normals = o3d.utility.Vector3dVector(pc_xyz)
##    status = o3d.io.write_point_cloud(os.path.join(path, f"{save_name}.ply"), pc_obj)
##    return status
##
### Specify the path to your PLY file
##file_path = "D:\occupancy_networks\ShapeNet\\02933112\\fe8c34a5275804d451f8aaa850306632\pointcloud.ply"
##
### Output directory
##desti = "D:\occupancy_networks\ShapeNet\\02933112\\fe8c34a5275804d451f8aaa850306632"
##if not os.path.exists(desti):
##    os.makedirs(desti)
##
##with open(file_path, 'rb') as f:
##    plydata = PlyData.read(f)
##
##x = np.expand_dims(plydata.elements[0].data['x'], axis=-1)
##y = np.expand_dims(plydata.elements[0].data['y'], axis=-1)
##z = np.expand_dims(plydata.elements[0].data['z'], axis=-1)
##pc = np.hstack([x, y, z])
##
### Specify the number of holes
##num_holes = 1
##
##for i in range(num_holes):
##    # Randomly select a point as the center of the hole
##    center = pc[np.random.choice(len(pc))]
##    
##    # Specify the radius of the hole
##    hole_radius = 0.12 # Adjust this value as needed
##    
##    # Identify points within the hole radius and remove them
##    distances = np.linalg.norm(pc - center, axis=1)
##    hole_mask = distances < hole_radius
##    pc = pc[~hole_mask]  # Remove points within the hole radius
##
##print(pc.shape)
##status = savePC(pc, desti, "pointcloud_cut")
##print("{}/{}".format(desti, "pointcloud_cut"))
##
##
##
### ***************************************************************************************************************************************
### Convert ply to npz
### ***************************************************************************************************************************************
##
### Load the .ply file
##ply_file_path = "D:\occupancy_networks\ShapeNet\\02933112\\fe8c34a5275804d451f8aaa850306632\\pointcloud_cut.ply"
##point_cloud = o3d.io.read_point_cloud(ply_file_path)
##
### Extract the points and normals as NumPy arrays
##points = np.asarray(point_cloud.points)
##normals = np.asarray(point_cloud.normals)
##
### Save points and normals into a single .npz file
##npz_file_path = "D:\occupancy_networks\ShapeNet\\02933112\\pc_cut\\pointcloud.npz"
##np.savez(npz_file_path, points=points, normals=normals)
##
##print("Points and normals saved ")




