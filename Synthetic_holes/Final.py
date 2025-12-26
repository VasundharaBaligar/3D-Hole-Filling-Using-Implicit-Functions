import numpy as np
import open3d as o3d
from plyfile import PlyData
import os

### Specify the folder containing the input files
##input_folder = r'D:\occupancy_networks\PU_Net_Occ\star_2\pointcloud'
##
### Specify the output folder where you want to save the .ply files
##output_folder = r'D:\occupancy_networks\PU_Net_Occ\star_2\pointcloud_ply'
##
### Create the output folder if it doesn't exist
##if not os.path.exists(output_folder):
##    os.makedirs(output_folder)
##
### Iterate over all files in the input folder
##for filename in os.listdir(input_folder):
##    if filename.endswith(".npz"):
##        # Load the data from the .npz file
##        file_path = os.path.join(input_folder, filename)
##        data = np.load(file_path)
##
##        # Load the point cloud data
##        points_data = data["points"]
##        normals_data = data["normals"]
##
##        # Create a point cloud object
##        pcd = o3d.geometry.PointCloud()
##        pcd.points = o3d.utility.Vector3dVector(points_data)
##
##        # Optionally set normals if available
##        if normals_data is not None:
##            pcd.normals = o3d.utility.Vector3dVector(normals_data)
##
##        # Specify the output file path
##        output_file_path = os.path.join(output_folder, filename.replace(".npz", ".ply"))
##
##        # Save the point cloud as a .ply file
##        o3d.io.write_point_cloud(output_file_path, pcd)
##
##        print(f"Successfully converted and saved {filename.replace('.npz', '.ply')}")
##
##print("All files converted and saved successfully.")



# ***************************************************************************************************************************************
# Create hole
# ***************************************************************************************************************************************



def savePC(pc_xyz, path, save_name):
    pc_obj = o3d.geometry.PointCloud()
    pc_obj.points = o3d.utility.Vector3dVector(pc_xyz)
    pc_obj.normals = o3d.utility.Vector3dVector(pc_xyz)
    status = o3d.io.write_point_cloud(os.path.join(path, f"{save_name}.ply"), pc_obj)
    return status

# Specify the path to your PLY file
file_path = "D:\occupancy_networks\PU_Net_Occ\\star_2\pointcloud_ply\pointcloud_04.ply"

# Output directory
desti = "D:\occupancy_networks\PU_Net_Occ\\star_2\pointcloud_hole"
if not os.path.exists(desti):
    os.makedirs(desti)

with open(file_path, 'rb') as f:
    plydata = PlyData.read(f)

x = np.expand_dims(plydata.elements[0].data['x'], axis=-1)
y = np.expand_dims(plydata.elements[0].data['y'], axis=-1)
z = np.expand_dims(plydata.elements[0].data['z'], axis=-1)
pc = np.hstack([x, y, z])

# Specify the number of holes
num_holes = 2

for i in range(num_holes):
    # Randomly select a point as the center of the hole
    center = pc[np.random.choice(len(pc))]
    
    # Specify the radius of the hole
    hole_radius = 0.06 # Adjust this value as needed
    
    # Identify points within the hole radius and remove them
    distances = np.linalg.norm(pc - center, axis=1)
    hole_mask = distances < hole_radius
    pc = pc[~hole_mask]  # Remove points within the hole radius

print(pc.shape)
status = savePC(pc, desti, "pointcloud_04_hole")
print("{}/{}".format(desti, "pointcloud_04_hole"))



# ***************************************************************************************************************************************
# Convert ply to npz
# ***************************************************************************************************************************************

# Load the .ply file
ply_file_path = "D:\occupancy_networks\PU_Net_Occ\\star_2\pointcloud_hole\pointcloud_04_hole.ply"
point_cloud = o3d.io.read_point_cloud(ply_file_path)

# Extract the points and normals as NumPy arrays
points = np.asarray(point_cloud.points)
normals = np.asarray(point_cloud.normals)

# Save points and normals into a single .npz file
npz_file_path = "D:\\occupancy_networks\\PU_Net_Occ\\star_2\\pointcloud_NPZ\\pointcloud_04_hole.npz"
np.savez(npz_file_path, points=points, normals=normals)

print("Points and normals saved ")





















