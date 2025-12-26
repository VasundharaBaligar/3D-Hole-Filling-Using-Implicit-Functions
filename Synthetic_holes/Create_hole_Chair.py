import numpy as np
import open3d as o3d
import os
from plyfile import PlyData

def savePC(pc_xyz, path, save_name):
    pc_obj = o3d.geometry.PointCloud()
    pc_obj.points = o3d.utility.Vector3dVector(pc_xyz)
    pc_obj.normals = o3d.utility.Vector3dVector(pc_xyz)
    status = o3d.io.write_point_cloud(os.path.join(path, f"{save_name}.ply"), pc_obj)
    return status

# Specify the path to your PLY file
file_path = "D:\occupancy_networks\Merged_car.ply"

# Output directory
desti = "./"
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
status = savePC(pc, desti, "Merged_car_hole")
print("{}/{}".format(desti, "Merged_car_hole"))



