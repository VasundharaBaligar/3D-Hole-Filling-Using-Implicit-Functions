import numpy as np
import open3d as o3d

def create_synthetic_holes(point_cloud, hole_radius1, hole_center1, hole_radius2, hole_center2):
    # Compute distances from each point to the hole centers
    distances1 = np.linalg.norm(point_cloud[:, :3] - hole_center1, axis=1)
    distances2 = np.linalg.norm(point_cloud[:, :3] - hole_center2, axis=1)

    # Create masks for points inside the holes
    hole_mask1 = distances1 < hole_radius1
    hole_mask2 = distances2 < hole_radius2

    # Combine the hole masks to get the final mask
    final_hole_mask = hole_mask1 | hole_mask2

    # Create a point cloud with the holes
    point_cloud_with_holes = point_cloud[~final_hole_mask]

    return point_cloud_with_holes

# Load point cloud data
data = np.load('C:\\Users\\Vasundhara V.Baligar\\occupancy_networks\\IAE\\datasets\\1a6f615e8b1b5ae4dbbc9440457e303e\\pointcloud\\points.npy')

# Choose points from your dataset as hole centers
hole_center1 = np.array([-2.2253e-01, 2.0251e-01, -1.2720e-01])  # Use coordinates from your dataset
hole_center2 = np.array([-2.8711e-01, 4.2480e-01 , 2.2913e-01])  # Use coordinates from your dataset

# Set parameters for the synthetic holes
hole_radius1 = 0.1  # Adjust as needed
hole_radius2 = 0.15  # Adjust as needed

# Create point cloud with synthetic holes
point_cloud_with_holes = create_synthetic_holes(data, hole_radius1, hole_center1, hole_radius2, hole_center2)

# Save the point cloud with synthetic holes using np.savez
np.savez('point_cloud_with_synthetic_holes.npz', data=point_cloud_with_holes)

# Visualize the original point cloud and the one with synthetic holes
pc_original = o3d.geometry.PointCloud()
pc_original.points = o3d.utility.Vector3dVector(data)

pc_with_holes = o3d.geometry.PointCloud()
pc_with_holes.points = o3d.utility.Vector3dVector(point_cloud_with_holes)

o3d.visualization.draw_geometries([pc_original, pc_with_holes], window_name="Point Clouds with Synthetic Holes")
