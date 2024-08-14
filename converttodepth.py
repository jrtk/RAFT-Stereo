import numpy as np
import open3d as o3d

# Load disparity map
disparity = np.load('disparity.npy')

# Camera parameters (example values, replace with actual values)
f = 3995.335  # focal length in pixels
B = .0503047   # baseline in meters
cx = 1309.208   # principal point x-coordinate
cy = 982.134   # principal point y-coordinate

# Compute depth map
depth = (f * B) / (disparity + 1e-6)  # add small value to avoid division by zero

dh, dw = depth.shape

print ("depth shape  height ", dh, " depth width ", dw)

# Generate point cloud
h, w = disparity.shape  # h  1948  disparity w  2804
print ("disparith h ", h ,  " disparity w ", w)
i, j = np.meshgrid(np.arange(w), np.arange(h))
z = depth
x = (i - cx) * z / f
y = (j - cy) * z / f

# Stack to get point cloud (Nx3)
points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

# Filter out points with infinite depth (invalid points)
valid_points = points[np.isfinite(points).all(axis=1)]

# Create Open3D point cloud object
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(valid_points)

# Save point cloud to a file (e.g., PLY format)
o3d.io.write_point_cloud("point_cloud.ply", pcd)

# Optionally visualize the point cloud
o3d.visualization.draw_geometries([pcd])

