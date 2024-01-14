import open3d as o3d
import numpy as np

def fit_plane_ransac(point_cloud, distance_threshold=0.017, max_iterations=5000):
    # Extracting the points to a numpy array
    points = np.asarray(point_cloud.points)

    # Number of points in the point cloud
    num_points = points.shape[0]

    # Initialising the best plane
    best_plane_model = None
    best_inliers = []

    for iteration in range(max_iterations):
        # Randomly sample three points to form a plane hypothesis
        random_indices = np.random.choice(num_points, 3, replace=False)
        sample_points = points[random_indices, :]

        # Compute the plane model using the sampled points
        d_sample1 = sample_points[1] - sample_points[0]
        d_sample2 = sample_points[2] - sample_points[0]
        normal_vector = np.cross(d_sample1, d_sample2)
        normal_vector /= np.linalg.norm(normal_vector)

        # Plane model: [A, B, C, D], where A, B, C are the normal vector components
        # and D is the negated dot product of the normal with any point on the plane
        D = -normal_vector.dot(sample_points[0])
        plane_model = np.concatenate((normal_vector, np.array([D])))

        # Compute the distances from all points to the plane
        coeff = plane_model[:-1]
        D = plane_model[-1]
        distances = np.abs(points @ coeff + D) / np.linalg.norm(coeff)


        # Find inliers within the distance threshold set
        inliers = np.where(distances < distance_threshold)[0]

        # Finding the model with the most inliers. The best model.
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_plane_model = plane_model

    return best_plane_model, best_inliers

# Extracting the demo point cloud by Open3D
pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

# Fitting a plane using RANSAC
plane_model, inliers = fit_plane_ransac(pcd)

# Inlier and outlier point clouds
inlier_cloud = pcd.select_by_index(inliers)
outlier_cloud = pcd.select_by_index(inliers, invert=True)

inlier_cloud.paint_uniform_color([1, 0, 0])         #Coloring inlier points red

# Visualization of the point cloud and the fitted plane
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=1,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])

