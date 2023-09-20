import open3d as o3d
import json
import numpy as np

def load_landmarks_from_json(json_filename):
    with open(json_filename, 'r') as f:
        landmarks_list = json.load(f)
    landmarks = np.array([[p['x'], p['y'], p['z']] for p in landmarks_list])
    return landmarks

def estimate_normals(point_cloud):
    radius = 0.05
    max_nn = 30
    point_cloud.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )

def align_centroids(source, target):
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    source -= source_centroid
    target -= target_centroid
    return source, target

def save_point_cloud(cloud, filename):
    o3d.io.write_point_cloud(filename, cloud)

def main():
    # Load landmarks from JSON files
    front_landmarks = load_landmarks_from_json("front_landmarks.json")
    right_landmarks = load_landmarks_from_json("right_landmarks.json")
    left_landmarks = load_landmarks_from_json("left_landmarks.json")
    
    # Align centroids
    front_landmarks, right_landmarks = align_centroids(front_landmarks, right_landmarks)
    front_landmarks, left_landmarks = align_centroids(front_landmarks, left_landmarks)

    # Create point clouds
    front_cloud = o3d.geometry.PointCloud()
    right_cloud = o3d.geometry.PointCloud()
    left_cloud = o3d.geometry.PointCloud()

    front_cloud.points = o3d.utility.Vector3dVector(front_landmarks)
    right_cloud.points = o3d.utility.Vector3dVector(right_landmarks)
    left_cloud.points = o3d.utility.Vector3dVector(left_landmarks)

    # Estimate normals
    estimate_normals(front_cloud)
    estimate_normals(right_cloud)
    estimate_normals(left_cloud)

    # ICP settings
    threshold = 200 # Adjust this value based on your specific use-case
    trans_init = np.eye(4)  # Identity matrix

    # Align right_cloud to front_cloud with point-to-plane ICP
    reg_p2p_right = o3d.pipelines.registration.registration_icp(
        right_cloud, front_cloud, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    right_cloud.transform(reg_p2p_right.transformation)
    print("Right cloud to Front cloud")
    print("Fitness:", reg_p2p_right.fitness)
    print("Inlier RMSE:", reg_p2p_right.inlier_rmse)

    # Align left_cloud to front_cloud with point-to-plane ICP
    reg_p2p_left = o3d.pipelines.registration.registration_icp(
        left_cloud, front_cloud, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    left_cloud.transform(reg_p2p_left.transformation)
    print("Left cloud to Front cloud")
    print("Fitness:", reg_p2p_left.fitness)
    print("Inlier RMSE:", reg_p2p_left.inlier_rmse)

    # Visualize aligned point clouds
    o3d.visualization.draw_geometries([front_cloud, right_cloud, left_cloud])

    # Save the aligned point clouds to disk
    save_point_cloud(front_cloud, "aligned_front_cloud.pcd")
    save_point_cloud(right_cloud, "aligned_right_cloud.pcd")
    save_point_cloud(left_cloud, "aligned_left_cloud.pcd")

if __name__ == "__main__":
    main()
