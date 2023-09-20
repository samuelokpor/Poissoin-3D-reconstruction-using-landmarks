
import open3d as o3d
import numpy as np

def visualize_geometry(geo):
    o3d.visualization.draw_geometries([geo])

def poisson_reconstruction(pcd):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    pcd.orient_normals_towards_camera_location()
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
    return mesh

def crop_mesh(mesh, min_bound, max_bound):
    vertices = np.asarray(mesh.vertices)
    in_bound = np.all((min_bound <= vertices) & (vertices <= max_bound), axis=1)
    return mesh.select_by_index(np.where(in_bound)[0])

def main():
    front_pcd = o3d.io.read_point_cloud("aligned_front_cloud.pcd")
    left_pcd = o3d.io.read_point_cloud("aligned_left_cloud.pcd")
    right_pcd = o3d.io.read_point_cloud("aligned_right_cloud.pcd")

    combined_pcd = front_pcd + left_pcd + right_pcd

    # Apply Poisson Surface Reconstruction
    mesh = poisson_reconstruction(combined_pcd)

    # Visualize the combined point cloud
    visualize_geometry(combined_pcd)
    
    # Visualize the full mesh
    visualize_geometry(mesh)

    # Crop the mesh
    min_bound = np.array([-0.17, -0.3, -0.1])  # Adjust as needed height of face is middle value
    max_bound = np.array([0.17, 0.3, 0.1])  # Adjust as needed
    cropped_mesh = crop_mesh(mesh, min_bound, max_bound)
    
    # Visualize the cropped mesh
    visualize_geometry(cropped_mesh)

    # Convert the cropped mesh to a denser point cloud
    denser_pcd = cropped_mesh.sample_points_poisson_disk(number_of_points=8500)  # Adjust as needed
    
    # Visualize the denser point cloud
    visualize_geometry(denser_pcd)

    # Save the denser point cloud
    o3d.io.write_point_cloud("denser_cropped_cloud.pcd", denser_pcd)

if __name__ == "__main__":
    main()
