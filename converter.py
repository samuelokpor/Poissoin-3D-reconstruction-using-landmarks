import open3d as o3d

def convert_pcd_to_ply(pcd_path, ply_path):
    # Read the .pcd file
    pcd = o3d.io.read_point_cloud(pcd_path)
    
    # Write the point cloud object to a .ply file
    o3d.io.write_point_cloud(ply_path, pcd)

if __name__ == "__main__":
    # Replace with your file paths
    pcd_path = "./denser_cropped_cloud.pcd"
    ply_path = "./denser_cropped_cloud.ply"
    
    convert_pcd_to_ply(pcd_path, ply_path)
