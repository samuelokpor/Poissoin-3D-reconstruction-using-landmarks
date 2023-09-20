import bpy
import open3d as o3d
import numpy as np

# Function to load PCD file into Blender
def load_pcd_into_blender(file_path):
    # Load the point cloud data using Open3D
    pcd = o3d.io.read_point_cloud(file_path)
    
    # Convert to numpy array and reshape
    points = np.asarray(pcd.points).reshape(-1, 3)
    
    # Create a new mesh
    mesh = bpy.data.meshes.new(name="New_Object_Mesh")
    obj = bpy.data.objects.new("New_Object", mesh)

    # Link it to scene
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    # Construct the mesh
    mesh.from_pydata(points, [], [])
    mesh.update()

# Clear mesh objects in the scene
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.object.select_by_type(type='MESH')
bpy.ops.object.delete()

# Load your .pcd file into Blender
load_pcd_into_blender("path/to/your/denser_cropped_cloud.pcd")

# Here you would need to programmatically set up UV mapping.
# This generally involves unwrapping the mesh, loading a texture, 
# and then assigning that texture to the UV map.

# UV Unwrapping (Simple Example)
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.uv.unwrap(method='ANGLE_BASED', margin=0.001)
bpy.ops.object.mode_set(mode='OBJECT')

# Load texture (Assume texture.png is your texture file)
img = bpy.data.images.load("path/to/texture.png")

# Create a texture and assign the image
tex = bpy.data.textures.new('PCD_Texture', type='IMAGE')
tex.image = img

# Create a material and link the texture to it
mat = bpy.data.materials.new(name="PCD_Material")

# Link texture to material
mtex = mat.texture_slots.add()
mtex.texture = tex

# Assign material to object
bpy.context.object.data.materials.append(mat)
