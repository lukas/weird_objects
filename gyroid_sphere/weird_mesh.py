import numpy as np
from skimage import measure
import trimesh

# -----------------------
# Parameters (tune these)
# -----------------------
resolution = 100        # grid resolution (higher = smoother, slower)
size = 2.0              # overall bounding box
gyroid_scale = 6.0      # frequency of pattern
thickness = 0.15        # thickness of surface band
sphere_radius = 0.9     # radius of final object

# -----------------------
# Create 3D grid
# -----------------------
x = np.linspace(-size, size, resolution)
y = np.linspace(-size, size, resolution)
z = np.linspace(-size, size, resolution)
X, Y, Z = np.meshgrid(x, y, z, indexing='ij')

# -----------------------
# Gyroid function
# -----------------------
def gyroid(x, y, z):
    return (
        np.sin(x) * np.cos(y) +
        np.sin(y) * np.cos(z) +
        np.sin(z) * np.cos(x)
    )

# Scale coordinates
Xg = X * gyroid_scale
Yg = Y * gyroid_scale
Zg = Z * gyroid_scale

G = gyroid(Xg, Yg, Zg)

# -----------------------
# Create thin shell (|G| < thickness)
# -----------------------
gyroid_shell = np.abs(G) - thickness

# -----------------------
# Sphere mask
# -----------------------
sphere = np.sqrt(X**2 + Y**2 + Z**2) - sphere_radius

# Combine: inside both
field = np.maximum(gyroid_shell, sphere)

# -----------------------
# Extract mesh
# -----------------------
verts, faces, normals, _ = measure.marching_cubes(field, level=0)

# Scale vertices to real coordinates
scale = (2 * size) / resolution
verts = verts * scale - size

# -----------------------
# Create mesh
# -----------------------
mesh = trimesh.Trimesh(vertices=verts, faces=faces)

# -----------------------
# Export STL
# -----------------------
mesh.export("gyroid_sphere.stl")

print("Saved gyroid_sphere.stl")