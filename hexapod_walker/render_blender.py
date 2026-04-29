"""Photoreal Cycles render of the hexapod walker.

Must be run *inside Blender* (it imports `bpy`).  The recommended way is
via the wrapper:

    ./render_blender.sh

which (a) checks Blender is installed, (b) builds the assembly STLs via
build_full_assembly.py if they are missing or stale, then (c) calls
Blender headless with this script.

Manual invocation:

    blender --background --factory-startup --python render_blender.py -- \
        --assets assembly/ \
        --out renders/walker.png

The scene is an outdoor showroom: the walker stands on a clean concrete
floor under a soft HDRI-style sky, lit by a single sun + soft world
background.  Camera is a 3/4 view from front-right at ~ 1.6 m
eye-height (the rider's perspective).

Materials are assigned per imported STL category:

    frame.stl    -> brushed satin aluminum
    motors.stl   -> dark anodized aluminum (graphite black, slight metal)
    battery.stl  -> matte black powder-coat
    soft.stl     -> deep black urethane / saddle leather
    rider.stl    -> neutral denim grey
"""

import math
import os
import sys
from pathlib import Path

import bpy  # noqa: F401  -- only available inside Blender


# ---------------------------------------------------------------------------
# Argument parsing (Blender passes our args after `--`)
# ---------------------------------------------------------------------------

def parse_args():
    import argparse
    if "--" in sys.argv:
        argv = sys.argv[sys.argv.index("--") + 1:]
    else:
        argv = []
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--assets", default="assembly",
                   help="Directory containing the category STL files.")
    p.add_argument("--out", default="renders/walker.png",
                   help="Output PNG (resolves relative to hexapod_walker/).")
    p.add_argument("--samples", type=int, default=96,
                   help="Cycles render samples (higher = less noise, slower).")
    p.add_argument("--width",  type=int, default=1600)
    p.add_argument("--height", type=int, default=1000)
    p.add_argument("--device", default="CPU",
                   choices=("CPU", "CUDA", "OPTIX", "METAL", "HIP"),
                   help="Cycles compute device.  Mac users: try METAL.")
    p.add_argument("--engine", default="CYCLES",
                   choices=("CYCLES", "BLENDER_EEVEE_NEXT"),
                   help="Render engine (Cycles is photoreal; EEVEE is real-time).")
    p.add_argument("--exposure", type=float, default=0.5)
    p.add_argument("--sun-strength", type=float, default=4.0,
                   help="Sun light energy in W/m^2.")
    p.add_argument("--sky-strength", type=float, default=0.8,
                   help="World background (sky) strength.")
    p.add_argument("--sun-elevation-deg", type=float, default=55.0)
    p.add_argument("--sun-azimuth-deg",   type=float, default=35.0,
                   help="Sun azimuth measured CCW from +X (default = right-and-back).")
    p.add_argument("--camera-distance", type=float, default=5.5,
                   help="Camera distance from walker centre (m).")
    p.add_argument("--camera-height",   type=float, default=1.4,
                   help="Camera eye-height above the floor (m).")
    p.add_argument("--camera-azimuth-deg", type=float, default=35.0,
                   help="Camera azimuth in degrees from +X.")
    p.add_argument("--camera-pitch-deg",   type=float, default=-2.0,
                   help="Camera pitch (negative = looking slightly down).")
    p.add_argument("--lens", type=float, default=42.0,
                   help="Camera focal length (mm).")
    p.add_argument("--ground", default="#7a7872",
                   help="Hex color for the concrete floor.")
    p.add_argument("--frame-color",   default="#9aa0a6")  # brushed Al
    p.add_argument("--motor-color",   default="#1c1d20")  # graphite black
    p.add_argument("--battery-color", default="#0d0d10")  # matte black
    p.add_argument("--soft-color",    default="#171515")  # urethane / leather
    p.add_argument("--rider-color",   default="#3b4a5e")  # denim grey-blue
    p.add_argument("--sky-zenith",    default="#9bb6cf")  # cool sky blue
    p.add_argument("--sky-horizon",   default="#dfd9c8")  # warm horizon
    return p.parse_args(argv)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def hex_to_rgb(s, gamma=2.2):
    s = s.lstrip("#")
    r = int(s[0:2], 16) / 255.0
    g = int(s[2:4], 16) / 255.0
    b = int(s[4:6], 16) / 255.0
    return (r ** gamma, g ** gamma, b ** gamma, 1.0)


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.lights,
                  bpy.data.cameras, bpy.data.images, bpy.data.textures):
        for item in list(block):
            block.remove(item, do_unlink=True)


def make_metal_material(name, hex_color, *, roughness=0.4, metallic=1.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = hex_to_rgb(hex_color)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    if "Specular IOR Level" in bsdf.inputs:
        bsdf.inputs["Specular IOR Level"].default_value = 0.5
    elif "Specular" in bsdf.inputs:
        bsdf.inputs["Specular"].default_value = 0.5
    mat.node_tree.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def make_diffuse_material(name, hex_color, *, roughness=0.85, metallic=0.0):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = hex_to_rgb(hex_color)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = roughness
    mat.node_tree.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def import_stl(path):
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=str(path))
    else:
        bpy.ops.import_mesh.stl(filepath=str(path))
    return bpy.context.selected_objects[-1]


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    print(f"[render] cwd: {os.getcwd()}")
    print(f"[render] assets: {args.assets}")
    print(f"[render] out: {out}")

    assets = Path(args.assets)
    required = ["frame.stl", "motors.stl", "battery.stl", "soft.stl",
                "rider.stl"]
    for r in required:
        if not (assets / r).exists():
            raise SystemExit(
                f"Missing {assets/r} -- run "
                f"`./run.sh hexapod_walker/build_full_assembly.py` first."
            )

    clear_scene()
    scene = bpy.context.scene
    scene.render.engine = args.engine
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.filepath = out

    scene.view_settings.view_transform = "Filmic"
    scene.view_settings.look = "Medium Contrast"
    scene.view_settings.exposure = args.exposure
    scene.view_settings.gamma = 1.0

    # ---- Cycles ----
    if args.engine == "CYCLES":
        scene.cycles.samples = args.samples
        scene.cycles.use_denoising = True
        if hasattr(scene.cycles, "denoising_prefilter"):
            scene.cycles.denoising_prefilter = "ACCURATE"
        if hasattr(scene.cycles, "denoiser"):
            scene.cycles.denoiser = "OPENIMAGEDENOISE"
        scene.cycles.glossy_bounces = 6
        scene.cycles.transparent_max_bounces = 6
        prefs = bpy.context.preferences.addons["cycles"].preferences
        if args.device != "CPU":
            try:
                prefs.compute_device_type = args.device
                for d in prefs.devices:
                    d.use = True
                scene.cycles.device = "GPU"
                print(f"[render] using {args.device} GPU")
            except Exception as e:
                print(f"[render] could not enable {args.device}: {e}; falling back to CPU")
                scene.cycles.device = "CPU"
        else:
            scene.cycles.device = "CPU"

    # ---- World: gradient sky ----
    world = bpy.data.worlds.new("WalkerWorld")
    scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()
    bg_out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    bg.inputs["Strength"].default_value = args.sky_strength

    # Procedural sky-ish gradient using a Texture Coordinate -> Mapping ->
    # Gradient -> ColorRamp.  Vertical (Z) gradient from horizon (warm)
    # at the bottom to zenith (cool blue) at the top.
    tex_coord = nt.nodes.new("ShaderNodeTexCoord")
    mapping   = nt.nodes.new("ShaderNodeMapping")
    grad      = nt.nodes.new("ShaderNodeTexGradient")
    grad.gradient_type = "EASING"
    ramp      = nt.nodes.new("ShaderNodeValToRGB")
    ramp.color_ramp.elements[0].position = 0.30
    ramp.color_ramp.elements[1].position = 0.85
    ramp.color_ramp.elements[0].color = hex_to_rgb(args.sky_horizon)
    ramp.color_ramp.elements[1].color = hex_to_rgb(args.sky_zenith)

    nt.links.new(tex_coord.outputs["Generated"], mapping.inputs["Vector"])
    nt.links.new(mapping.outputs["Vector"], grad.inputs["Vector"])
    nt.links.new(grad.outputs["Fac"], ramp.inputs["Fac"])
    nt.links.new(ramp.outputs["Color"], bg.inputs["Color"])
    nt.links.new(bg.outputs["Background"], bg_out.inputs["Surface"])

    # ---- Materials ----
    mat_frame   = make_metal_material("Frame_BrushedAl",   args.frame_color,
                                       roughness=0.42, metallic=1.0)
    mat_motor   = make_metal_material("Motor_Anodized",    args.motor_color,
                                       roughness=0.55, metallic=0.7)
    mat_battery = make_diffuse_material("Battery_Matte",   args.battery_color,
                                         roughness=0.78)
    mat_soft    = make_diffuse_material("Soft_RubberLeather", args.soft_color,
                                          roughness=0.92)
    mat_rider   = make_diffuse_material("Rider_Denim",     args.rider_color,
                                         roughness=0.88)
    mat_ground  = make_diffuse_material("Ground_Concrete", args.ground,
                                          roughness=0.95)

    # ---- Import the five category STLs ----
    # The build_full_assembly.py exporter already rotated them to Y-up
    # (= Blender's "Up = Y" import preset).  Blender's *world* uses Z-up,
    # so we re-rotate Y-up -> Z-up by +90 deg about X.  Then convert
    # mm -> m.
    def yup_mm_to_zup_m(obj):
        obj.rotation_euler = (math.radians(90), 0.0, 0.0)
        obj.scale = (0.001, 0.001, 0.001)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True,
                                        scale=True)

    def add_stl(name, mat, sharp=False):
        path = assets / f"{name}.stl"
        print(f"[render] importing {path} ...")
        obj = import_stl(path)
        obj.name = f"Walker_{name}"
        obj.data.materials.append(mat)
        if sharp:
            # Keep flat shading on cubic / boxy assets (battery, motors)
            for poly in obj.data.polygons:
                poly.use_smooth = False
        else:
            bpy.ops.object.shade_smooth()
        yup_mm_to_zup_m(obj)
        return obj

    frame_obj   = add_stl("frame",   mat_frame)
    motor_obj   = add_stl("motors",  mat_motor)
    battery_obj = add_stl("battery", mat_battery, sharp=True)
    soft_obj    = add_stl("soft",    mat_soft)
    rider_obj   = add_stl("rider",   mat_rider)

    # ---- Compute scene bounds (Z-up metres) ----
    bbox = []
    for o in (frame_obj, motor_obj, battery_obj, soft_obj, rider_obj):
        bbox.extend(o.matrix_world @ v.co for v in o.data.vertices)
    xs = [v.x for v in bbox]; ys = [v.y for v in bbox]; zs = [v.z for v in bbox]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    cx = (xmin + xmax) / 2
    cy = (ymin + ymax) / 2
    cz = (zmin + zmax) / 2
    print(f"[render] walker bounds (m, Z-up): "
          f"x[{xmin:.2f},{xmax:.2f}] y[{ymin:.2f},{ymax:.2f}] z[{zmin:.2f},{zmax:.2f}]")

    # ---- Ground plane ----
    bpy.ops.mesh.primitive_plane_add(size=40.0, location=(cx, cy, zmin - 0.001))
    ground = bpy.context.object
    ground.name = "Ground"
    ground.data.materials.append(mat_ground)

    # ---- Sun light (directional) ----
    sun_data = bpy.data.lights.new("Sun", type="SUN")
    sun_data.energy = args.sun_strength
    sun_data.angle  = math.radians(2.0)   # ~ 2 deg disc -> soft real-sun shadows
    sun_data.color  = (1.0, 0.97, 0.92)
    sun = bpy.data.objects.new("Sun", sun_data)
    bpy.context.collection.objects.link(sun)
    el = math.radians(args.sun_elevation_deg)
    az = math.radians(args.sun_azimuth_deg)
    # Build a direction vector from azimuth/elevation, then orient the sun
    # so its -Z points along that direction.
    dx = math.cos(el) * math.cos(az)
    dy = math.cos(el) * math.sin(az)
    dz = math.sin(el)
    # The sun in Blender shines along -Z of its local frame.  Aim -Z at -dir.
    import mathutils
    direction = mathutils.Vector((-dx, -dy, -dz))
    rot = direction.to_track_quat('-Z', 'Y')
    sun.rotation_euler = rot.to_euler()

    # ---- Soft fill (area light) opposite the sun, weaker ----
    fill_data = bpy.data.lights.new("Fill", type="AREA")
    fill_data.energy = args.sun_strength * 60.0  # area W is total, so much higher
    fill_data.size = 6.0
    fill_data.color = (0.85, 0.92, 1.0)   # cool sky bounce
    fill = bpy.data.objects.new("Fill", fill_data)
    bpy.context.collection.objects.link(fill)
    fill_dist = 6.0
    fill_az = az + math.radians(180.0)
    fill.location = (cx + fill_dist * math.cos(fill_az),
                     cy + fill_dist * math.sin(fill_az),
                     zmax + 0.5)
    fill_target = mathutils.Vector((cx - fill.location[0],
                                     cy - fill.location[1],
                                     cz - fill.location[2]))
    fill.rotation_euler = fill_target.to_track_quat('-Z', 'Y').to_euler()

    # ---- Camera ----
    az = math.radians(args.camera_azimuth_deg)
    cam_x = cx + args.camera_distance * math.cos(az)
    cam_y = cy + args.camera_distance * math.sin(az)
    cam_z = zmin + args.camera_height
    bpy.ops.object.camera_add(location=(cam_x, cam_y, cam_z))
    cam = bpy.context.object
    cam.name = "Camera"
    target = (cx, cy, zmin + 0.6 + (zmax - zmin) * 0.25)
    direction = mathutils.Vector((target[0] - cam_x,
                                   target[1] - cam_y,
                                   target[2] - cam_z))
    cam.rotation_euler = direction.to_track_quat('-Z', 'Y').to_euler()
    # Apply optional pitch correction
    cam.rotation_euler.x += math.radians(args.camera_pitch_deg)
    cam.data.lens = args.lens
    scene.camera = cam

    print(f"[render] camera at ({cam_x:.2f},{cam_y:.2f},{cam_z:.2f}) "
          f"-> target ({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})")
    print(f"[render] sun: elevation {args.sun_elevation_deg:.0f} deg, "
          f"azimuth {args.sun_azimuth_deg:.0f} deg, energy {args.sun_strength}")

    # ---- Render ----
    print(f"[render] rendering {args.width}x{args.height} at "
          f"{args.samples} samples -> {out}")
    bpy.ops.render.render(write_still=True)
    print("[render] done.")


if __name__ == "__main__":
    main()
