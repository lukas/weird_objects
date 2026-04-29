"""Photorealistic chandelier render using Blender's Cycles ray-tracer.

This script must be run *inside Blender* (it imports `bpy`).  The
recommended way is via the wrapper:

    ./render_blender.sh

which (a) checks Blender is installed, (b) runs simulate_chandelier.py
with --export-blender to produce panels.ply + led_positions.json, then
(c) calls Blender headless with this script.

Manual invocation:

    blender --background --factory-startup --python render_blender.py -- \
        --assets blender_assets/ \
        --stl all_polyhedra_31.stl \
        --out chandelier_blender.png

Inside the renderer:
- Chandelier metal frame: brass-colored Principled BSDF, metallic = 1.0
- Acrylic panels: Principled BSDF with transmission = 1.0, roughness = 0.3
  (frosted look; light from interior LEDs transmits through with scattering)
- LEDs: small emissive spheres (~ 12 mm) at each polyhedron centre with
  warm-white emission strength tuned to a comfortable interior brightness
- Room: 5 m × 5 m × 3.5 m box of matte planes around the chandelier
- Camera: standing person at 1.6 m, looking from one corner of the room
"""

import json
import math
import os
import sys
from pathlib import Path

import bpy  # noqa: F401  — only available inside Blender


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
    p.add_argument("--assets", default="blender_assets",
                   help="Directory containing panels.ply + led_positions.json")
    p.add_argument("--stl", default="all_polyhedra_31.stl",
                   help="Chandelier metal-frame STL.")
    p.add_argument("--out", default="renders/chandelier_blender.png",
                   help="Output PNG (relative paths resolve under chandelier/).")
    p.add_argument("--samples", type=int, default=96,
                   help="Cycles render samples (higher = less noise, slower).")
    p.add_argument("--width", type=int, default=1280,
                   help="Render width in pixels.")
    p.add_argument("--height", type=int, default=960,
                   help="Render height in pixels.")
    p.add_argument("--room-width", type=float, default=5.0,
                   help="Room width / depth in metres.")
    p.add_argument("--room-height", type=float, default=3.5,
                   help="Floor-to-ceiling height in metres.")
    p.add_argument("--led-strength", type=float, default=120.0,
                   help="Emission shader strength on the visible LED spheres "
                        "(only affects how bright the dots look, not room illum).")
    p.add_argument("--led-watts", type=float, default=120.0,
                   help="Wattage of the point light at each LED in Cycles. "
                        "Cycles measures light in radiometric watts, not lumens, "
                        "and frosted/translucent panels absorb most of it.  "
                        "Boost to 300–500 W with --sealed-ceiling for moody "
                        "dark-room shots.")
    p.add_argument("--shadow-softness", type=float, default=0.005,
                   help="Light-source radius (m) controlling shadow softness. "
                        "0.001 = near-point source = razor-sharp shadows that "
                        "throw clean polyhedron silhouettes on the walls.  "
                        "0.02 = soft like a real LED bulb.")
    p.add_argument("--projection", action="store_true",
                   help="Preset: render the chandelier in a dark room with "
                        "clear-ish panels, sharp shadows, and bright LEDs so "
                        "the polyhedra project crisp lattice patterns onto "
                        "the walls.  Override individual flags to fine-tune.")
    p.add_argument("--minimalist", action="store_true",
                   help="Render the *minimalist* build (cast metal star + "
                        "cable suspension + plastic polyhedra) instead of "
                        "the full-cast chandelier.  Loads three STLs:"
                        " chandelier_metal_minimal.stl (cast aluminum),"
                        " chandelier_minimalist_polyhedra.stl (painted plastic),"
                        " and chandelier_minimalist_cables.stl (stainless wire)."
                        " Run build_minimalist_render.py first to produce"
                        " the polyhedra + cables STLs.")
    p.add_argument("--led-color", default="#ffd9a3",
                   help="Hex color for LED emission (default warm white).")
    p.add_argument("--metal-color", default="#a07a45",
                   help="Hex color for metal frame (default antique brass).")
    p.add_argument("--panel-color", default="#fff4d6",
                   help="Hex color for acrylic panels (default warm-white frost).")
    p.add_argument("--panel-clarity", type=float, default=0.35,
                   help="0 = milky translucent panels (lots of glow, no shadow "
                        "projection); 1 = clear glass panels (no glow, sharp "
                        "shadow projection on walls).  0.35 = balanced.")
    p.add_argument("--panel-frostiness", type=float, default=0.18,
                   help="Roughness of the glass component (0 = optically "
                        "smooth, 0.5+ = very frosted).  Higher values blur "
                        "the projected shadow edges.")
    p.add_argument("--exposure", type=float, default=2.5,
                   help="Scene film exposure stops (each +1 doubles brightness).")
    p.add_argument("--ambient", type=float, default=1.0,
                   help="World-background strength.  Adds soft ambient light so "
                        "walls/floor aren't pitch black where the chandelier "
                        "doesn't reach directly.  In a sealed room this only "
                        "shows where the camera sees the back of a wall, so "
                        "it acts like a faint global fill.")
    p.add_argument("--no-room", action="store_true",
                   help="Skip the surrounding room.")
    p.add_argument("--sealed-ceiling", action="store_true",
                   help="Add a closed ceiling.  Default is OPEN ceiling so the "
                        "world-background ambient light floods in from above "
                        "like a skylight; that gives a brightly-lit room.  Use "
                        "this flag for a moody dark-room render where only the "
                        "chandelier's own light reaches the walls.")
    p.add_argument("--camera-height", type=float, default=1.6,
                   help="Camera eye-height above the floor in metres "
                        "(default 1.6 m = standing person).")
    p.add_argument("--camera-pitch-deg", type=float, default=None,
                   help="If set, override the camera pitch in degrees "
                        "(0 = horizontal, +30 = looking up, -30 = down).")
    p.add_argument("--device", default="CPU",
                   choices=("CPU", "CUDA", "OPTIX", "METAL", "HIP"),
                   help="Cycles compute device.  Mac users: try METAL.")
    p.add_argument("--engine", default="CYCLES", choices=("CYCLES", "BLENDER_EEVEE_NEXT"),
                   help="Render engine (Cycles is photoreal; EEVEE is real-time).")
    args = p.parse_args(argv)

    # ---- presets: only override values the user did not explicitly set -----
    user_set = {a.lstrip("-").replace("-", "_") for a in argv if a.startswith("--")}
    if args.projection:
        # Apply a "moody dark-room with crisp shadow projection" look.
        defaults = {
            "sealed_ceiling":   True,
            "panel_clarity":    0.85,   # mostly clear glass → projects shadows
            "panel_frostiness": 0.06,   # slight haze, mostly sharp
            "led_watts":        450.0,  # bright enough to throw light to walls
            "shadow_softness":  0.0015, # 1.5 mm — near point source
            "exposure":         3.5,
            "ambient":          0.04,   # near-zero so projection contrasts
            "panel_color":      "#fff7e1",
        }
        for k, v in defaults.items():
            if k not in user_set:
                setattr(args, k, v)
    return args


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def hex_to_rgb(s, gamma=2.2):
    s = s.lstrip("#")
    r = int(s[0:2], 16) / 255.0
    g = int(s[2:4], 16) / 255.0
    b = int(s[4:6], 16) / 255.0
    # Convert sRGB display color to linear for Cycles' linear pipeline.
    return (r ** gamma, g ** gamma, b ** gamma, 1.0)


def clear_scene():
    """Wipe out the default cube / camera / light."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (bpy.data.meshes, bpy.data.materials, bpy.data.lights,
                  bpy.data.cameras, bpy.data.images, bpy.data.textures):
        for item in list(block):
            block.remove(item, do_unlink=True)


def make_metal_material(name, hex_color):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = hex_to_rgb(hex_color)
    bsdf.inputs["Metallic"].default_value = 1.0
    bsdf.inputs["Roughness"].default_value = 0.4
    if "Specular IOR Level" in bsdf.inputs:
        bsdf.inputs["Specular IOR Level"].default_value = 0.5
    elif "Specular" in bsdf.inputs:
        bsdf.inputs["Specular"].default_value = 0.5
    mat.node_tree.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def make_acrylic_material(name, hex_color, clarity=0.35, frostiness=0.18):
    """Frosted-acrylic shader = Transparent BSDF + Translucent BSDF.

    - Transparent BSDF passes light through completely unaltered (no
      refraction, no caustics needed) so the metal lattice INSIDE the
      polyhedron cleanly projects its shadow onto the surrounding walls.
    - Translucent BSDF scatters light hemispherically — that's what makes
      the panel itself appear to glow softly from within.

    `clarity` (0..1):
        0 = pure milky-translucent (rich glow, no shadow projection)
        1 = pure transparent       (no glow, sharp projection on walls)
        0.35 = balanced "frosted glow + faint projection"
    `frostiness` (0..1):  unused for the transparent component (it has no
        roughness in Cycles); kept in the signature so you can tune the
        sheen of the surface in the future.  A small Glossy is added
        proportional to (1 - frostiness) for a faint outer reflection.
    """
    del frostiness  # currently unused; argument kept for API stability.
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    links = mat.node_tree.links

    out = nodes.new("ShaderNodeOutputMaterial")
    color_rgba = hex_to_rgb(hex_color)

    transparent = nodes.new("ShaderNodeBsdfTransparent")
    transparent.inputs["Color"].default_value = color_rgba

    translucent = nodes.new("ShaderNodeBsdfTranslucent")
    translucent.inputs["Color"].default_value = color_rgba

    mix = nodes.new("ShaderNodeMixShader")
    mix.inputs["Fac"].default_value = max(0.0, min(1.0, clarity))
    links.new(translucent.outputs["BSDF"], mix.inputs[1])
    links.new(transparent.outputs["BSDF"], mix.inputs[2])
    links.new(mix.outputs["Shader"], out.inputs["Surface"])
    return mat


def make_emission_material(name, hex_color, strength):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    em = nodes.new("ShaderNodeEmission")
    em.inputs["Color"].default_value = hex_to_rgb(hex_color)
    em.inputs["Strength"].default_value = strength
    mat.node_tree.links.new(em.outputs["Emission"], out.inputs["Surface"])
    return mat


def make_diffuse_material(name, hex_color, roughness=0.85):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = hex_to_rgb(hex_color)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = 0.0
    mat.node_tree.links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat


def import_stl(path):
    if hasattr(bpy.ops.wm, "stl_import"):
        bpy.ops.wm.stl_import(filepath=path)
    else:
        bpy.ops.import_mesh.stl(filepath=path)
    return bpy.context.selected_objects[-1]


def import_ply(path):
    if hasattr(bpy.ops.wm, "ply_import"):
        bpy.ops.wm.ply_import(filepath=path)
    else:
        bpy.ops.import_mesh.ply(filepath=path)
    return bpy.context.selected_objects[-1]


def add_plane(name, location, rotation, size_xy):
    """Add a finite plane (= a quad) of given size, oriented by rotation."""
    bpy.ops.mesh.primitive_plane_add(size=1.0, location=location, rotation=rotation)
    obj = bpy.context.object
    obj.name = name
    obj.scale = (size_xy[0], size_xy[1], 1.0)
    return obj


def add_emissive_sphere(name, location, radius, material):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=location,
                                         segments=20, ring_count=12)
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(material)
    bpy.ops.object.shade_smooth()
    return obj


def add_point_light(name, location, watts, color_rgba, soft_size=0.005):
    """Add a Blender Point light.  Energy in Cycles is in Watts.
    `soft_size` is the radius of the (virtual) emitter; smaller = sharper
    shadows projected on walls / floors."""
    light_data = bpy.data.lights.new(name=name, type="POINT")
    light_data.energy = watts
    light_data.color = color_rgba[:3]
    light_data.shadow_soft_size = max(0.0, soft_size)
    obj = bpy.data.objects.new(name=name, object_data=light_data)
    obj.location = location
    bpy.context.collection.objects.link(obj)
    return obj


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    args.out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    print(f"[render_blender] cwd: {os.getcwd()}")
    print(f"[render_blender] assets: {args.assets}")
    print(f"[render_blender] stl: {args.stl}")
    print(f"[render_blender] out: {args.out}")

    assets = Path(args.assets)
    if not (assets / "panels.ply").exists():
        raise SystemExit(
            f"Missing {assets/'panels.ply'} — run "
            f"`./run.sh simulate_chandelier.py --export-blender {args.assets}` first."
        )
    if not Path(args.stl).exists():
        raise SystemExit(f"Missing {args.stl} — run `./run.sh all_polyhedra.py` first.")

    led_meta = json.loads((assets / "led_positions.json").read_text())
    led_positions_mm = [tuple(p) for p in led_meta["led_positions_post_mm"]]
    print(f"[render_blender] {len(led_positions_mm)} LED positions loaded")

    # ----- scene setup ------------------------------------------------------
    clear_scene()
    scene = bpy.context.scene
    scene.render.engine = args.engine
    scene.render.resolution_x = args.width
    scene.render.resolution_y = args.height
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.filepath = args.out

    scene.view_settings.view_transform = "Filmic"
    scene.view_settings.look = "Medium Low Contrast"  # preserves shadow detail
    scene.view_settings.exposure = args.exposure
    scene.view_settings.gamma = 1.1

    if args.engine == "CYCLES":
        scene.cycles.samples = args.samples
        scene.cycles.use_denoising = True
        if hasattr(scene.cycles, "denoising_prefilter"):
            scene.cycles.denoising_prefilter = "ACCURATE"
        if hasattr(scene.cycles, "denoiser"):
            scene.cycles.denoiser = "OPENIMAGEDENOISE"
        scene.cycles.transmission_bounces = 12  # let LED light bounce out of panels
        scene.cycles.glossy_bounces = 6
        scene.cycles.transparent_max_bounces = 8
        scene.cycles.caustics_reflective = True
        scene.cycles.caustics_refractive = True
        # Compute device
        prefs = bpy.context.preferences.addons["cycles"].preferences
        if args.device != "CPU":
            try:
                prefs.compute_device_type = args.device
                for d in prefs.devices:
                    d.use = True
                scene.cycles.device = "GPU"
                print(f"[render_blender] using {args.device} GPU")
            except Exception as e:
                print(f"[render_blender] could not enable {args.device}: {e}; "
                      "falling back to CPU")
                scene.cycles.device = "CPU"
        else:
            scene.cycles.device = "CPU"

    # World — soft warm ambient so room walls aren't pitch black.
    world = bpy.data.worlds.new("ChandelierWorld")
    scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()
    bg_out = nt.nodes.new("ShaderNodeOutputWorld")
    bg = nt.nodes.new("ShaderNodeBackground")
    bg.inputs["Color"].default_value = (0.6, 0.5, 0.4, 1.0)
    bg.inputs["Strength"].default_value = args.ambient
    nt.links.new(bg.outputs["Background"], bg_out.inputs["Surface"])
    print(f"[render_blender] world ambient: color (0.6, 0.5, 0.4) "
          f"strength {args.ambient}")

    # ----- materials --------------------------------------------------------
    mat_metal   = make_metal_material("Metal", args.metal_color)
    mat_acrylic = make_acrylic_material("Acrylic", args.panel_color,
                                        clarity=args.panel_clarity,
                                        frostiness=args.panel_frostiness)
    mat_led     = make_emission_material("LED", args.led_color, args.led_strength)
    mat_floor   = make_diffuse_material("Floor",   "#5d4a36", roughness=0.85)  # warm wood
    mat_ceiling = make_diffuse_material("Ceiling", "#d8d2c4", roughness=0.85)  # cream
    mat_wall    = make_diffuse_material("Wall",    "#c2b59b", roughness=0.85)  # warm beige

    # Minimalist-build materials: painted-plastic polyhedra and bare
    # stainless cable suspension.  The polyhedra are PETG sprayed with
    # Rust-Oleum Specialty Metallic + clear coat, so they read as metal
    # but with slightly less crisp specular than the cast aluminum
    # canopy / star above them.  Cables are 0.8 mm 7×7 stainless aircraft
    # cable — high-roughness on the per-strand surface but reads silver
    # at a distance.
    mat_painted_plastic = bpy.data.materials.new(name="PaintedPlastic")
    mat_painted_plastic.use_nodes = True
    _ppn = mat_painted_plastic.node_tree.nodes
    _ppn.clear()
    _ppo = _ppn.new("ShaderNodeOutputMaterial")
    _ppb = _ppn.new("ShaderNodeBsdfPrincipled")
    _ppb.inputs["Base Color"].default_value = hex_to_rgb(args.metal_color)
    _ppb.inputs["Metallic"].default_value = 0.85   # painted, not pure metal
    _ppb.inputs["Roughness"].default_value = 0.55  # softer specular than cast
    if "Specular IOR Level" in _ppb.inputs:
        _ppb.inputs["Specular IOR Level"].default_value = 0.4
    elif "Specular" in _ppb.inputs:
        _ppb.inputs["Specular"].default_value = 0.4
    mat_painted_plastic.node_tree.links.new(
        _ppb.outputs["BSDF"], _ppo.inputs["Surface"])

    mat_cable = make_metal_material("StainlessCable", "#bfc1c4")
    # Override roughness — cables are 7x7 strand stainless, read slightly
    # brushed at viewing distance (lower than 0.4 of cast metal).
    _ck = mat_cable.node_tree.nodes
    for n in _ck:
        if n.bl_idname == "ShaderNodeBsdfPrincipled":
            n.inputs["Roughness"].default_value = 0.25

    # ----- import meshes ----------------------------------------------------
    # The chandelier authoring code uses +Y as "up" (gravity).  Blender's
    # convention is +Z up.  Rotate every imported asset by +90° around X so
    # authoring-Y maps to Blender-Z (up) and authoring-Z maps to Blender-Y.
    # Also scale mm -> metres.
    def yup_mm_to_zup_m(obj):
        obj.rotation_euler = (math.radians(90), 0.0, 0.0)
        obj.scale = (0.001, 0.001, 0.001)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    def yup_mm_point_to_zup_m(p_mm):
        x, y, z = p_mm
        return (x * 0.001, -z * 0.001, y * 0.001)

    if args.minimalist:
        # Three STLs, three materials: cast metal star + canopy, painted
        # plastic polyhedra, stainless cable suspension.
        for path in ("chandelier_metal_minimal.stl",
                     "chandelier_minimalist_polyhedra.stl",
                     "chandelier_minimalist_cables.stl"):
            if not Path(path).exists():
                raise SystemExit(
                    f"Missing {path} — run "
                    f"`./run.sh build_minimalist_render.py` first."
                )

        print("[render_blender] minimalist mode: importing 3 STLs ...")
        metal_obj = import_stl("chandelier_metal_minimal.stl")
        metal_obj.name = "Chandelier_CastMetal"
        metal_obj.data.materials.append(mat_metal)
        bpy.ops.object.shade_smooth()
        yup_mm_to_zup_m(metal_obj)

        plastic_obj = import_stl("chandelier_minimalist_polyhedra.stl")
        plastic_obj.name = "Chandelier_PlasticPolyhedra"
        plastic_obj.data.materials.append(mat_painted_plastic)
        bpy.ops.object.shade_smooth()
        yup_mm_to_zup_m(plastic_obj)

        cable_obj = import_stl("chandelier_minimalist_cables.stl")
        cable_obj.name = "Chandelier_Cables"
        cable_obj.data.materials.append(mat_cable)
        bpy.ops.object.shade_smooth()
        yup_mm_to_zup_m(cable_obj)
    else:
        print("[render_blender] importing chandelier metal STL ...")
        metal_obj = import_stl(args.stl)
        metal_obj.name = "Chandelier_Metal"
        metal_obj.data.materials.append(mat_metal)
        bpy.ops.object.shade_smooth()
        yup_mm_to_zup_m(metal_obj)

    print("[render_blender] importing acrylic panels PLY ...")
    panels_obj = import_ply(str(assets / "panels.ply"))
    panels_obj.name = "Chandelier_Panels"
    panels_obj.data.materials.append(mat_acrylic)
    bpy.ops.object.shade_smooth()
    yup_mm_to_zup_m(panels_obj)

    # Bounds in Blender Z-up metres — gather vertices from all chandelier
    # objects so the room sizes correctly even in minimalist mode (where
    # ``metal_obj`` is just the small canopy + 2 stars and the wider
    # polyhedra ring is in a separate object).
    bbox_objs = [metal_obj]
    if args.minimalist:
        bbox_objs += [plastic_obj, cable_obj]
    bbox = []
    for o in bbox_objs:
        bbox.extend(o.matrix_world @ v.co for v in o.data.vertices)
    xs = [v.x for v in bbox]; ys = [v.y for v in bbox]; zs = [v.z for v in bbox]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    cx = (xmin + xmax) / 2
    cy = (ymin + ymax) / 2
    cz = (zmin + zmax) / 2
    print(f"[render_blender] chandelier bounds (m, Z-up): "
          f"x[{xmin:.2f},{xmax:.2f}] y[{ymin:.2f},{ymax:.2f}] z[{zmin:.2f},{zmax:.2f}]")

    # ----- LEDs: emissive sphere (visible) + point light (illumination) ----
    led_radius_m = 0.012   # 12 mm
    led_color_rgba = hex_to_rgb(args.led_color)
    print(f"[render_blender] adding {len(led_positions_mm)} LEDs "
          f"(emission strength {args.led_strength}, point-light watts {args.led_watts})")
    for i, p in enumerate(led_positions_mm):
        loc = yup_mm_point_to_zup_m(p)
        add_emissive_sphere(f"LED_sphere_{i:02d}", loc, led_radius_m, mat_led)
        add_point_light(f"LED_light_{i:02d}", loc, args.led_watts,
                        led_color_rgba, soft_size=args.shadow_softness)

    # ----- room ------------------------------------------------------------
    if not args.no_room:
        ceiling_z = zmax + 0.20            # 20 cm above the eye loop
        floor_z   = ceiling_z - args.room_height
        half_w    = args.room_width / 2.0
        # Floor (normal +Z up — plane primitive is XY by default → no rotation)
        f = add_plane("Floor", (cx, cy, floor_z),
                      (0, 0, 0),
                      (args.room_width, args.room_width))
        f.data.materials.append(mat_floor)
        if args.sealed_ceiling:
            c = add_plane("Ceiling", (cx, cy, ceiling_z),
                          (math.radians(180), 0, 0),
                          (args.room_width, args.room_width))
            c.data.materials.append(mat_ceiling)
        # Four walls (each plane rotated to face inward)
        room_mid_z = (floor_z + ceiling_z) / 2
        wn = add_plane("Wall_N", (cx, cy + half_w, room_mid_z),
                       (math.radians(90), 0, 0),
                       (args.room_width, args.room_height))
        ws = add_plane("Wall_S", (cx, cy - half_w, room_mid_z),
                       (math.radians(-90), 0, 0),
                       (args.room_width, args.room_height))
        we = add_plane("Wall_E", (cx + half_w, cy, room_mid_z),
                       (0, math.radians(-90), 0),
                       (args.room_height, args.room_width))
        ww = add_plane("Wall_W", (cx - half_w, cy, room_mid_z),
                       (0, math.radians(90), 0),
                       (args.room_height, args.room_width))
        for w in (wn, ws, we, ww):
            w.data.materials.append(mat_wall)

    # ----- camera ----------------------------------------------------------
    if args.no_room:
        cam_pos = (cx + (xmax - xmin) * 1.2,
                   cy + (ymax - ymin) * 1.2,
                   cz)
        target = (cx, cy, cz)
    else:
        ceiling_z = zmax + 0.20
        floor_z   = ceiling_z - args.room_height
        eye_level = floor_z + args.camera_height
        # Stand ~ 60% of the way out from the chandelier, but keep a comfortable
        # gap from the wall (≥ 0.6 m) so the camera FOV captures floor + walls.
        offset = min(args.room_width * 0.30, args.room_width / 2.0 - 0.7)
        cam_pos = (cx + offset, cy + offset, eye_level)
        target = (cx, cy, cz - 0.20)  # aim slightly below chandelier for floor view
    bpy.ops.object.camera_add(location=cam_pos)
    cam = bpy.context.object
    cam.name = "Camera"
    direction = (target[0] - cam_pos[0],
                 target[1] - cam_pos[1],
                 target[2] - cam_pos[2])
    import mathutils
    rot = mathutils.Vector(direction).to_track_quat('-Z', 'Y')
    cam.rotation_euler = rot.to_euler()
    cam.data.lens = 24  # 24 mm — wide-angle so we see the room
    scene.camera = cam

    print(f"[render_blender] camera at {tuple(round(c, 2) for c in cam_pos)} "
          f"looking at {tuple(round(c, 2) for c in target)}")
    if not args.no_room:
        print(f"[render_blender] room: floor_z={floor_z:.2f} ceiling_z={ceiling_z:.2f} "
              f"width={args.room_width:.1f} (walls at ±{args.room_width/2:.2f})")
        print(f"[render_blender] camera distance to nearest wall: "
              f"{min(args.room_width/2 - abs(cam_pos[0]), args.room_width/2 - abs(cam_pos[1])):.2f} m")

    # ----- render ----------------------------------------------------------
    print(f"[render_blender] rendering {args.width}×{args.height} at "
          f"{args.samples} samples → {args.out}")
    bpy.ops.render.render(write_still=True)
    print("[render_blender] done.")


if __name__ == "__main__":
    main()
