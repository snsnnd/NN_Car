#!/usr/bin/env python3
"""Build the baseline differential-drive vehicle asset from the vehicle profile."""

import json
import math
from pathlib import Path

import bpy


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = Path(__file__).resolve().parent
SOURCE_DIR = ROOT / "assets_source"
PROFILE_PATH = ROOT / "configs" / "vehicle_default.json"


def material(name, color, metallic=0.0, roughness=0.7, emission=None):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1.0)
    shader = mat.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1.0)
    shader.inputs["Metallic"].default_value = metallic
    shader.inputs["Roughness"].default_value = roughness
    if emission is not None:
        shader.inputs["Emission Color"].default_value = (*emission, 1.0)
        shader.inputs["Emission Strength"].default_value = 0.35
    return mat


def parent_to_root(obj, root):
    obj.parent = root
    obj.matrix_parent_inverse = root.matrix_world.inverted()
    return obj


def box(name, dimensions, location, mat, root, bevel=0.002):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Soft manufactured edges", "BEVEL")
        modifier.width = bevel
        modifier.segments = 3
        obj.modifiers.new("Weighted corner normals", "WEIGHTED_NORMAL")
    return parent_to_root(obj, root)


def cylinder(name, radius, depth, location, mat, root, vertices=32, bevel=0.0):
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=vertices, radius=radius, depth=depth, location=location
    )
    obj = bpy.context.object
    obj.name = name
    obj.data.materials.append(mat)
    if bevel > 0.0:
        modifier = obj.modifiers.new("Rounded rim", "BEVEL")
        modifier.width = bevel
        modifier.segments = 2
        obj.modifiers.new("Weighted corner normals", "WEIGHTED_NORMAL")
    return parent_to_root(obj, root)


def main():
    with PROFILE_PATH.open("r", encoding="utf-8") as source:
        profile = json.load(source)
    geometry = profile["geometry"]
    wheel_radius = float(geometry["wheel_radius_m"])
    track_width = float(geometry["track_width_m"])
    sensor_to_axle = float(geometry["sensor_to_axle_m"])
    sensor_positions = geometry["sensor_position_m"][: int(geometry["sensor_count"])]
    wheel_width = 0.018
    chassis_width = track_width - 2.0 * wheel_width - 0.008
    chassis_length = 0.19  # visual placeholder; wheelbase is not in the vehicle profile
    chassis_height = 0.034

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for datablocks in (bpy.data.meshes, bpy.data.curves, bpy.data.materials):
        for datablock in list(datablocks):
            if datablock.users == 0:
                datablocks.remove(datablock)

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    root = bpy.data.objects.new("NNCarVehicle", None)
    scene.collection.objects.link(root)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.08
    # Blender forward is local +Y; the GLTF/Godot import maps it to local -Z.

    chassis_mat = material("Chassis | deep teal", (0.035, 0.22, 0.24), 0.32, 0.38)
    deck_mat = material("Deck | graphite", (0.075, 0.12, 0.15), 0.28, 0.42)
    battery_mat = material("Battery | ceramic gray", (0.62, 0.70, 0.68), 0.12, 0.48)
    tire_mat = material("Tire | rubber", (0.025, 0.035, 0.04), 0.0, 0.92)
    hub_mat = material("Hub | warm aluminum", (0.72, 0.48, 0.23), 0.68, 0.3)
    board_mat = material("Sensor board | dark green", (0.045, 0.15, 0.12), 0.15, 0.58)
    sensor_mat = material("IR sensor | emitter", (0.025, 0.12, 0.11), 0.0, 0.28, (0.12, 0.8, 0.58))
    bumper_mat = material("Front bumper | signal orange", (0.92, 0.34, 0.10), 0.08, 0.38)
    metal_mat = material("Axle | steel", (0.32, 0.38, 0.40), 0.82, 0.28)

    z_chassis = wheel_radius + chassis_height * 0.5
    box("Chassis", (chassis_width, chassis_length, chassis_height),
        (0.0, 0.0, z_chassis), chassis_mat, root, 0.004)
    box("TopDeck", (chassis_width * 0.76, 0.108, 0.022),
        (0.0, 0.012, wheel_radius + chassis_height + 0.011), deck_mat, root, 0.003)
    box("BatteryPack", (0.052, 0.048, 0.025),
        (0.0, 0.026, wheel_radius + chassis_height + 0.0345), battery_mat, root, 0.003)
    box("FrontBumper", (chassis_width * 0.86, 0.012, 0.014),
        (0.0, chassis_length * 0.48, wheel_radius + 0.018), bumper_mat, root, 0.003)

    axle_length = max(0.01, track_width - 2.0 * wheel_width)
    axle = cylinder("DriveAxle", 0.006, axle_length, (0.0, 0.0, wheel_radius), metal_mat, root)
    axle.rotation_euler[1] = math.pi * 0.5
    for side, label in ((-1.0, "L"), (1.0, "R")):
        wheel_x = side * track_width * 0.5
        wheel = cylinder(f"DriveWheel_{label}", wheel_radius, wheel_width,
                         (wheel_x, 0.0, wheel_radius), tire_mat, root, bevel=0.0012)
        wheel.rotation_euler[1] = math.pi * 0.5
        hub_x = wheel_x + side * (wheel_width * 0.5 + 0.001)
        hub = cylinder(f"WheelHub_{label}", wheel_radius * 0.43, 0.002,
                       (hub_x, 0.0, wheel_radius), hub_mat, root, bevel=0.0008)
        hub.rotation_euler[1] = math.pi * 0.5

    # A small passive rear ball is a visual assumption, not a profile parameter.
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=20, ring_count=12, radius=0.009,
        location=(0.0, -chassis_length * 0.42, 0.009)
    )
    caster = bpy.context.object
    caster.name = "RearCaster"
    caster.data.materials.append(metal_mat)
    parent_to_root(caster, root)

    board_width = max(abs(float(position)) for position in sensor_positions) * 2.0 + 0.018
    box("GraySensorBoard", (board_width, 0.018, 0.004),
        (0.0, sensor_to_axle, 0.014), board_mat, root, 0.001)
    for index, position in enumerate(sensor_positions):
        # Profile positions are positive on the vehicle's left (local -X).
        x = -float(position)
        lens = cylinder(f"Sensor_{index:02d}", 0.0038, 0.007,
                        (x, sensor_to_axle, 0.0085), sensor_mat, root, vertices=16)
        lens.data.materials.clear()
        lens.data.materials.append(sensor_mat)

    scene.render.engine = "BLENDER_EEVEE"
    scene.render.resolution_x = 960
    scene.render.resolution_y = 640
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"

    SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    blend_path = SOURCE_DIR / "nncar_vehicle.blend"
    glb_path = ASSET_DIR / "nncar_vehicle.glb"
    bpy.ops.object.select_all(action="DESELECT")
    for obj in scene.objects:
        if obj == root or obj.parent == root:
            obj.select_set(True)
    bpy.context.view_layer.objects.active = root
    bpy.ops.wm.save_as_mainfile(filepath=str(blend_path))
    bpy.ops.export_scene.gltf(
        filepath=str(glb_path),
        export_format="GLB",
        use_selection=True,
        export_apply=True,
    )
    print(f"Wrote {blend_path}")
    print(f"Wrote {glb_path}")


if __name__ == "__main__":
    main()
