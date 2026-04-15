"""MuJoCo scene construction helpers."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


_PI_HALF = "1.5707963"


def _ensure_child(root: ET.Element, tag: str) -> ET.Element:
    node = root.find(tag)
    if node is None:
        node = ET.SubElement(root, tag)
    return node


def _add_axis_geom(
    parent: ET.Element,
    *,
    axis: str,
    radius: float,
    half_length: float,
    color: str,
) -> None:
    geom = ET.SubElement(parent, "geom")
    geom.set("type", "cylinder")
    geom.set("size", f"{radius} {half_length}")
    geom.set("rgba", color)
    geom.set("contype", "0")
    geom.set("conaffinity", "0")
    geom.set("mass", "0")

    if axis == "x":
        geom.set("pos", f"{half_length} 0 0")
        geom.set("euler", f"0 {_PI_HALF} 0")
    elif axis == "y":
        geom.set("pos", f"0 {half_length} 0")
        geom.set("euler", f"-{_PI_HALF} 0 0")
    else:
        geom.set("pos", f"0 0 {half_length}")


def _add_mocap_marker(
    worldbody: ET.Element,
    *,
    body_name: str,
    box_size: float,
    box_rgba: str,
    axis_radius: float,
    axis_half_length: float,
    axis_alpha_suffix: str,
) -> None:
    body = ET.SubElement(worldbody, "body")
    body.set("name", body_name)
    body.set("pos", "0 0 0")
    body.set("mocap", "true")

    geom = ET.SubElement(body, "geom")
    geom.set("type", "box")
    geom.set("size", f"{box_size} {box_size} {box_size}")
    geom.set("rgba", box_rgba)
    geom.set("contype", "0")
    geom.set("conaffinity", "0")
    geom.set("mass", "0")

    _add_axis_geom(body, axis="x", radius=axis_radius, half_length=axis_half_length, color=f"1 0 0 {axis_alpha_suffix}")
    _add_axis_geom(body, axis="y", radius=axis_radius, half_length=axis_half_length, color=f"0 1 0 {axis_alpha_suffix}")
    _add_axis_geom(body, axis="z", radius=axis_radius, half_length=axis_half_length, color=f"0 0 1 {axis_alpha_suffix}")


def _attach_tcp_body(worldbody: ET.Element, tcp_offset: np.ndarray) -> None:
    for body in worldbody.iter("body"):
        if body.get("name") != "ArmLseventh_Link":
            continue

        tcp_body = ET.SubElement(body, "body")
        tcp_body.set("name", "tcp")
        tcp_body.set("pos", f"{tcp_offset[0]} {tcp_offset[1]} {tcp_offset[2]}")

        _add_axis_geom(tcp_body, axis="x", radius=0.003, half_length=0.06, color="1 0 0 1")
        _add_axis_geom(tcp_body, axis="y", radius=0.003, half_length=0.06, color="0 1 0 1")
        _add_axis_geom(tcp_body, axis="z", radius=0.003, half_length=0.06, color="0 0 1 1")
        return

    raise ValueError("未找到新模型末端 body 'ArmLseventh_Link'，无法挂载 TCP")


def _normalize_mesh_paths(root: ET.Element) -> None:
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and filename.startswith("package://"):
            mesh.set("filename", os.path.basename(filename))


def _augment_scene(root: ET.Element, tcp_offset: np.ndarray) -> None:
    asset = _ensure_child(root, "asset")

    tex_grid = ET.SubElement(asset, "texture")
    tex_grid.set("name", "grid_tex")
    tex_grid.set("type", "2d")
    tex_grid.set("builtin", "checker")
    tex_grid.set("rgb1", "0.4 0.4 0.4")
    tex_grid.set("rgb2", "0.3 0.3 0.3")
    tex_grid.set("width", "512")
    tex_grid.set("height", "512")

    mat_grid = ET.SubElement(asset, "material")
    mat_grid.set("name", "grid_mat")
    mat_grid.set("texture", "grid_tex")
    mat_grid.set("texrepeat", "8 8")
    mat_grid.set("reflectance", "0.05")

    tex_sky = ET.SubElement(asset, "texture")
    tex_sky.set("name", "sky_tex")
    tex_sky.set("type", "skybox")
    tex_sky.set("builtin", "gradient")
    tex_sky.set("rgb1", "0.55 0.75 0.98")
    tex_sky.set("rgb2", "0.08 0.10 0.25")
    tex_sky.set("width", "512")
    tex_sky.set("height", "3072")

    visual = _ensure_child(root, "visual")
    headlight = _ensure_child(visual, "headlight")
    headlight.set("ambient", "0.15 0.15 0.15")
    headlight.set("diffuse", "0.3 0.3 0.3")
    headlight.set("specular", "0.1 0.1 0.1")

    worldbody = _ensure_child(root, "worldbody")

    floor = ET.SubElement(worldbody, "geom")
    floor.set("name", "floor")
    floor.set("type", "plane")
    floor.set("size", "3 3 0.1")
    floor.set("material", "grid_mat")
    floor.set("contype", "0")
    floor.set("conaffinity", "0")

    key_light = ET.SubElement(worldbody, "light")
    key_light.set("name", "key_light")
    key_light.set("pos", "0 -1.5 3")
    key_light.set("dir", "0 0.4 -1")
    key_light.set("diffuse", "0.4 0.4 0.4")
    key_light.set("specular", "0.1 0.1 0.1")
    key_light.set("directional", "true")

    fill_light = ET.SubElement(worldbody, "light")
    fill_light.set("name", "fill_light")
    fill_light.set("pos", "2 2 2.5")
    fill_light.set("dir", "-0.5 -0.5 -1")
    fill_light.set("diffuse", "0.2 0.2 0.2")
    fill_light.set("specular", "0.05 0.05 0.05")
    fill_light.set("directional", "true")

    _add_mocap_marker(
        worldbody,
        body_name="target_pose",
        box_size=0.015,
        box_rgba="0.2 0.8 0.2 0.3",
        axis_radius=0.002,
        axis_half_length=0.05,
        axis_alpha_suffix="0.8",
    )
    _add_mocap_marker(
        worldbody,
        body_name="reported_pose",
        box_size=0.016,
        box_rgba="0.2 0.2 0.8 0.5",
        axis_radius=0.0015,
        axis_half_length=0.04,
        axis_alpha_suffix="0.5",
    )
    _attach_tcp_body(worldbody, tcp_offset)


def build_enhanced_model(urdf_filename: str, tcp_offset: np.ndarray) -> mujoco.MjModel:
    """Load URDF and inject a viewer-friendly scene."""
    resolved_urdf_filename = "_resolved_model.urdf"
    tree = ET.parse(urdf_filename)
    root = tree.getroot()

    mujoco_ext = _ensure_child(root, "mujoco")
    compiler = _ensure_child(mujoco_ext, "compiler")
    compiler.set("meshdir", "../meshes")
    _normalize_mesh_paths(root)

    tree.write(resolved_urdf_filename, encoding="utf-8", xml_declaration=True)
    try:
        basic_model = mujoco.MjModel.from_xml_path(resolved_urdf_filename)
    finally:
        try:
            os.remove(resolved_urdf_filename)
        except OSError:
            pass

    enhanced_path = "_enhanced_scene.xml"
    try:
        mujoco.mj_saveLastXML(enhanced_path, basic_model)
        tree = ET.parse(enhanced_path)
        root = tree.getroot()
        _augment_scene(root, tcp_offset)
        tree.write(enhanced_path)
        return mujoco.MjModel.from_xml_path(enhanced_path)
    except Exception as exc:
        print(f"  [场景] 增强失败 ({exc})，使用基本模型")
        return basic_model
    finally:
        try:
            os.remove(enhanced_path)
        except OSError:
            pass
