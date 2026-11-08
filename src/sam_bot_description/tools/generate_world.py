#!/usr/bin/env python3
"""从 world/model.sdf（Gazebo Building Editor 导出的裸模型）生成 world/my_newworld.sdf。

背景（2026-11-04 已修复）
------------------------
Building Editor 是按“楼层”保存模型的，在哪一层画，导出的 link z 就加上那一层的
基准高度。本仓库的墙原本画在 4 楼的层级上，导出的 model.sdf 里全部 128 个 link
都在 model z=3.0（碰撞体 3.0~6.1 m）。而机器人出生在 z≈0.24，于是墙整体悬浮在
机器人头顶 3 米。

修复方式：直接把 model.sdf 本身下移 3.1 m（备份 /tmp/model.sdf.bak），并把
z=6.0 上两块多余的 1x1 小地板 Floor_3 / Floor_4 删除。现在 model.sdf 的碰撞体
范围是 −0.10~2.90 m，地板顶面在 0.00。

本脚本仍然保留，用于两种情况：
  * 以后又在 Building Editor 里重新保存了 model.sdf（可能再次带上楼层偏移）；
  * 需要重新生成一份“去贴图 / 去地板视觉”的 world 文件。
脚本会读取当前 model.sdf，动态计算“让地板顶面落到 z=0”所需的平移量，
所以无论 model.sdf 有没有楼层偏移都能正确处理。

同时做三件事（都是为了这台机器上的软件渲染，Mesa swrast）：
1. 删除所有 <material> 贴图：126 面墙各贴一张 1024x1024 砖块纹理 + mipmap，
   软件渲染会卡死（ogre.log 停在 bricks.png）并出现闪烁/条纹。删掉后墙面
   使用 Gazebo 默认灰色，渲染快且干净。
2. 删除地板 link 的 <visual>，保留 <collision>：地板由 world 里的灰色
   ground_plane 显示，机器人仍然靠地板 collision 站立。
3. 删除原模型里悬在 z=6.0 的两块 1x1 小地板（Floor_3 / Floor_4）：下移后
   它们会变成世界原点上方 2.9 m 处的“悬浮障碍物”。

用法
----
    python3 src/sam_bot_description/tools/generate_world.py
    # 指定输入/输出
    python3 src/sam_bot_description/tools/generate_world.py \
        src/sam_bot_description/world/model.sdf \
        src/sam_bot_description/world/my_newworld.sdf

注意：--symlink-install 时 install/ 下的 world 是指向这里的软链，重新生成后
直接重新 launch 即可，不需要 colcon build。
"""

import copy
import os
import sys
import xml.etree.ElementTree as ET

WORLD_HEADER = """<?xml version='1.0' encoding='utf-8'?>
<sdf version="1.7">
  <world name="my_newworld">
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    <model name="ground_plane">
      <static>1</static>
      <pose>0 0 -0.01 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>2 -6 9 0 0.75 1.5708</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic" />
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>0</shadows>
    </scene>
    <wind />
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
"""

WORLD_FOOTER = """  </world>
</sdf>
"""

# 模型在世界里的平面位置：沿用拆楼时 CAD 场景的摆放，使原点落在地板内部，
# 且与建图/导航的 map 原点一致（map_server 的原点 = base_link 出生点）。
MODEL_XY = (1.20003, -12.5601)


def pose_xyz(elem):
    text = (elem.findtext("pose") or "0 0 0 0 0 0").split()
    return [float(v) for v in text]


def collision_box(link):
    col = link.find("collision")
    if col is None:
        return None
    geom = col.find("geometry")
    if geom is None:
        return None
    box = geom.find("box")
    if box is None:
        return None
    size = [float(v) for v in box.findtext("size").split()]
    cpose = pose_xyz(col) if col.find("pose") is not None else [0, 0, 0, 0, 0, 0]
    lz = pose_xyz(link)[2]
    return size, lz + cpose[2]


def find_floor(links):
    """返回地板 link：碰撞盒最大的那个。"""
    best, best_area = None, -1.0
    for link in links:
        cb = collision_box(link)
        if cb is None:
            continue
        size, _ = cb
        area = size[0] * size[1]
        if area > best_area:
            best, best_area = link, area
    return best


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)  # src/sam_bot_description
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(pkg, "world", "model.sdf")
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(pkg, "world", "my_newworld.sdf")

    root = ET.parse(src).getroot()
    model = root.find("model")
    if model is None:
        model = root  # model.sdf 的根就是 <sdf><model>
        model = model.find("model")
    if model is None:
        sys.exit(f"✗ {src} 里找不到 <model>")

    model = copy.deepcopy(model)

    # 去掉可能的 <include>/<joint> 干扰（Building Editor 不会导出这些，保险起见）
    for tag in ("include", "pose"):
        for el in model.findall(tag):
            model.remove(el)

    links = model.findall("link")
    print(f"[1/5] 读入 {len(links)} 个 link")

    # ---- 1. 计算下移量：让地板碰撞体的顶面落在 z=0 ----
    floor = find_floor(links)
    if floor is None:
        sys.exit("✗ 找不到地板碰撞体（最大的 box）")
    fsize, fcenter_z = collision_box(floor)
    floor_top = fcenter_z + fsize[2] / 2.0
    shift_z = -floor_top
    print(f"[2/5] 地板 {floor.get('name')} 顶面 model-z={floor_top:.3f} "
          f"→ 整体下移 {shift_z:.3f} m")

    # ---- 2. 删除悬空的小块（Floor_3/Floor_4 之类的 1x1 板） ----
    # 判别依据是 link 的 z：墙体和地板都在 floor 所在层（model z=3.0），
    # 而 Floor_3/Floor_4 在上一层（model z=6.0）。不能用“盒子小”来判断——
    # 很多 0.6 m 的墙段盒子也很小，会误删。
    floor_z = pose_xyz(floor)[2]
    removed = []
    for link in list(links):
        cb = collision_box(link)
        if cb is None:
            continue
        size, _ = cb
        if pose_xyz(link)[2] - floor_z > 2.0 and size[0] <= 1.5 and size[1] <= 1.5:
            removed.append(link.get("name"))
            model.remove(link)
            links.remove(link)
    if removed:
        print(f"[3/5] 删除悬空小地板: {', '.join(removed)}")
    else:
        print("[3/5] 没有悬空小地板")

    # ---- 3. 删除所有贴图材质；4. 删除地板 visual（保留 collision） ----
    tex = 0
    for link in links:
        for visual in link.findall("visual"):
            for mat in visual.findall("material"):
                visual.remove(mat)
                tex += 1
        if link is floor:
            for visual in link.findall("visual"):
                link.remove(visual)
    print(f"[4/5] 删除 {tex} 个材质贴图；地板 visual 已删除（保留 collision）")

    # ---- 5. 组装 world ----
    model.set("name", "my_newworld")
    static = model.find("static")
    if static is None:
        static = ET.SubElement(model, "static")
    static.text = "1"
    model_pose = ET.Element("pose")
    shift_text = "0.000" if abs(shift_z) < 5e-4 else f"{shift_z:.3f}"
    model_pose.text = f"{MODEL_XY[0]:.5f} {MODEL_XY[1]:.5f} {shift_text} 0 -0 0"
    model.insert(0, model_pose)

    ET.indent(model, space="  ")
    body = ET.tostring(model, encoding="unicode")

    with open(dst, "w", encoding="utf-8") as f:
        f.write(WORLD_HEADER)
        f.write(body)
        f.write("\n")
        f.write(WORLD_FOOTER)

    # 用实际的碰撞体计算生成后的世界 z 范围，避免公式写死
    world_z = []
    for link in links:
        cb = collision_box(link)
        if cb is None:
            continue
        size, center_z = cb
        world_z.append(center_z + shift_z - size[2] / 2.0)
        world_z.append(center_z + shift_z + size[2] / 2.0)
    print(f"[5/5] ✓ 已生成 {dst}")
    print(f"      世界坐标：碰撞体 z = {min(world_z):.2f} ~ {max(world_z):.2f}"
          f"（地板顶面 = 0.00）")


if __name__ == "__main__":
    main()
