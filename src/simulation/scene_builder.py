"""场景搭建工具 — 从 YAML 配置生成 MuJoCo 场景 XML"""

import yaml
import numpy as np
from typing import Dict


def load_scene_config(layout_id: int) -> Dict:
    """加载场景布局配置"""
    path = f"config/scenes/layout_{layout_id}.yaml"
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _mm2m(val):
    """毫米转米（MuJoCo 使用国际单位制）"""
    if isinstance(val, list):
        return [v / 1000.0 for v in val]
    return val / 1000.0


def build_mjcf_string(scene_config: Dict) -> str:
    """
    根据场景配置生成 MuJoCo MJCF XML 字符串。

    场景结构：
      - 地面（plane）
      - 桌面（box）
      - 红色目标物体（free body，可被推动）
      - 蓝色障碍物（free body，可被推动）
      - 灰色末端执行器（mocap body，受代码控制移动）
      - 绿色放置区标记
      - 位置传感器（实时读取物体坐标）
    """
    objects = scene_config["objects"]
    target = objects["target"]
    obstacle = objects["obstacle"]
    placement = scene_config["placement_zone"]

    t_pos = _mm2m(target["position"])        # [x, y, z] in meters
    t_half = [_mm2m(s) / 2 for s in target["size"]]   # half-sizes
    o_pos = _mm2m(obstacle["position"])
    o_half = [_mm2m(s) / 2 for s in obstacle["size"]]
    p_pos = _mm2m(placement["position"])
    p_radius = _mm2m(placement["radius"])

    xml = f'''<mujoco model="push_grasp_scene">
  <compiler angle="radian"/>

  <option timestep="0.002" gravity="0 0 -9.81">
    <flag contact="enable" gravity="enable"/>
  </option>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0.1 0.2 0.3" width="512" height="512"/>
  </asset>

  <worldbody>
    <!-- 地面 -->
    <geom type="plane" size="1 1 0.1" rgba="0.3 0.3 0.3 1"/>

    <!-- 桌面 (z=0 为桌面顶面) -->
    <body pos="0.2 0 -0.005" name="table">
      <geom type="box" size="0.25 0.2 0.005" rgba="0.7 0.5 0.3 1" name="table_geom"
            friction="0.6 0.2 0.1"/>
    </body>

    <!-- 目标物体 (红色) — free body 必须在 worldbody 顶层 -->
    <body pos="{t_pos[0]} {t_pos[1]} {t_half[2] + 0.005}" name="target_body">
      <freejoint name="target_joint"/>
      <geom type="box" size="{t_half[0]} {t_half[1]} {t_half[2]}"
            rgba="1 0 0 1" name="target_geom" density="500"
            friction="0.8 0.3 0.1" margin="0.001"/>
    </body>

    <!-- 障碍物 (蓝色) -->
    <body pos="{o_pos[0]} {o_pos[1]} {o_half[2] + 0.005}" name="obstacle_body">
      <freejoint name="obstacle_joint"/>
      <geom type="box" size="{o_half[0]} {o_half[1]} {o_half[2]}"
            rgba="0 0 1 1" name="obstacle_geom" density="300"
            friction="0.8 0.3 0.1" margin="0.001"/>
    </body>

    <!-- 末端执行器 (mocap body: 由代码控制位置，参与碰撞但不被推动)
         半径 12mm，高度 30mm — 模拟手指/推杆 -->
    <body mocap="true" pos="0.25 0 0.08" name="ee_mocap">
      <geom type="cylinder" size="0.012 0.030" rgba="0.6 0.6 0.6 1" name="ee_geom"
            friction="1.0 0.2 0.1" margin="0.001"/>
    </body>

    <!-- 放置区标记 (绿色半透明圆盘) -->
    <body pos="{p_pos[0]} {p_pos[1]} 0.002" name="placement_marker">
      <geom type="cylinder" size="{p_radius} 0.001" rgba="0 1 0 0.25" name="placement_geom"/>
    </body>
  </worldbody>

  <sensor>
    <!-- 物体位置传感器：实时读取 xyz -->
    <framepos objtype="geom" objname="target_geom" name="target_pos"/>
    <framepos objtype="geom" objname="obstacle_geom" name="obstacle_pos"/>
    <!-- 末端执行器位置 -->
    <framepos objtype="geom" objname="ee_geom" name="ee_pos"/>
  </sensor>
</mujoco>'''
    return xml


def create_default_scene(scene_config: Dict):
    """
    高层接口：根据场景配置构建仿真场景的 MJCF 模型。

    Returns:
        mj_model: MuJoCo 模型对象
        mj_data: MuJoCo 数据对象
    """
    import mujoco
    xml_string = build_mjcf_string(scene_config)
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    return model, data
