"""场景搭建工具"""

import yaml
from typing import Dict


def load_scene_config(layout_id: int) -> Dict:
    """加载场景布局配置"""
    path = f"config/scenes/layout_{layout_id}.yaml"
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_scene(config: Dict):
    """根据配置搭建仿真场景 (占位)"""
    layout = config["scene"]["layouts"][0]
    scene_cfg = load_scene_config(layout)
    # TODO: 在 MuJoCo/PyBullet 中创建物体
    print(f"Building scene: {scene_cfg.get('description', 'unknown')}")
    return scene_cfg
