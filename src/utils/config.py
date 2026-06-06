"""配置读取工具"""
import yaml
from typing import Any, Dict


def load_config(config_path: str) -> Dict[str, Any]:
    """加载 YAML 配置文件"""
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def merge_configs(*configs: Dict) -> Dict:
    """合并多个配置字典, 后面的覆盖前面的"""
    result = {}
    for cfg in configs:
        result.update(cfg)
    return result
