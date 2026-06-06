"""日志系统"""
import logging
import json
import os
from typing import Any, Dict
from datetime import datetime

LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s:%(lineno)d - %(message)s"


def setup_logger(name: str, log_dir: str = "logs") -> logging.Logger:
    """创建并配置 logger"""
    os.makedirs(log_dir, exist_ok=True)
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)

    # 文件 handler
    date_str = datetime.now().strftime("%Y-%m-%d")
    fh = logging.FileHandler(os.path.join(log_dir, f"{date_str}.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(LOG_FORMAT))
    logger.addHandler(fh)

    # 控制台 handler
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(LOG_FORMAT))
    logger.addHandler(ch)

    return logger


class ExperimentLogger:
    """实验数据记录器"""

    def __init__(self, exp_name: str, config: dict):
        self.exp_name = exp_name
        self.config = config
        self.exp_dir = os.path.join("experiments", exp_name)
        os.makedirs(self.exp_dir, exist_ok=True)
        self.records = []

    def log_step(self, step: int, data: Dict[str, Any]) -> None:
        self.records.append({"step": step, **data})

    def log_result(self, success: bool, metrics: Dict[str, float]) -> None:
        self.records.append({"type": "result", "success": success, **metrics})

    def save(self) -> str:
        path = os.path.join(self.exp_dir, "results.json")
        with open(path, "w") as f:
            json.dump(self.records, f, indent=2, ensure_ascii=False)
        return path
