"""安装配置"""
from setuptools import setup, find_packages

setup(
    name="push_grasp_world_model",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.24.0",
        "torch>=2.0.0",
        "pyyaml>=6.0",
    ],
    python_requires=">=3.10",
)
