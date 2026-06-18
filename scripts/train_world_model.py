#!/usr/bin/env python3
"""世界模型训练脚本"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.utils.config import load_config
from src.world_model.trainer import train_world_model

def main():
    config = load_config("config/world_model.yaml")
    data_path = "data/raw/push_data.npz"
    save_dir = "models/world_model"
    print(f"Training {config['model']['type']} world model...")
    model = train_world_model(config, data_path, save_dir)
    print(f"Done! Model saved to {save_dir}/world_model.pt")

if __name__ == "__main__":
    main()
