#!/usr/bin/env python3
"""Train world model on collected data."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.utils.config import load_config
from src.world_model.trainer import train_world_model

def main():
    config = load_config("config/world_model.yaml")
    data_path = "data/raw/push_data.npz"
    save_dir = "models/world_model"
    if not os.path.exists(data_path):
        print("ERROR: No data found. Run scripts/collect_data.py first.")
        sys.exit(1)
    print("Training world model...")
    model = train_world_model(config, data_path, save_dir)
    print("Done! Model saved to", save_dir)

if __name__ == "__main__":
    main()
