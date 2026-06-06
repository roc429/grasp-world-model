"""世界模型训练器"""

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import os
from typing import Dict

from src.world_model.model import WorldModelMLP, WorldModelGRU
from src.world_model.dataset import PushGraspDataset


def train_world_model(config: Dict, data_path: str, save_dir: str) -> nn.Module:
    """训练世界模型"""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model_cfg = config["model"]
    train_cfg = config["training"]
    data_cfg = config["data"]

    if model_cfg["type"] == "mlp_gru":
        model = WorldModelGRU(
            state_dim=model_cfg["state_dim"],
            action_dim=model_cfg["action_dim"],
            hidden_dim=model_cfg["hidden_dim"],
        )
    else:
        model = WorldModelMLP(
            state_dim=model_cfg["state_dim"],
            action_dim=model_cfg["action_dim"],
            hidden_dim=model_cfg["hidden_dim"],
        )
    model = model.to(device)

    dataset = PushGraspDataset(data_path=data_path)
    n = len(dataset)
    train_n = int(n * data_cfg["train_ratio"])
    val_n = int(n * data_cfg["val_ratio"])
    test_n = n - train_n - val_n
    train_ds, val_ds, test_ds = random_split(dataset, [train_n, val_n, test_n])

    train_loader = DataLoader(train_ds, batch_size=train_cfg["batch_size"], shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=train_cfg["batch_size"])

    optimizer = torch.optim.Adam(model.parameters(), lr=train_cfg["learning_rate"])
    criterion = nn.MSELoss()
    writer = SummaryWriter(os.path.join(save_dir, "tensorboard"))

    for epoch in range(train_cfg["epochs"]):
        model.train()
        train_loss = 0.0
        for states, actions, next_states in tqdm(train_loader, desc=f"Epoch {epoch}"):
            states = states.to(device)
            actions = actions.to(device)
            next_states = next_states.to(device)
            pred_next, _ = model(states, actions)
            loss = criterion(pred_next, next_states)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        train_loss /= len(train_loader)
        writer.add_scalar("Loss/train", train_loss, epoch)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for states, actions, next_states in val_loader:
                states = states.to(device)
                actions = actions.to(device)
                next_states = next_states.to(device)
                pred_next, _ = model(states, actions)
                val_loss += criterion(pred_next, next_states).item()
        val_loss /= len(val_loader)
        writer.add_scalar("Loss/val", val_loss, epoch)

        if epoch % 10 == 0:
            print(f"Epoch {epoch}: train_loss={train_loss:.6f}, val_loss={val_loss:.6f}")

    os.makedirs(save_dir, exist_ok=True)
    torch.save(model.state_dict(), os.path.join(save_dir, "world_model.pt"))
    writer.close()
    return model
