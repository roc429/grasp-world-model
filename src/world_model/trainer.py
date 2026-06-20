"""世界模型训练器"""

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm
import os
from typing import Dict, Optional

from src.world_model.model import WorldModelMLP, WorldModelGRU
from src.world_model.dataset import PushGraspDataset


def _build_model(model_cfg: dict) -> nn.Module:
    """根据配置构建世界模型"""
    if model_cfg["type"] == "mlp_gru":
        return WorldModelGRU(
            state_dim=model_cfg["state_dim"],
            action_dim=model_cfg["action_dim"],
            hidden_dim=model_cfg["hidden_dim"],
        )
    else:
        return WorldModelMLP(
            state_dim=model_cfg["state_dim"],
            action_dim=model_cfg["action_dim"],
            hidden_dim=model_cfg["hidden_dim"],
        )


def train_world_model(
    config: Dict,
    data_path: str,
    save_dir: str,
    device: Optional[str] = None,
) -> nn.Module:
    """训练世界模型"""
    if device is None:
        device = "cuda" if torch.cuda.is_available() else "cpu"
    device = torch.device(device)

    model_cfg = config["model"]
    train_cfg = config["training"]
    data_cfg = config["data"]

    model = _build_model(model_cfg).to(device)

    dataset = PushGraspDataset(data_path=data_path)
    n = len(dataset)
    train_n = int(n * data_cfg["train_ratio"])
    val_n = int(n * data_cfg["val_ratio"])
    test_n = n - train_n - val_n
    train_ds, val_ds, test_ds = random_split(
        dataset, [train_n, val_n, test_n],
        generator=torch.Generator().manual_seed(42),
    )

    train_loader = DataLoader(
        train_ds, batch_size=train_cfg["batch_size"], shuffle=True,
    )
    val_loader = DataLoader(val_ds, batch_size=train_cfg["batch_size"])

    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=train_cfg["learning_rate"],
        weight_decay=train_cfg.get("weight_decay", 0.0),
    )
    criterion = nn.MSELoss()
    writer = SummaryWriter(os.path.join(save_dir, "tensorboard"))

    best_val_loss = float("inf")

    for epoch in range(train_cfg["epochs"]):
        # --- train ---
        model.train()
        train_loss = 0.0
        for states, actions, next_states in tqdm(
            train_loader, desc=f"Epoch {epoch}"
        ):
            states = states.to(device)
            actions = actions.to(device)
            next_states = next_states.to(device)

            if isinstance(model, WorldModelGRU):
                pred_next, _ = model(states, actions)
            else:
                pred_next = model(states, actions)

            loss = criterion(pred_next, next_states)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        train_loss /= len(train_loader)
        writer.add_scalar("Loss/train", train_loss, epoch)

        # --- validate ---
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for states, actions, next_states in val_loader:
                states = states.to(device)
                actions = actions.to(device)
                next_states = next_states.to(device)
                if isinstance(model, WorldModelGRU):
                    pred_next, _ = model(states, actions)
                else:
                    pred_next = model(states, actions)
                val_loss += criterion(pred_next, next_states).item()
        val_loss /= len(val_loader)
        writer.add_scalar("Loss/val", val_loss, epoch)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            os.makedirs(save_dir, exist_ok=True)
            torch.save(model.state_dict(), os.path.join(save_dir, "world_model.pt"))

        if epoch % 10 == 0:
            print(
                f"Epoch {epoch}: train_loss={train_loss:.6f}, "
                f"val_loss={val_loss:.6f}"
            )

    writer.close()
    return model


def evaluate_world_model(
    model: nn.Module,
    test_loader: DataLoader,
    device: str = "cpu",
) -> float:
    """评估世界模型在测试集上的 MSE"""
    device = torch.device(device)
    model = model.to(device)
    model.eval()
    criterion = nn.MSELoss()
    total_loss = 0.0
    with torch.no_grad():
        for states, actions, next_states in test_loader:
            states = states.to(device)
            actions = actions.to(device)
            next_states = next_states.to(device)
            if isinstance(model, WorldModelGRU):
                pred_next, _ = model(states, actions)
            else:
                pred_next = model(states, actions)
            total_loss += criterion(pred_next, next_states).item()
    return total_loss / len(test_loader)
