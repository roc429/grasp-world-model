#!/usr/bin/env python3
"""Dobot 直接控制演示 — 硬编码推-抓-放序列，不依赖感知/世界模型/规划器

用法:
  python scripts/dobot_direct_demo.py --port COM3 --layout 2

安全:
  - 运行前确保 Dobot 周围无障碍物
  - 按 Ctrl+C 随时停止
"""

import sys, os, argparse, time
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.robot.dobot_arm import DobotArm
from src.robot.base_arm import Pose


# ═══════════════════════════════════════════════════════
# 硬编码动作序列 — 根据你实际摆放的物体位置修改这些坐标
# 单位: mm, Dobot 坐标系
# ═══════════════════════════════════════════════════════

# ── Layout 2: 障碍物在前, 目标在后 ──
LAYOUT_2 = {
    "target_x": 230,    # 目标物体 X
    "target_y": 0,      # 目标物体 Y
    "target_z": 10,     # 物体高度
    "obstacle_x": 180,  # 障碍物 X
    "obstacle_y": 0,    # 障碍物 Y
    "obstacle_z": 10,   # 障碍物高度
    "push_dir": 1.57,   # 推的方向 (弧度, 1.57=右, 3.14=后, 0=前, -1.57=左)
    "push_dist": 60,    # 推的距离 (mm)
    "place_x": 200,     # 放置区 X
    "place_y": -100,    # 放置区 Y
    "z_safe": 50,       # 安全高度 (mm)
}

# ── Layout 1: 无障碍, 直接抓 ──
LAYOUT_1 = {
    "target_x": 200, "target_y": 0, "target_z": 10,
    "obstacle_x": 250, "obstacle_y": 80, "obstacle_z": 0,
    "push_dir": 0, "push_dist": 0,  # 不需要推
    "place_x": 250, "place_y": -80,
    "z_safe": 50,
}

# ── Layout 3: 角落, 需要长推 ──
LAYOUT_3 = {
    "target_x": 80, "target_y": -110, "target_z": 10,
    "obstacle_x": 150, "obstacle_y": -80, "obstacle_z": 10,
    "push_dir": -1.57, "push_dist": 50,
    "place_x": 200, "place_y": 80,
    "z_safe": 50,
}

LAYOUTS = {1: LAYOUT_1, 2: LAYOUT_2, 3: LAYOUT_3}


def demo_push_grasp_place(arm, L, verbose=True):
    """执行一次推-抓-放置"""

    # ── 0. 回零 ──
    if verbose:
        print("[0/4] 回零...")
    arm.home()
    time.sleep(1.0)

    # ── 1. 判断是否需要推 ──
    need_push = (L["push_dist"] > 0)

    if need_push:
        if verbose:
            print(f"[1/4] 推动作: 从障碍物({L['obstacle_x']},{L['obstacle_y']}) "
                  f"向{int(L['push_dir']*180/3.14)}°推{L['push_dist']}mm")

        # 计算推的起点和终点
        push_start_x = L["obstacle_x"] - 10
        push_start_y = L["obstacle_y"]
        push_end_x = push_start_x + L["push_dist"] * 1.4  # 1.4x 安全余量
        push_end_y = L["obstacle_y"]
        z_push = L["obstacle_z"] + 5  # 略高于障碍物

        # 移到推起点上方
        arm.move_to_pose(Pose(push_start_x, push_start_y, L["z_safe"], 0))
        time.sleep(0.3)
        # 下降到推高度
        arm.move_to_pose(Pose(push_start_x, push_start_y, z_push, 0))
        time.sleep(0.3)
        # 直线推
        arm.move_cp(Pose(push_end_x, push_end_y, z_push, 0), velocity=30)
        time.sleep(0.5)
        # 抬起
        arm.move_to_pose(Pose(push_end_x, push_end_y, L["z_safe"], 0))
        time.sleep(0.3)

        if verbose:
            print("  推动作完成")
    else:
        if verbose:
            print("[1/4] 跳过推 (无障碍物)")

    # ── 2. 抓取 ──
    if verbose:
        print(f"[2/4] 抓取: 目标({L['target_x']},{L['target_y']})")

    z_grasp = L["target_z"] + 5

    # 移动到目标上方
    arm.move_to_pose(Pose(L["target_x"], L["target_y"], L["z_safe"], 0))
    time.sleep(0.3)
    # 下降
    arm.move_to_pose(Pose(L["target_x"], L["target_y"], z_grasp, 0))
    time.sleep(0.3)
    # 闭合吸盘/夹爪
    arm.set_suction_cup(True)
    time.sleep(0.5)
    # 抬起
    arm.move_to_pose(Pose(L["target_x"], L["target_y"], L["z_safe"], 0))
    time.sleep(0.3)

    if verbose:
        print("  抓取完成")

    # ── 3. 放置 ──
    if verbose:
        print(f"[3/4] 放置: 目标区({L['place_x']},{L['place_y']})")

    # 移动到放置区上方
    arm.move_to_pose(Pose(L["place_x"], L["place_y"], L["z_safe"], 0))
    time.sleep(0.5)
    # 下降
    arm.move_to_pose(Pose(L["place_x"], L["place_y"], 20, 0))
    time.sleep(0.3)
    # 释放
    arm.set_suction_cup(False)
    time.sleep(0.5)
    # 抬起
    arm.move_to_pose(Pose(L["place_x"], L["place_y"], L["z_safe"], 0))
    time.sleep(0.3)

    if verbose:
        print("  放置完成")

    # ── 4. 回零 ──
    if verbose:
        print("[4/4] 回零")
    arm.home()

    print("  演示完成!")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, default="COM3")
    parser.add_argument("--layout", type=int, default=2, choices=[1, 2, 3])
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印动作序列, 不实际控制机械臂")
    args = parser.parse_args()

    L = LAYOUTS[args.layout]
    print(f"=== Dobot 直接控制演示 | Layout {args.layout} ===")
    print(f"  Target:   ({L['target_x']}, {L['target_y']})")
    print(f"  Obstacle: ({L['obstacle_x']}, {L['obstacle_y']})")
    print(f"  Place:    ({L['place_x']}, {L['place_y']})")
    print(f"  Push:     dir={int(L['push_dir']*180/3.14)}°, dist={L['push_dist']}mm")
    print(f"  Z safe:   {L['z_safe']}mm")
    print("")

    if args.dry_run:
        print("[DRY RUN] 仅打印, 不控制机械臂")
        return

    arm = DobotArm()
    print(f"连接 Dobot (port={args.port})...")

    if not arm.connect(port=args.port):
        print("[ERROR] 连接失败! 检查: USB线, 电源, Key开关(绿灯)")
        return

    print("连接成功!")
    print("3 秒后开始运动, 请保持安全距离...")
    time.sleep(3)

    try:
        demo_push_grasp_place(arm, L)
    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")
        arm.emergency_stop()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        arm.emergency_stop()
    finally:
        arm.disconnect()
        print("已断开连接")


if __name__ == "__main__":
    main()
