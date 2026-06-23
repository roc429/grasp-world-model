#!/usr/bin/env python3
"""示教脚本: 手动拖动机械臂到各位置, 记录真实坐标到 DOBOT_SCENES

用法:
  1. python scripts/teach_positions.py
  2. 用手拖动 Dobot 末端到 Layout 2 场景的物体位置
  3. 按 Enter 记录坐标
  4. 记录完所有位置后, 把输出的 DOBOT_SCENES 替换到 run_task.py
"""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import json

try:
    from src.robot.dobot_arm import DobotArm, DOBOT_DLL_AVAILABLE
except ImportError:
    print("[ERROR] DobotDllType.py not found")
    print("Copy it from magicician package to src/robot/")
    sys.exit(1)

def main():
    arm = DobotArm()
    port = input("Serial port [COM3]: ").strip() or "COM3"

    if not arm.connect(port=port):
        print("[ERROR] Connection failed. Check: USB, power, Key=ON(green)")
        sys.exit(1)

    print("\n=== Dobot Connected ===")
    print(f"Current pose: {arm.get_pose()}")
    print("")
    print("For each position, physically MOVE the Dobot arm end-effector to")
    print("where you want the object/obstacle/placement to be, then press Enter.")
    print("The arm will record its current coordinates.")
    print("")

    scenes = {}
    for layout_id in [1, 2, 3]:
        print(f"--- Layout {layout_id} ---")
        scene = {}

        input("  Move arm to TARGET OBJECT position, then press Enter...")
        p = arm.get_pose()
        scene["target"] = [round(p.x, 1), round(p.y, 1)]
        print(f"    target = [{scene['target'][0]}, {scene['target'][1]}]")

        if layout_id in [2, 3]:
            input("  Move arm to OBSTACLE position, then press Enter...")
            p = arm.get_pose()
            scene["obstacle"] = [round(p.x, 1), round(p.y, 1)]
            print(f"    obstacle = [{scene['obstacle'][0]}, {scene['obstacle'][1]}]")

        input("  Move arm to PLACEMENT ZONE center, then press Enter...")
        p = arm.get_pose()
        scene["placement"] = [round(p.x, 1), round(p.y, 1)]
        print(f"    placement = [{scene['placement'][0]}, {scene['placement'][1]}]")

        scene["z"] = 10.0
        scenes[str(layout_id)] = scene
        print("")

    print("\n=== Copy this into scripts/run_task.py (replace DOBOT_SCENES) ===\n")
    print("DOBOT_SCENES = {")
    for lid, scene in scenes.items():
        print(f"    {lid}: {{")
        print(f'        "target": np.array({scene["target"]}),')
        print(f'        "obstacle": np.array({scene["obstacle"]}),')
        print(f'        "placement": np.array({scene["placement"]}),')
        print(f'        "z": {scene["z"]},')
        print(f"    }},")
    print("}")

    # 保存到文件
    with open("config/dobot_scenes.json", "w") as f:
        json.dump(scenes, f, indent=2)
    print("\n Also saved to config/dobot_scenes.json")

    arm.disconnect()

if __name__ == "__main__":
    main()
