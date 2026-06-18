#!/usr/bin/env python3
"""Batch evaluation across all layouts."""
import subprocess, sys, os

def main():
    print("Batch evaluation: all layouts, cem planner, 10 trials each
")
    for layout in [1, 2, 3]:
        print("=" * 60)
        cmd = [sys.executable, "scripts/run_task.py", "--layout", str(layout),
               "--trials", "10", "--planner", "cem"]
        subprocess.run(cmd, cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        print("")
    print("All evaluations complete. See experiments/ for results.")

if __name__ == "__main__":
    main()
