#!/usr/bin/env python3
"""
第五赛段：孤梁稳渡 - 独立运行入口
在 docker 内运行: python3 run_stage5.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from cyberdog_controller.FSM.fsm import FSM


def main():
    print("=" * 60)
    print("  第五赛段：孤梁稳渡 (Stage 5: Bridge Crossing)")
    print("  任务：桥上稳定前进 -> 四足越虚线 -> 安全下桥")
    print("=" * 60)

    fsm = FSM()
    fsm.execute()


if __name__ == "__main__":
    main()
