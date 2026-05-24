#!/usr/bin/env python3
"""
第六赛段：撷金建功 - 独立运行入口
在 docker 内运行: python3 run_stage6.py
"""
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from cyberdog_controller.FSM.fsm import FSM


def main():
    print("=" * 60)
    print("  第六赛段：撷金建功 (Stage 6: Kick & Finish)")
    print("  任务：踢足球出口 -> 走到终点圈 -> 趴下")
    print("=" * 60)

    fsm = FSM()
    fsm.execute()


if __name__ == "__main__":
    main()
