#!/usr/bin/env python3
import sys
import os

# 确保能找到 src 目录下的包
sys.path.insert(0, '/usr/local/lib/python3.8/site-packages')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from cyberdog_controller.FSM.fsm import FSM

def main():
    print("=" * 60)
    print("  启动第六赛段独立测试")
    print("=" * 60)

    fsm = FSM()
    fsm.execute()

if __name__ == "__main__":
    main()