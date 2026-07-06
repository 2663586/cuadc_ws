#!/usr/bin/env python3
"""
CUADC 2026 —— 任务入口点。

用法：
    python run_mission.py [--address udp://:14540] [--sim]

程序连接到 PX4（真机或 SITL），运行完整的任务状态机，
并在完成或紧急情况下断开上锁。
"""

import argparse
import asyncio
import sys
import os

# 确保 src/ 包可以被导入
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from interface import PX4Interface
from main_fsm import MissionFSM
from logger_manager import init_logger


async def main(system_address: str = "udp://:14540", sim: bool = False):
    print("=" * 60)
    print("CUADC 2026 — 自主任务控制器")
    print(f"PX4 地址: {system_address}")
    print(f"模式: {'仿真' if sim else '实机'}")
    print("=" * 60)

    # 初始化日志记录器
    logger = init_logger()
    logger.log_message("info", "CUADC 2026 — 自主任务控制器")
    logger.log_message("info", f"PX4 地址: {system_address}")
    logger.log_message("info", f"模式: {'仿真' if sim else '实机'}")

    # 初始化通信层
    interface = PX4Interface(system_address=system_address)
    fsm = MissionFSM(interface)

    try:
        logger.log_event("mission_start", sim=sim)
        await fsm.run()
        logger.log_event("mission_complete")
    except KeyboardInterrupt:
        print("\n[中止] 手动中断 — 紧急降落")
        logger.log_message("abort", "手动中断 — 紧急降落", "fail")
        logger.log_event("abort", reason="keyboard_interrupt")
        await interface.land()
    except Exception as e:
        print(f"\n[致命错误] 未处理的异常: {e}")
        logger.log_message("fatal", f"未处理的异常: {e}", "fail")
        logger.log_event("fatal", error=str(e))
        try:
            await interface.land()
        except Exception:
            pass
    finally:
        print("[信息] 任务结束")
        logger.log_message("info", "任务结束")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="CUADC 2026 自主任务控制器"
    )
    parser.add_argument(
        "--address", default="udp://0.0.0.0:14540",
        help="PX4 MAVLink 地址（默认: udp://0.0.0.0:14540）"
    )
    parser.add_argument(
        "--sim", action="store_true",
        help="以仿真模式运行"
    )
    args = parser.parse_args()

    asyncio.run(main(args.address, args.sim))
