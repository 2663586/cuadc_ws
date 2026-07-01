#!/usr/bin/env python3
"""
MAVSDK 到 PX4 SITL 的快速连接测试。

在启动 PX4 SITL（例如 `make px4_sitl jmavsim`）之后运行。
此脚本连接、等待 GPS、上锁、起飞、悬停，然后降落。

用法：
    python3 scripts/test_mavsdk_connection.py
    python3 scripts/test_mavsdk_connection.py --address udp://:14540
"""

import asyncio
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from mavsdk import System
from mavsdk.offboard import PositionNedYaw


async def test_connection(system_address: str = "udp://:14540"):
    print(f"正在连接到 PX4，地址: {system_address} ...")
    drone = System()
    await drone.connect(system_address=system_address)

    # 等待连接
    print("等待连接 ...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("  已连接。")
            break

    # 等待 GPS 和家点位置
    print("等待 GPS 锁定和家点位置 ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("  GPS 正常，家点位置已设置。")
            break

    # 上锁
    print("正在上锁 ...")
    await drone.action.arm()
    print("  已上锁。")

    # 进入 offboard 模式
    print("正在进入 offboard 模式 ...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    print("  offboard 模式已激活。")

    # 起飞到 5 米
    print("正在起飞到 5 米 ...")
    await drone.action.set_takeoff_altitude(5.0)
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # 在 5 米处悬停 3 秒
    print("在 5 米处悬停 ...")
    for _ in range(30):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # 向前飞 10 米
    print("向前飞行 10 米 ...")
    for _ in range(50):
        await drone.offboard.set_position_ned(
            PositionNedYaw(10.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # 飞回原点
    print("返回原点 ...")
    for _ in range(50):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -5.0, 0.0)
        )
        await asyncio.sleep(0.1)

    # 降落
    print("正在降落 ...")
    await drone.offboard.stop()
    await drone.action.land()
    await asyncio.sleep(5)

    # 断开上锁
    await drone.action.disarm()
    print("=== 测试完成: 全链路正常! ===")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="udp://:14540")
    args = parser.parse_args()

    asyncio.run(test_connection(args.address))
