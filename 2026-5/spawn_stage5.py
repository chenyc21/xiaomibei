#!/usr/bin/env python3
"""
把机器人移动到第五赛段桥上开始位置，便于跳过入口动作调试 Stage5。

默认先使用 Gazebo ROS /set_entity_state 传送已有 robot。
失败后 fallback 到 Gazebo 原生 pose/modify，再失败则删除重生。
如只想强制删除重生，可加 --respawn。

用法：先启动 Gazebo，再在同一容器新终端运行：
    source /opt/ros/galactic/setup.bash
    source /home/cyberdog_sim/install/setup.bash
    python3 spawn_stage5.py
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, GetModelList, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty


def stage5_pose_values():
    # 第五赛段蓝圈位置：右侧桥段靠上平台口，面朝世界坐标正左方。
    yaw = math.pi
    return {
        "x": 3.2,
        "y": 12.2,
        "z": 0.60,
        "qx": 0.0,
        "qy": 0.0,
        "qz": math.sin(yaw / 2.0),
        "qw": math.cos(yaw / 2.0),
    }


def stage5_pose():
    values = stage5_pose_values()
    pose = Pose()
    pose.position.x = values["x"]
    pose.position.y = values["y"]
    pose.position.z = values["z"]
    pose.orientation.x = values["qx"]
    pose.orientation.y = values["qy"]
    pose.orientation.z = values["qz"]
    pose.orientation.w = values["qw"]
    return pose


def teleport_robot_with_gz_topic():
    pose = stage5_pose_values()
    msg = (
        'name: "robot" '
        f'position {{ x: {pose["x"]} y: {pose["y"]} z: {pose["z"]} }} '
        f'orientation {{ x: {pose["qx"]} y: {pose["qy"]} '
        f'z: {pose["qz"]} w: {pose["qw"]} }}'
    )

    last_error = ""
    for attempt in range(1, 4):
        try:
            result = subprocess.run(
                ["gz", "topic", "-p", "/gazebo/earth/pose/modify", "-m", msg],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=4.0,
            )
        except subprocess.TimeoutExpired:
            last_error = f"gz topic 第 {attempt} 次调用超时"
            print(f"[spawn_stage5] {last_error}，继续尝试 fallback。", file=sys.stderr)
            continue

        if result.returncode == 0:
            subprocess.run(["gz", "world", "-w", "earth", "-p", "0"], check=False)
            print("[spawn_stage5] 已通过 gz topic 传送 robot 到第五赛段桥上开始位置。")
            return
        last_error = (result.stderr or result.stdout).strip()
        time.sleep(0.3)

    raise RuntimeError(f"传送 robot 失败: {last_error or 'gz topic 无响应'}")


class Stage5Spawner(Node):
    def __init__(self):
        super().__init__("stage5_spawner")
        self.robot_description = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(
            String, "/robot_description", self._robot_description_cb, qos
        )
        self.delete_cli = self.create_client(DeleteEntity, "/delete_entity")
        self.spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self.set_state_cli = self.create_client(SetEntityState, "/set_entity_state")
        self.model_cli = self.create_client(GetModelList, "/get_model_list")
        self.unpause_cli = self.create_client(Empty, "/unpause_physics")

    def _robot_description_cb(self, msg):
        self.robot_description = msg.data

    def wait_ready(self):
        deadline = time.time() + 30.0
        while rclpy.ok() and self.robot_description is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.robot_description:
            raise RuntimeError("没有收到 /robot_description，请确认 Gazebo 已启动。")

        for name, cli in [
            ("/delete_entity", self.delete_cli),
            ("/spawn_entity", self.spawn_cli),
            ("/get_model_list", self.model_cli),
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务不可用: {name}")
        self._wait_gazebo_response()

    def wait_teleport_ready(self):
        for name, cli in [
            ("/set_entity_state", self.set_state_cli),
            ("/get_model_list", self.model_cli),
        ]:
            if not cli.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"服务不可用: {name}")
        models = self._wait_gazebo_response()
        if "robot" not in list(models.model_names):
            raise RuntimeError("Gazebo 中没有 robot 模型，无法直接传送。")

    def _wait_gazebo_response(self):
        req = GetModelList.Request()
        future = self.model_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        if result is None:
            raise RuntimeError("Gazebo 服务无响应，请重启 Gazebo 后再运行。")
        return result

    def teleport_robot(self):
        self.wait_teleport_ready()

        state = EntityState()
        state.name = "robot"
        state.pose = stage5_pose()
        state.reference_frame = "world"

        req = SetEntityState.Request()
        req.state = state
        future = self.set_state_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None or not result.success:
            status = getattr(result, "status_message", "") if result else "服务无返回"
            raise RuntimeError(f"/set_entity_state 传送失败: {status}")

        print("[spawn_stage5] 已通过 /set_entity_state 传送 robot 到第五赛段桥上开始位置。")
        self.unpause_physics()

    def delete_robot(self):
        req = DeleteEntity.Request()
        req.name = "robot"
        future = self.delete_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        if future.result() is not None:
            self.get_logger().info(f"删除旧 robot: {future.result().success}")

    def spawn_robot(self):
        req = SpawnEntity.Request()
        req.name = "robot"
        req.xml = self.robot_description
        req.robot_namespace = ""
        req.reference_frame = "world"
        req.initial_pose = stage5_pose()

        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        if future.result() is None or not future.result().success:
            raise RuntimeError("重生 robot 失败：/spawn_entity 没有返回成功，请检查 Gazebo 是否存活。")
        self.get_logger().info(f"已重生到第五赛段桥上开始位置: {future.result().status_message}")

    def unpause_physics(self):
        try:
            subprocess.run(
                ["gz", "world", "-w", "earth", "-p", "0"],
                check=False,
                timeout=2.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass

        if not self.unpause_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("/unpause_physics 不可用，跳过取消暂停。")
            return
        future = self.unpause_cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            self.get_logger().info("Gazebo physics 已取消暂停。")
        else:
            self.get_logger().warning("调用 /unpause_physics 超时，继续。")

def respawn_robot():
    rclpy.init()
    node = Stage5Spawner()
    try:
        node.wait_ready()
        node.delete_robot()
        time.sleep(1.0)
        node.spawn_robot()
        node.unpause_physics()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def teleport_robot_with_service():
    rclpy.init()
    node = Stage5Spawner()
    try:
        node.teleport_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def teleport_robot():
    failures = []
    for label, action in [
        ("/set_entity_state", teleport_robot_with_service),
        ("gz topic", teleport_robot_with_gz_topic),
        ("respawn", respawn_robot),
    ]:
        try:
            action()
            return
        except Exception as exc:
            message = f"{label} 失败: {exc}"
            failures.append(message)
            print(f"[spawn_stage5] {message}", file=sys.stderr)

    raise RuntimeError("移动到第五赛段桥上开始位置失败；" + "；".join(failures))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--respawn",
        action="store_true",
        help="删除 robot 后用 /spawn_entity 重生；默认只传送已有 robot。",
    )
    args = parser.parse_args()
    if args.respawn:
        respawn_robot()
    else:
        teleport_robot()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[spawn_stage5] {exc}", file=sys.stderr)
        sys.exit(1)
