#!/usr/bin/env python3
"""FAST-LIO-SAM-SC-QN adapter for the generic web_mapping contract."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import signal
import shlex
import subprocess
import struct
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String

try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except Exception:  # noqa: BLE001
    LivoxCustomMsg = None


SAVED_MAP_MARKERS = (
    "map_optimized.pcd",
    "map_raw.pcd",
    "poses_matrix.txt",
    "poses_kitti.txt",
    "poses_tum.txt",
)


class FastLioWebBroker(Node):
    def __init__(self) -> None:
        super().__init__("fast_lio_web_broker")
        self._declare_parameters()

        self.map_history_root = self._resolve_path(str(self.get_parameter("map_history_root").value))
        save_root_param = str(self.get_parameter("save_root").value).strip()
        self.save_root = self._resolve_path(save_root_param) if save_root_param else str(Path(self.map_history_root).parent)
        self.save_trigger_topic = str(self.get_parameter("save_trigger_topic").value)
        self.start_command = str(self.get_parameter("start_command").value)
        self.process_cwd = str(self.get_parameter("process_cwd").value).strip()
        self.stop_timeout_sec = float(self.get_parameter("stop_timeout_sec").value)
        self.save_before_stop_timeout_sec = float(self.get_parameter("save_before_stop_timeout_sec").value)
        self.status_period_sec = float(self.get_parameter("status_period_sec").value)

        self.state = "idle"
        self.message = "等待开始"
        self.session_name = ""
        self.last_command = ""
        self.last_result = ""
        self._process: subprocess.Popen[str] | None = None
        self._saw_raw = False
        self._saw_map = False

        cloud_qos = QoSProfile(depth=5)
        cloud_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        reliable_qos = QoSProfile(depth=10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sensor_qos = QoSProfile(depth=50)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.raw_pub = self.create_publisher(PointCloud2, "/web_mapping/raw_cloud", cloud_qos)
        self.current_pub = self.create_publisher(PointCloud2, "/web_mapping/current_frame", cloud_qos)
        self.map_pub = self.create_publisher(PointCloud2, "/web_mapping/global_map", map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/web_mapping/pose", reliable_qos)
        self.raw_path_pub = self.create_publisher(RosPath, "/web_mapping/raw_path", reliable_qos)
        self.optimized_path_pub = self.create_publisher(RosPath, "/web_mapping/optimized_path", reliable_qos)
        self.imu_pub = self.create_publisher(Imu, "/web_mapping/imu", sensor_qos)
        self.lidar_status_pub = self.create_publisher(String, "/web_mapping/lidar_status", status_qos)
        self.status_pub = self.create_publisher(String, "/web_mapping/status", status_qos)
        self.save_pub = self.create_publisher(String, self.save_trigger_topic, reliable_qos)

        if LivoxCustomMsg is not None:
            self.raw_source_topic = str(self.get_parameter("livox_lidar_topic").value)
            self.create_subscription(LivoxCustomMsg, self.raw_source_topic, self._livox_callback, sensor_qos)
        else:
            self.raw_source_topic = str(self.get_parameter("fast_lio_raw_cloud_topic").value)
            self.create_subscription(PointCloud2, self.raw_source_topic, self._raw_callback, cloud_qos)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("fast_lio_current_frame_topic").value),
            self._relay(self.current_pub),
            cloud_qos,
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("fast_lio_global_map_topic").value),
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("fast_lio_pose_topic").value),
            self._relay(self.pose_pub),
            reliable_qos,
        )
        self.create_subscription(
            RosPath,
            str(self.get_parameter("fast_lio_raw_path_topic").value),
            self._relay(self.raw_path_pub),
            reliable_qos,
        )
        self.create_subscription(
            RosPath,
            str(self.get_parameter("fast_lio_optimized_path_topic").value),
            self._relay(self.optimized_path_pub),
            reliable_qos,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("fast_lio_imu_topic").value),
            self._relay(self.imu_pub),
            sensor_qos,
        )
        self.create_subscription(String, "/web_mapping/command", self._command_callback, reliable_qos)
        self.status_timer = self.create_timer(self.status_period_sec, self._publish_status)

        self.get_logger().info(f"Web broker map history root: {self.map_history_root}")
        self.get_logger().info(f"Web raw cloud source: {self.raw_source_topic}")
        self.get_logger().info("FAST-LIO web broker ready on /web_mapping/*")
        self._publish_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter("map_history_root", "maps")
        self.declare_parameter("save_root", "")
        self.declare_parameter("save_trigger_topic", "save_dir")
        self.declare_parameter("start_command", "")
        self.declare_parameter("process_cwd", "")
        self.declare_parameter("stop_timeout_sec", 8.0)
        self.declare_parameter("save_before_stop_timeout_sec", 30.0)
        self.declare_parameter("status_period_sec", 0.5)
        self.declare_parameter("livox_lidar_topic", "/livox/lidar")
        self.declare_parameter("fast_lio_raw_cloud_topic", "cloud_registered_1")
        self.declare_parameter("fast_lio_current_frame_topic", "corrected_current_pcd")
        self.declare_parameter("fast_lio_global_map_topic", "corrected_map")
        self.declare_parameter("fast_lio_pose_topic", "pose_stamped")
        self.declare_parameter("fast_lio_raw_path_topic", "ori_path")
        self.declare_parameter("fast_lio_optimized_path_topic", "corrected_path")
        self.declare_parameter("fast_lio_imu_topic", "/livox/imu")

    def _resolve_path(self, value: str) -> str:
        path_text = value.strip()
        if not path_text:
            path_text = "."
        path = Path(path_text).expanduser()
        if path.is_absolute():
            return str(path)
        cwd = Path.cwd()
        if path.parts and cwd.name == path.parts[0]:
            return str(cwd.joinpath(*path.parts[1:]))
        return str(cwd / path)

    def _relay(self, publisher: Any):
        def callback(msg: Any) -> None:
            publisher.publish(msg)

        return callback

    def _raw_callback(self, msg: PointCloud2) -> None:
        self._saw_raw = True
        self.raw_pub.publish(msg)
        self._publish_lidar_status("online")

    def _livox_callback(self, msg: Any) -> None:
        cloud_msg = self._livox_to_pointcloud2(msg)
        self._saw_raw = True
        self.raw_pub.publish(cloud_msg)
        self._publish_lidar_status("online")

    def _livox_to_pointcloud2(self, msg: Any) -> PointCloud2:
        points = getattr(msg, "points", [])
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_bigendian = False
        cloud.is_dense = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = bytearray(cloud.row_step)
        for index, point in enumerate(points):
            struct.pack_into(
                "<ffff",
                data,
                index * cloud.point_step,
                float(point.x),
                float(point.y),
                float(point.z),
                float(point.reflectivity) / 255.0,
            )
        cloud.data = bytes(data)
        return cloud

    def _map_callback(self, msg: PointCloud2) -> None:
        self._saw_map = True
        self.map_pub.publish(msg)

    def _command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid web_mapping command JSON")
            return
        command = str(payload.get("command", "")).strip()
        session_name = self._clean_session_name(str(payload.get("session_name", "")).strip())
        if session_name:
            self.session_name = session_name
        self.last_command = command
        if command == "start":
            self._start_mapping()
        elif command == "stop":
            self._stop_mapping()
        elif command == "save":
            self._save_mapping()
        elif command == "reset_error":
            self.state = "idle"
            self.message = "等待开始"
            self.last_result = "error cleared"
        else:
            self.last_result = f"unknown command: {command}"
            self.message = "未知建图指令"
        self._publish_status()

    def _clean_session_name(self, value: str) -> str:
        cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("._-")
        return cleaned[:64]

    def _ensure_session_name(self) -> str:
        if not self.session_name:
            self.session_name = f"map_{time.strftime('%Y%m%d_%H%M%S')}"
        return self.session_name

    def _start_mapping(self) -> None:
        self._ensure_session_name()
        if self._process and self._process.poll() is None:
            self.state = "mapping" if self._saw_raw else "starting"
            self.message = "建图已在运行"
            self.last_result = "already running"
            return
        if not self.start_command:
            self.state = "mapping" if self._saw_raw else "starting"
            self.message = "等待雷达数据"
            self.last_result = "attached to existing mapping topics"
            return
        self._saw_raw = False
        self._saw_map = False
        command = self._format_start_command()
        try:
            self._process = subprocess.Popen(command, shell=True, cwd=self.process_cwd or None, preexec_fn=os.setsid)
        except OSError as exc:
            self.state = "error"
            self.message = "启动建图失败"
            self.last_result = str(exc)
            return
        self.state = "starting"
        self.message = "正在启动建图"
        self.last_result = "started"

    def _stop_mapping(self) -> None:
        if self._process and self._process.poll() is None:
            save_root, save_started_at = self._publish_save_trigger()
            save_detected = self._wait_for_saved_session(save_started_at, self.save_before_stop_timeout_sec)
            try:
                os.killpg(os.getpgid(self._process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self._process.wait(timeout=self.stop_timeout_sec)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
                except ProcessLookupError:
                    pass
                self._process.wait()
            self._process = None
            self.last_result = (
                f"stopped; save detected: {save_root}"
                if save_detected
                else f"stopped; save requested: {save_root}"
            )
            self.state = "stopped"
            self.message = "建图已停止，地图已保存" if save_detected else "建图已停止，已请求保存地图"
        elif self._process and self._process.poll() is not None:
            self._process = None
            self.last_result = "process already exited"
            self.state = "stopped"
            self.message = "建图已停止"
        elif not self.start_command:
            self.last_result = "attached mode, no process to stop"
            self._request_save("建图已停止，已请求保存地图")
        else:
            self.last_result = "no active mapping process"
            self.state = "stopped"
            self.message = "建图已停止"

    def _save_mapping(self) -> None:
        if self.start_command and (self._process is None or self._process.poll() is not None):
            self._request_save("地图已保存，可在历史地图中下载")
            return
        self._request_save("已请求保存地图")

    def _request_save(self, message: str) -> None:
        save_root, _ = self._publish_save_trigger()
        self.state = "stopped"
        self.message = message
        self.last_result = f"save requested: {save_root}"

    def _publish_save_trigger(self) -> tuple[str, float]:
        self._ensure_session_name()
        save_root = self.save_root
        msg = String()
        msg.data = save_root
        self.save_pub.publish(msg)
        return save_root, time.time()

    def _wait_for_saved_session(self, started_at: float, timeout_sec: float) -> bool:
        if not self.session_name:
            return False
        session_path = Path(self.map_history_root) / self.session_name
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline:
            if self._session_has_new_files(session_path, started_at):
                return True
            time.sleep(0.2)
        return self._session_has_new_files(session_path, started_at)

    def _session_has_new_files(self, session_path: Path, started_at: float) -> bool:
        for filename in SAVED_MAP_MARKERS:
            path = session_path / filename
            if not path.is_file():
                continue
            try:
                if path.stat().st_mtime >= started_at - 0.5:
                    return True
            except OSError:
                continue
        return False

    def _publish_lidar_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.lidar_status_pub.publish(msg)

    def _publish_status(self) -> None:
        self._refresh_process_state()
        if self.state in {"starting", "mapping"}:
            if self._saw_map:
                self.state = "mapping"
                self.message = "正在建图，地图持续更新中"
            elif self._saw_raw:
                self.state = "mapping"
                self.message = "已收到雷达数据，正在生成地图"
        payload = {
            "type": "mapping_status",
            "state": self.state,
            "message": self.message,
            "session_name": self.session_name,
            "map_history_root": self.map_history_root,
            "last_command": self.last_command,
            "last_result": self.last_result,
            "stamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)

    def _refresh_process_state(self) -> None:
        if self._process is None:
            return
        exit_code = self._process.poll()
        if exit_code is None:
            return
        self._process = None
        if self.state in {"starting", "mapping", "stopping"}:
            self.state = "stopped" if exit_code == 0 else "error"
            self.message = "建图已停止" if exit_code == 0 else "建图进程已退出"
            self.last_result = f"process exited: {exit_code}"

    def _format_start_command(self) -> str:
        session_arg = f"session_name:={self.session_name}" if self.session_name else ""
        command = self.start_command.format(
            session_name=self.session_name,
            session_arg=session_arg,
            save_root=shlex.quote(self.save_root),
            map_history_root=shlex.quote(self.map_history_root),
        )
        return " ".join(shlex.split(command))


def main() -> None:
    rclpy.init()
    node = FastLioWebBroker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
