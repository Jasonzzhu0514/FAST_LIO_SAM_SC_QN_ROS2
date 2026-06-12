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
import time
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


SAVED_METADATA_MARKERS = (
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
        self.frontend_map_save_timeout_sec = float(self.get_parameter("frontend_map_save_timeout_sec").value)
        self.frontend_map_service_wait_sec = float(self.get_parameter("frontend_map_service_wait_sec").value)
        self.status_period_sec = float(self.get_parameter("status_period_sec").value)
        self.fast_lio_current_frame_topic = str(self.get_parameter("fast_lio_current_frame_topic").value).strip()
        self.fast_lio_global_map_topic = str(self.get_parameter("fast_lio_global_map_topic").value).strip()
        self.fast_lio_map_save_service = str(self.get_parameter("fast_lio_map_save_service").value).strip()
        self.stop_wait_topics = self._split_topics(str(self.get_parameter("stop_wait_topics").value))
        self.current_frame_uses_global_map = (
            not self.fast_lio_current_frame_topic
            or self.fast_lio_current_frame_topic == self.fast_lio_global_map_topic
        )

        self.state = "idle"
        self.message = "等待开始"
        self.session_name = ""
        self.last_command = ""
        self.last_result = ""
        self._process: subprocess.Popen[str] | None = None
        self._last_current_output_at = 0.0

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

        self.current_pub = self.create_publisher(PointCloud2, "/web_mapping/current_frame", cloud_qos)
        self.map_pub = self.create_publisher(PointCloud2, "/web_mapping/global_map", map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/web_mapping/pose", reliable_qos)
        self.raw_path_pub = self.create_publisher(RosPath, "/web_mapping/raw_path", reliable_qos)
        self.optimized_path_pub = self.create_publisher(RosPath, "/web_mapping/optimized_path", reliable_qos)
        self.imu_pub = self.create_publisher(Imu, "/web_mapping/imu", sensor_qos)
        self.status_pub = self.create_publisher(String, "/web_mapping/status", status_qos)
        self.save_pub = self.create_publisher(String, self.save_trigger_topic, reliable_qos)

        if not self.current_frame_uses_global_map:
            self.create_subscription(
                PointCloud2,
                self.fast_lio_current_frame_topic,
                self._current_callback,
                cloud_qos,
            )
        self.create_subscription(
            PointCloud2,
            self.fast_lio_global_map_topic,
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
        self.get_logger().info("Web raw cloud is handled directly by web_mapping_bridge")
        if self.current_frame_uses_global_map:
            self.get_logger().info(
                f"Web current-frame stream follows cumulative map: {self.fast_lio_global_map_topic}"
            )
        else:
            self.get_logger().info(
                f"Web current-frame stream source: {self.fast_lio_current_frame_topic}"
            )
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
        self.declare_parameter("frontend_map_save_timeout_sec", 180.0)
        self.declare_parameter("frontend_map_service_wait_sec", 3.0)
        self.declare_parameter("process_exit_timeout_sec", 3.0)
        self.declare_parameter("output_quiet_timeout_sec", 2.5)
        self.declare_parameter(
            "stop_wait_topics",
            "/livox/lidar,/livox/imu,/cloud_registered_1,/Odometry_loc,corrected_map",
        )
        self.declare_parameter("status_period_sec", 0.5)
        self.declare_parameter("fast_lio_current_frame_topic", "/cloud_registered_1")
        self.declare_parameter("fast_lio_global_map_topic", "corrected_map")
        self.declare_parameter("fast_lio_map_save_service", "map_save")
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

    def _current_callback(self, msg: PointCloud2) -> None:
        self._mark_current_output()
        self.current_pub.publish(msg)

    def _map_callback(self, msg: PointCloud2) -> None:
        self.map_pub.publish(msg)
        if self.current_frame_uses_global_map:
            self._mark_current_output()
            self.current_pub.publish(msg)

    def _mark_current_output(self) -> None:
        self._last_current_output_at = time.monotonic()

    def _has_recent_current_output(self) -> bool:
        if self._last_current_output_at <= 0.0:
            return False
        return time.monotonic() - self._last_current_output_at < 2.0

    def _command_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring invalid web_mapping command JSON")
            return
        command = str(payload.get("command", "")).strip()
        session_name = self._clean_session_name(str(payload.get("session_name", "")).strip())
        if command != "start" and session_name:
            self.session_name = session_name
        self.last_command = command
        if command == "start":
            self._start_mapping(session_name)
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

    def _start_mapping(self, requested_session_name: str = "") -> None:
        if self._process and self._process.poll() is None:
            self.state = "mapping" if self._has_recent_current_output() else "starting"
            self.message = "建图已在运行"
            self.last_result = "already running"
            return
        self.session_name = requested_session_name
        self._ensure_session_name()
        if not self.start_command:
            self.state = "mapping" if self._has_recent_current_output() else "starting"
            self.message = "等待雷达数据"
            self.last_result = "attached to existing mapping topics"
            return
        self._last_current_output_at = 0.0
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
            process = self._process
            process_group = self._process_group(process)
            self.state = "stopping"
            self.message = "建图正在停止，正在保存地图"
            self.last_result = "stopping"
            self._publish_status()
            save_root, save_started_at = self._publish_save_trigger()
            self.message = "建图正在停止，正在保存累计地图"
            self._publish_status()
            frontend_saved = self._save_frontend_map(save_started_at)
            self.message = "建图正在停止，正在等待地图元数据"
            self._publish_status()
            metadata_saved = self._wait_for_saved_metadata(save_started_at, self.save_before_stop_timeout_sec)
            map_saved = frontend_saved or self._map_file_is_new(self._frontend_map_path(), save_started_at)
            self.message = "地图保存完成，正在停止雷达与建图节点"
            self._publish_status()
            try:
                if process_group is not None:
                    os.killpg(process_group, signal.SIGINT)
                else:
                    process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=self.stop_timeout_sec)
            except subprocess.TimeoutExpired:
                self.message = "雷达与建图节点停止超时，正在强制结束"
                self._publish_status()
                try:
                    if process_group is not None:
                        os.killpg(process_group, signal.SIGTERM)
                    else:
                        process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=max(1.0, self.stop_timeout_sec * 0.5))
                except subprocess.TimeoutExpired:
                    try:
                        if process_group is not None:
                            os.killpg(process_group, signal.SIGKILL)
                        else:
                            process.kill()
                    except ProcessLookupError:
                        pass
                    process.wait()
            self.message = "雷达与建图节点正在退出，等待输出停止"
            self._publish_status()
            processes_exited = self._wait_for_process_group_exit(process_group)
            if not processes_exited:
                self.message = "雷达与建图节点仍在运行，正在强制结束"
                self._publish_status()
                try:
                    if process_group is not None:
                        os.killpg(process_group, signal.SIGKILL)
                    else:
                        process.kill()
                except ProcessLookupError:
                    pass
                processes_exited = self._wait_for_process_group_exit(process_group)
            output_quiet = self._wait_for_output_quiet()
            self._process = None
            self.state = "stopped" if processes_exited and output_quiet else "error"
            self._last_current_output_at = 0.0
            self.last_result = (
                f"stopped; save root: {save_root}; "
                f"metadata saved: {metadata_saved}; frontend map saved: {frontend_saved}; "
                f"processes exited: {processes_exited}; output quiet: {output_quiet}"
            )
            if not processes_exited:
                self.message = "地图已保存，但雷达与建图节点未完全退出"
            elif not output_quiet:
                self.message = "地图已保存，但雷达或建图话题仍有输出"
            elif map_saved:
                self.message = "建图已停止，累计地图已保存"
            elif metadata_saved:
                self.message = "建图已停止，已保存位姿，累计地图未生成"
            else:
                self.message = "建图已停止，已请求保存地图"
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
        self.state = "saving"
        self.message = "正在保存地图"
        self._publish_status()
        save_root, save_started_at = self._publish_save_trigger()
        self.message = "正在保存累计地图"
        self._publish_status()
        frontend_saved = self._save_frontend_map(save_started_at)
        self.message = "正在等待地图元数据"
        self._publish_status()
        metadata_saved = self._wait_for_saved_metadata(save_started_at, self.save_before_stop_timeout_sec)
        map_saved = frontend_saved or self._map_file_is_new(self._frontend_map_path(), save_started_at)
        self.state = "stopped"
        self._last_current_output_at = 0.0
        if map_saved:
            self.message = message
        elif metadata_saved:
            self.message = "已保存位姿，累计地图未生成"
        else:
            self.message = "已请求保存地图"
        self.last_result = (
            f"save requested: {save_root}; metadata saved: {metadata_saved}; "
            f"frontend map saved: {frontend_saved}"
        )

    def _publish_save_trigger(self) -> tuple[str, float]:
        self._ensure_session_name()
        save_root = self.save_root
        msg = String()
        msg.data = save_root
        self.save_pub.publish(msg)
        return save_root, time.time()

    def _wait_for_saved_metadata(self, started_at: float, timeout_sec: float) -> bool:
        if not self.session_name:
            return False
        session_path = Path(self.map_history_root) / self.session_name
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline:
            if self._session_has_new_metadata_files(session_path, started_at):
                return True
            time.sleep(0.2)
        return self._session_has_new_metadata_files(session_path, started_at)

    def _session_has_new_metadata_files(self, session_path: Path, started_at: float) -> bool:
        for filename in SAVED_METADATA_MARKERS:
            path = session_path / filename
            if not path.is_file():
                continue
            try:
                if path.stat().st_mtime >= started_at - 0.5:
                    return True
            except OSError:
                continue
        return False

    def _save_frontend_map(self, started_at: float) -> bool:
        if not self.fast_lio_map_save_service or not self.session_name:
            return False
        frontend_map_path = self._frontend_map_path()
        service_name = self.fast_lio_map_save_service
        if not service_name.startswith("/"):
            service_name = f"/{service_name}"
        if not self._service_is_available(service_name):
            self.get_logger().warning(f"FAST-LIO frontend map save service is unavailable: {service_name}")
            return self._wait_for_map_file(frontend_map_path, started_at, 1.0)
        try:
            result = subprocess.run(
                ["ros2", "service", "call", service_name, "std_srvs/srv/Trigger", "{}"],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.frontend_map_save_timeout_sec,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().warning(f"FAST-LIO frontend map save call failed: {exc}")
            return self._wait_for_map_file(frontend_map_path, started_at, 5.0)
        normalized_stdout = result.stdout.replace(" ", "").lower()
        if result.returncode != 0 or ("success=true" not in normalized_stdout and "success:true" not in normalized_stdout):
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            self.get_logger().warning(
                f"FAST-LIO frontend map save rejected: stdout={stdout} stderr={stderr}"
            )
            return False
        return self._wait_for_map_file(frontend_map_path, started_at, 5.0)

    def _frontend_map_path(self) -> Path:
        return Path(self.map_history_root) / self.session_name / f"{self.session_name}_map.pcd"

    def _service_is_available(self, service_name: str) -> bool:
        try:
            result = subprocess.run(
                ["ros2", "service", "type", service_name],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.frontend_map_service_wait_sec,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        service_type = result.stdout.strip()
        return result.returncode == 0 and service_type == "std_srvs/srv/Trigger"

    def _wait_for_map_file(self, path: Path, started_at: float, timeout_sec: float) -> bool:
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline:
            if self._map_file_is_new(path, started_at):
                return True
            time.sleep(0.2)
        return self._map_file_is_new(path, started_at)

    def _process_group(self, process: subprocess.Popen[str]) -> int | None:
        try:
            return os.getpgid(process.pid)
        except ProcessLookupError:
            return None

    def _wait_for_process_group_exit(self, process_group: int | None) -> bool:
        if process_group is None:
            return True
        timeout_sec = float(self.get_parameter("process_exit_timeout_sec").value)
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline:
            if not self._process_group_exists(process_group):
                return True
            time.sleep(0.1)
        return not self._process_group_exists(process_group)

    @staticmethod
    def _process_group_exists(process_group: int) -> bool:
        try:
            os.killpg(process_group, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _wait_for_output_quiet(self) -> bool:
        timeout_sec = float(self.get_parameter("output_quiet_timeout_sec").value)
        deadline = time.time() + max(0.0, timeout_sec)
        while time.time() < deadline:
            if self._output_topics_are_quiet():
                return True
            time.sleep(0.1)
        return self._output_topics_are_quiet()

    def _output_topics_are_quiet(self) -> bool:
        return all(self.count_publishers(topic) == 0 for topic in self.stop_wait_topics)

    @staticmethod
    def _split_topics(value: str) -> list[str]:
        return [topic.strip() for topic in value.split(",") if topic.strip()]

    @staticmethod
    def _file_is_new(path: Path, started_at: float) -> bool:
        if not path.is_file():
            return False
        try:
            return path.stat().st_mtime >= started_at - 0.5
        except OSError:
            return False

    @classmethod
    def _map_file_is_new(cls, path: Path, started_at: float) -> bool:
        return cls._file_is_new(path, started_at) and cls._pcd_has_points(path)

    @staticmethod
    def _pcd_has_points(path: Path) -> bool:
        try:
            with path.open("rb") as handle:
                header = handle.read(8192).decode("ascii", errors="ignore")
        except OSError:
            return False
        width = 0
        height = 1
        for line in header.splitlines():
            parts = line.strip().split()
            if len(parts) >= 2 and parts[0].upper() == "POINTS":
                try:
                    return int(parts[1]) > 0
                except ValueError:
                    return False
            if len(parts) >= 2 and parts[0].upper() == "WIDTH":
                try:
                    width = int(parts[1])
                except ValueError:
                    width = 0
            if len(parts) >= 2 and parts[0].upper() == "HEIGHT":
                try:
                    height = int(parts[1])
                except ValueError:
                    height = 1
            if line.strip().upper().startswith("DATA "):
                break
        return width * height > 0

    def _publish_status(self) -> None:
        self._refresh_process_state()
        if self.state in {"starting", "mapping"}:
            if self._has_recent_current_output():
                self.state = "mapping"
                self.message = "正在建图，地图持续更新中"
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
        if self.state in {"starting", "mapping"}:
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
