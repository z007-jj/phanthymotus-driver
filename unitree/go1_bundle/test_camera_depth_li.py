#!/usr/bin/env python3
"""
test_camera_depth_li.py — Go1 深度推流卡（li 版，双路输出）
- 现有JPEG路: 彩色深度图发布到/{ns}/camera/{position}/depth，兼容画布显示
- 新增RAW路(默认关闭): 原始16UC1深度(毫米单位)发布到/{ns}/camera/{position}/depth_raw，用于避障
开启RAW路仅需在config中设置enable_raw: true即可，不影响现有功能
"""

from __future__ import annotations

import socket
import select
import struct
import threading
import time

# 可选依赖：opencv用于图像翻转
try:
    import cv2
    import numpy as np
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False
    _QOS = None

CARD = "test_camera_depth_li"
TYPE = "sensor"
FMT_JPEG = "image/jpeg"
FMT_RAW = "image/depth-z16"

_CONNECT_TIMEOUT = 8.0
_FIRST_FRAME_TIMEOUT = 15.0
_STEADY_TIMEOUT = 8.0

# 机位配置: JPEG端口沿用原91xx, RAW端口自动+200为93xx
_DEFAULT_POSITIONS = {
    "front": {"board_ip": "192.168.123.13", "depth_port": 9101},
    "chin":  {"board_ip": "192.168.123.13", "depth_port": 9102},
    "left":  {"board_ip": "192.168.123.14", "depth_port": 9103},
    "right": {"board_ip": "192.168.123.14", "depth_port": 9104},
    "belly": {"board_ip": "192.168.123.15", "depth_port": 9105},
}
_POS_TITLE = {"front": "Front (头部前 dev1)", "chin": "Chin (头部下 dev0)",
              "left": "Left (侧左 dev0)", "right": "Right (侧右 dev1)", "belly": "Belly (腹部 dev0)"}
_VALID_POSITIONS = list(_DEFAULT_POSITIONS.keys())

DESC = ("Go1 五机位深度流（彩色JPEG用于画布显示，可选16UC1原始深度用于避障算法，毫米单位）— multiInstance，position下拉选机位。"
        "config.enable_raw=true时同时发布原始深度数据到depth_raw话题，不开启则完全兼容原有功能。")


def _err(code: str, message: str, **extra) -> dict:
    return {"ok": False, "code": code, "message": message, **extra}


def _now_ms() -> int:
    return int(time.time() * 1000)


class _DepthStream:
    """双路深度流桥接，JPEG路用于显示，RAW路用于避障"""
    def __init__(self, node: "Node", topic_jpeg: str, topic_raw: str):
        self._node = node
        self._topic_jpeg = topic_jpeg
        self._topic_raw = topic_raw
        self._pub_jpeg = node.create_publisher(CompressedImage, topic_jpeg, _QOS) if _HAS_ROS2 else None
        self._pub_raw = None  # RAW发布器按需创建
        self._run = False
        self._gen = 0
        self.connected = False
        self.connected_raw = False
        self.frames = 0
        self.frames_raw = 0
        self.position = None
        self.enable_raw = False
        self._last_publish_ms = 0
        self._MIN_INTERVAL_MS = 30
        self._MAX_DRAIN_BYTES = 1_048_576

    def start(self, position: str, host: str, port_jpeg: int, enable_raw: bool = False):
        self._run = True
        self._gen += 1
        gen = self._gen
        self.position = position
        self.enable_raw = enable_raw
        self.connected = False
        self.connected_raw = False

        # 按需创建RAW发布器
        if enable_raw and _HAS_ROS2 and self._pub_raw is None:
            self._pub_raw = self._node.create_publisher(Image, self._topic_raw, _QOS)

        # 启动JPEG流线程
        threading.Thread(target=self._jpeg_loop, args=(gen, position, host, port_jpeg), daemon=True).start()
        # 按需启动RAW流线程
        if enable_raw:
            port_raw = port_jpeg + 200
            threading.Thread(target=self._raw_loop, args=(gen, position, host, port_raw), daemon=True).start()

    def stop(self):
        self._run = False
        self._gen += 1
        self.connected = False
        self.connected_raw = False

    # JPEG流循环(原逻辑,兼容画布显示)
    def _jpeg_loop(self, gen, position, host, port):
        while self._run and gen == self._gen:
            try:
                s = socket.create_connection((host, port), timeout=_CONNECT_TIMEOUT)
                s.setblocking(False)
                self.connected = True
                self._node.get_logger().info(
                    f"[{position}] JPEG路已连上 {host}:{port}")
            except Exception:
                self.connected = False
                time.sleep(2)
                continue
            try:
                got_first = False
                rx = bytearray()
                while self._run and gen == self._gen:
                    timeout = _STEADY_TIMEOUT if got_first else _FIRST_FRAME_TIMEOUT
                    readable, _, _ = select.select([s], [], [], timeout)
                    if not readable:
                        raise TimeoutError("jpeg timed out")

                    received = 0
                    peer_closed = False
                    while received < self._MAX_DRAIN_BYTES:
                        try:
                            chunk = s.recv(min(65_536, self._MAX_DRAIN_BYTES - received))
                        except BlockingIOError:
                            break
                        if not chunk:
                            peer_closed = True
                            break
                        rx.extend(chunk)
                        received += len(chunk)
                    if peer_closed:
                        break

                    latest = None
                    complete_frames = 0
                    while len(rx) >= 4:
                        n = struct.unpack(">I", rx[:4])[0]
                        if n <= 0 or n > 5_000_000:
                            raise ValueError(f"invalid JPEG length: {n}")
                        end = 4 + n
                        if len(rx) < end:
                            break
                        latest = bytes(rx[4:end])
                        del rx[:end]
                        complete_frames += 1
                    self.frames += complete_frames
                    if latest is None:
                        continue

                    if not got_first:
                        got_first = True
                        self._node.get_logger().info(f"[{position}] JPEG首帧到达")

                    now_ms = int(time.time() * 1000)
                    if now_ms - self._last_publish_ms < self._MIN_INTERVAL_MS:
                        continue

                    if self._pub_jpeg is not None:
                        # 可选翻转(无cv2时跳过)
                        if _HAS_CV2:
                            img = cv2.imdecode(np.frombuffer(latest, dtype=np.uint8), cv2.IMREAD_COLOR)
                            _, encoded = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                            latest = encoded.tobytes()

                        msg = CompressedImage()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_depth"
                        msg.format = "jpeg"
                        msg.data = latest
                        try:
                            self._pub_jpeg.publish(msg)
                            self._last_publish_ms = now_ms
                        except Exception:
                            break
            except Exception as e:
                self._node.get_logger().warn(f"[{position}] JPEG流中断: {e}")
            finally:
                self.connected = False
                try:
                    s.close()
                except Exception:
                    pass

    # RAW原始深度流循环(新增,独立线程不影响JPEG路)
    def _raw_loop(self, gen, position, host, port):
        while self._run and gen == self._gen and self.enable_raw:
            try:
                s = socket.create_connection((host, port), timeout=_CONNECT_TIMEOUT)
                s.setblocking(False)
                self.connected_raw = True
                self._node.get_logger().info(
                    f"[{position}] RAW原始深度路已连上 {host}:{port}")
            except Exception:
                self.connected_raw = False
                time.sleep(2)
                continue
            try:
                got_first = False
                rx = bytearray()
                while self._run and gen == self._gen and self.enable_raw:
                    timeout = _STEADY_TIMEOUT if got_first else _FIRST_FRAME_TIMEOUT
                    readable, _, _ = select.select([s], [], [], timeout)
                    if not readable:
                        raise TimeoutError("raw timed out")

                    received = 0
                    peer_closed = False
                    while received < self._MAX_DRAIN_BYTES:
                        try:
                            chunk = s.recv(min(65_536, self._MAX_DRAIN_BYTES - received))
                        except BlockingIOError:
                            break
                        if not chunk:
                            peer_closed = True
                            break
                        rx.extend(chunk)
                        received += len(chunk)
                    if peer_closed:
                        break

                    latest = None
                    latest_w = 0
                    latest_h = 0
                    complete_frames = 0
                    while len(rx) >= 8:
                        w = struct.unpack(">I", rx[:4])[0]
                        h = struct.unpack(">I", rx[4:8])[0]
                        data_size = w * h * 2
                        if data_size <= 0 or data_size > 5_000_000:
                            raise ValueError(f"invalid raw size: {w}x{h}")
                        end = 8 + data_size
                        if len(rx) < end:
                            break
                        latest = bytes(rx[8:end])
                        latest_w = w
                        latest_h = h
                        del rx[:end]
                        complete_frames += 1
                    self.frames_raw += complete_frames
                    if latest is None:
                        continue

                    if not got_first:
                        got_first = True
                        self._node.get_logger().info(
                            f"[{position}] RAW首帧到达 {latest_w}x{latest_h} 16UC1(mm)")
                        if latest_w not in (464, 640) or latest_h not in (400, 480):
                            self._node.get_logger().warn(
                                f"[{position}] RAW帧尺寸异常 {latest_w}x{latest_h}，期望~464x400")

                    now_ms = int(time.time() * 1000)
                    if now_ms - self._last_publish_ms < self._MIN_INTERVAL_MS:
                        continue

                    if self._pub_raw is not None:
                        msg = Image()
                        msg.header.stamp = self._node.get_clock().now().to_msg()
                        msg.header.frame_id = f"go1_{position}_depth_raw"
                        msg.encoding = "16UC1"
                        msg.height = latest_h
                        msg.width = latest_w
                        msg.is_bigendian = 0
                        msg.step = latest_w * 2
                        msg.data = list(latest)
                        try:
                            self._pub_raw.publish(msg)
                        except Exception:
                            break
            except Exception as e:
                self._node.get_logger().warn(f"[{position}] RAW流中断: {e}")
            finally:
                self.connected_raw = False
                try:
                    s.close()
                except Exception:
                    pass


class CameraDepthPlugin:
    PREFIX = "test_camera_depth_li"

    def __init__(self, plugin_config, namespace, executor, client):
        c = plugin_config or {}
        self._ns = namespace
        self._executor = executor
        self._positions = {p: dict(v) for p, v in _DEFAULT_POSITIONS.items()}
        for pos, ov in (c.get("positions") or {}).items():
            if pos in self._positions and isinstance(ov, dict):
                self._positions[pos].update(ov)
        self._default_pos = str(c.get("default_position", "front")).lower()
        if self._default_pos not in self._positions:
            self._default_pos = "front"
        self._enable_raw_default = bool(c.get("enable_raw", False))
        self._node = None
        self._streams: dict = {}
        self._cfg: dict = {}
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_test_camera_depth_li")
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2不可用: {e}", flush=True)
                self._node = None
        print(f"[{CARD}] 机位就绪：{sorted(self._positions.keys())}（default={self._default_pos}, enable_raw_default={self._enable_raw_default}）", flush=True)

    def _topic_jpeg(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/depth"

    def _topic_raw(self, iid: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in iid)
        return f"/{self._ns}/camera/{safe}/depth_raw"

    def _resolve_pos(self, iid: str, args: dict) -> str:
        cfg = args.get("config") or {}
        cand = (cfg.get("position") or args.get("position") or args.get("camera_source")
                or self._cfg.get(iid, {}).get("position"))
        if not cand:
            cand = iid if iid in self._positions else self._default_pos
        pos = str(cand).lower()
        return pos if pos in self._positions else self._default_pos

    def _resolve_enable_raw(self, iid: str, args: dict) -> bool:
        cfg = args.get("config") or {}
        if "enable_raw" in cfg:
            return bool(cfg.get("enable_raw"))
        if iid in self._cfg and "enable_raw" in self._cfg[iid]:
            return bool(self._cfg[iid]["enable_raw"])
        return self._enable_raw_default

    def _stream_for(self, iid: str) -> "_DepthStream":
        st = self._streams.get(iid)
        if st is None:
            st = _DepthStream(self._node, self._topic_jpeg(iid), self._topic_raw(iid))
            self._streams[iid] = st
        return st

    def start(self):
        if self._node is None:
            print(f"[{CARD}] 无rclpy/executor，推流不可用", flush=True)

    def stop(self):
        for st in self._streams.values():
            try:
                st.stop()
            except Exception:
                pass

    def _start_instance(self, iid: str, position: str, enable_raw: bool) -> dict:
        if position not in self._positions:
            return _err("INVALID_ARGUMENT", f"unknown position {position!r}; valid: {_VALID_POSITIONS}")
        if self._node is None:
            return _err("COMMUNICATION_ERROR", "no rclpy/executor")
        p = self._positions[position]
        st = self._stream_for(iid)
        st.stop()
        st.start(position, p["board_ip"], int(p["depth_port"]), enable_raw)
        topics_out = [{"topic": self._topic_jpeg(iid), "format": FMT_JPEG}]
        if enable_raw:
            topics_out.append({"topic": self._topic_raw(iid), "format": FMT_RAW})
        return {"ok": True, "card": CARD, "action": "start", "timestamp_ms": _now_ms(),
                "state": "running", "position": position, "enable_raw": enable_raw,
                "topic_out": topics_out}

    def get_tools(self) -> list:
        return [{
            "name": CARD, "type": TYPE, "multiInstance": True,
            "description": DESC,
            "configSchema": {
                "type": "object",
                "properties": {
                    "position": {
                        "type": "string",
                        "description": "读取哪一路相机的深度",
                        "scope": "instance",
                        "oneOf": [{"const": p, "title": _POS_TITLE[p]} for p in _VALID_POSITIONS],
                    },
                    "enable_raw": {
                        "type": "boolean",
                        "description": "是否同时发布原始16UC1深度数据到depth_raw话题(用于避障)",
                        "scope": "instance",
                        "default": False
                    }
                },
            },
            "inputSchema": {
                "type": "object",
                "properties": {"action": {"type": "string", "enum": ["info", "start", "stop"],
                                          "description": "start=推流 / stop=释放相机 / info=查询状态"}},
                "required": ["action"],
            },
            "topic_out": [],
        }]

    def dispatch(self, action, args) -> dict | None:
        iid = args.get("instance_id") or "default"

        if action == "config":
            pos = self._resolve_pos(iid, args)
            enable_raw = self._resolve_enable_raw(iid, args)
            if pos not in self._positions:
                return _err("INVALID_ARGUMENT", f"unknown position {pos!r}; valid: {_VALID_POSITIONS}")
            self._cfg[iid] = {"position": pos, "enable_raw": enable_raw}
            st = self._streams.get(iid)
            need_restart = False
            if st is not None and st._run:
                if st.position != pos or st.enable_raw != enable_raw:
                    need_restart = True
            if need_restart:
                self._start_instance(iid, pos, enable_raw)
            topics_out = [{"topic": self._topic_jpeg(iid), "format": FMT_JPEG}]
            if enable_raw:
                topics_out.append({"topic": self._topic_raw(iid), "format": FMT_RAW})
            return {"ok": True, "position": pos, "enable_raw": enable_raw, "topic_out": topics_out}

        if action == "start":
            pos = self._resolve_pos(iid, args)
            enable_raw = self._resolve_enable_raw(iid, args)
            return self._start_instance(iid, pos, enable_raw)

        if action == "stop":
            st = self._streams.get(iid)
            if st is not None:
                st.stop()
            return {"ok": True, "card": CARD, "action": "stop", "timestamp_ms": _now_ms(),
                    "state": "idle", "position": self._cfg.get(iid, {}).get("position", self._default_pos)}

        if action in ("info", "read", "get", CARD):
            pos = self._resolve_pos(iid, args)
            enable_raw = self._resolve_enable_raw(iid, args)
            p = self._positions.get(pos, {})
            st = self._streams.get(iid)
            state = "running" if (st and st.connected) else ("waiting" if st and st._run else "idle")
            topics_out = [{"topic": self._topic_jpeg(iid), "format": FMT_JPEG}]
            if enable_raw:
                topics_out.append({"topic": self._topic_raw(iid), "format": FMT_RAW})
            base = {
                "state": state, "position": pos, "enable_raw": enable_raw,
                "positions_available": _VALID_POSITIONS,
                "format_jpeg": "sensor_msgs/CompressedImage (jpeg, colorized depth)",
                "format_raw": "sensor_msgs/Image (16UC1, raw depth in mm)",
                "source_jpeg": f"{pos} @ {p.get('board_ip')}:{p.get('depth_port')}",
                "source_raw": f"{pos} @ {p.get('board_ip')}:{p.get('depth_port') + 200 if p.get('depth_port') else None}",
                "connected_jpeg": bool(st and st.connected),
                "connected_raw": bool(st and st.connected_raw),
                "frames_jpeg": st.frames if st else 0,
                "frames_raw": st.frames_raw if st else 0,
                "topic_out": topics_out,
            }
            return base
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return CameraDepthPlugin(plugin_config, namespace, executor, client)