"""
body_pose.py — Go1 机身姿态/高度卡（HIGHLEVEL）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。
下发经共享 client 的 set_pose（mode=1 力控站立 + euler/bodyHeight/footRaiseHeight）。
卡内持一份姿态组合，各动作只改自己那部分，其余保持（互不清零）。

动作：set_attitude(roll/pitch/yaw 弧度)、set_body_height(offset_m)、set_foot_raise_height(offset_m)、reset。
偏移量相对默认姿态；越界值直接拒绝，不静默裁剪。前置:狗必须【已站立】。
⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。
"""

from __future__ import annotations

import time

CARD = "body_pose"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 机身姿态/高度(狗须站立):set_attitude(roll/pitch/yaw 弧度)、set_body_height(offset_m 机身高度偏移)、"
        "set_foot_raise_height(offset_m 抬脚高度偏移)、reset(复位)。偏移相对默认姿态;越界值直接拒绝。")

ATT_RANGES = {"roll_rad": (-0.75, 0.75), "pitch_rad": (-0.75, 0.75), "yaw_rad": (-0.6, 0.6)}
BODY_H_RANGE = (-0.13, 0.03)
FOOT_H_RANGE = (-0.06, 0.03)
_ACTIONS = ["set_attitude", "set_body_height", "set_foot_raise_height", "reset"]


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：持姿态组合；经共享 client 的 set_pose 下发。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._pose = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "body_height": 0.0, "foot_raise": 0.0}

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": _ACTIONS},
                        "roll_rad": {"type": "number", "description": "set_attitude:[-0.75, 0.75]"},
                        "pitch_rad": {"type": "number", "description": "set_attitude:[-0.75, 0.75]"},
                        "yaw_rad": {"type": "number", "description": "set_attitude:[-0.6, 0.6]"},
                        "offset_m": {"type": "number",
                                     "description": "set_body_height:[-0.13,0.03] / set_foot_raise_height:[-0.06,0.03]"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "set_attitude": {"params": ["roll_rad", "pitch_rad", "yaw_rad"],
                                         "description": "机身姿态(欧拉角，站立力控)。"},
                        "set_body_height": {"params": ["offset_m"], "description": "机身高度偏移。"},
                        "set_foot_raise_height": {"params": ["offset_m"], "description": "行走抬脚高度偏移。"},
                        "reset": {"params": [], "description": "姿态/高度全部复位。"},
                    },
                }}

    def start(self):
        pass

    def stop(self):
        pass

    def _apply(self):
        p = self._pose
        return self._client.set_pose(p["roll"], p["pitch"], p["yaw"], p["body_height"], p["foot_raise"])

    def _ranged(self, args, key, lo, hi):
        v = args.get(key, 0.0)
        try:
            v = float(v)
        except (TypeError, ValueError):
            return None, _err("INVALID_ARGUMENT", "'%s' must be a number" % key)
        if v < lo or v > hi:
            return None, _err("INVALID_ARGUMENT", "'%s'=%s out of range [%s, %s]" % (key, v, lo, hi))
        return v, None

    def dispatch(self, action, args):
        args = args or {}
        if action in ("start",):
            return {"state": "ready"}
        if action in ("stop", "info"):
            return {"state": "idle" if action == "stop" else "running"}
        if action == "set_attitude":
            vals = {}
            for k, (lo, hi) in ATT_RANGES.items():
                v, e = self._ranged(args, k, lo, hi)
                if e:
                    return e
                vals[k] = v
            self._pose.update(roll=vals["roll_rad"], pitch=vals["pitch_rad"], yaw=vals["yaw_rad"])
            self._apply()
            return _ok(action, {"roll_rad": vals["roll_rad"], "pitch_rad": vals["pitch_rad"],
                                "yaw_rad": vals["yaw_rad"], "mode": 1})
        if action == "set_body_height":
            v, e = self._ranged(args, "offset_m", *BODY_H_RANGE)
            if e:
                return e
            self._pose["body_height"] = v
            self._apply()
            return _ok(action, {"body_height_offset_m": v})
        if action == "set_foot_raise_height":
            v, e = self._ranged(args, "offset_m", *FOOT_H_RANGE)
            if e:
                return e
            self._pose["foot_raise"] = v
            self._apply()
            return _ok(action, {"foot_raise_height_offset_m": v})
        if action == "reset":
            self._pose = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "body_height": 0.0, "foot_raise": 0.0}
            self._apply()
            return _ok(action, {"roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0,
                                "body_height_offset_m": 0.0, "foot_raise_height_offset_m": 0.0})
        return None


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
