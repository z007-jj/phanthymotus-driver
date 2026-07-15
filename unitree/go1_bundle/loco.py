"""
loco.py — Go1 基础运动控制卡（HIGHLEVEL）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 里的卡名自动 import 并 make_plugin()。
下发经共享 client 的高层原语（move/stop_move/set_mode）。与 spin 卡共享同一份 HighCmd，
由 client 加锁合成，互不覆盖。

动作：
- move(vx_mps/vy_mps/yaw_speed_rad_s)：mode=2 速度行走，参数【按当前步态】校验，越界拒绝；约 0.5s 看门狗自停。
- stop_move：停下站稳。
- balance_stand(1)/stand_down(5,卧倒)/stand_up(6)/damp(7,需 confirm)/recovery_stand(8,需 confirm)。

前置：任何移动都要求狗【已站立】（软件无法从地面扶起——起立靠遥控；stand_up 对本机常无效，保留无害）。
⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。
"""

from __future__ import annotations

import time

CARD = "loco"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 基础运动。move(vx_mps/vy_mps/yaw_speed_rad_s 机体速度，按当前步态校验量程)、"
        "stop_move(停下站稳)、balance_stand(平衡站立)、stand_down(卧倒)、stand_up(起立)、"
        "damp(阻尼急停/省电，需 confirm)、recovery_stand(跌倒后恢复站立，需 confirm)。"
        "前置:狗必须【已站立】(软件无法从地面扶起，起立靠遥控);普通 move 需先切行走步态(switch_gait trot)，"
        "且约 0.5 秒后自动停。越界值直接拒绝，不静默裁剪。")

# 简单模式（HighCmd.mode）
_SIMPLE = {"balance_stand": 1, "stand_down": 5, "stand_up": 6, "damp": 7, "recovery_stand": 8}
_CONFIRM = {"damp", "recovery_stand"}
ACTIONS = ["move", "stop_move"] + list(_SIMPLE.keys())

# move 速度范围随步态（comm.h / MT §4.1）；idle / trot_obstacle 拒绝非零 move
GAIT_NAMES = {0: "idle", 1: "trot", 2: "trot_run", 3: "climb_stair", 4: "trot_obstacle"}
MOVE_RANGES = {
    "trot":        {"vx_mps": (-1.1, 1.5), "vy_mps": (-1.0, 1.0), "yaw_speed_rad_s": (-4.0, 4.0)},
    "trot_run":    {"vx_mps": (-2.5, 3.5), "vy_mps": (-1.0, 1.0), "yaw_speed_rad_s": (-4.0, 4.0)},
    "climb_stair": {"vx_mps": (-0.2, 0.25), "vy_mps": (-0.15, 0.15), "yaw_speed_rad_s": (-0.7, 0.7)},
}


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：无 ROS2 topic；经共享 client 下发高层运动命令。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": ACTIONS},
                        "vx_mps": {"type": "number", "description": "前后速度 m/s（+前/−后）"},
                        "vy_mps": {"type": "number", "description": "平移速度 m/s（+左/−右）"},
                        "yaw_speed_rad_s": {"type": "number", "description": "偏航角速度 rad/s（+左转/−右转）"},
                        "confirm": {"type": "boolean", "description": "damp/recovery_stand 需要"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "move": {"params": ["vx_mps", "vy_mps", "yaw_speed_rad_s"],
                                 "description": "按当前步态量程行走；约 0.5s 看门狗自停，连发持续走。"},
                        "stop_move": {"params": [], "description": "停下站稳。"},
                        "balance_stand": {"params": [], "description": "平衡站立(mode1)。"},
                        "stand_down": {"params": [], "description": "卧倒(mode5)。"},
                        "stand_up": {"params": [], "description": "起立(mode6，本机常需遥控)。"},
                        "damp": {"params": ["confirm"], "description": "阻尼急停/省电，电机变软(mode7)。"},
                        "recovery_stand": {"params": ["confirm"], "description": "跌倒后恢复站立(mode8)。"},
                    },
                }}

    def start(self):
        pass

    def stop(self):
        try:
            self._client.stop_move()
        except Exception:  # noqa: BLE001
            pass

    def dispatch(self, action, args):
        args = args or {}
        if action == "move":
            return self._move(args)
        if action == "stop_move":
            self._client.stop_move()
            return _ok("stop_move", {"mode": 0, "vx_mps": 0.0, "vy_mps": 0.0, "yaw_speed_rad_s": 0.0})
        if action in _SIMPLE:
            if action in _CONFIRM and not args.get("confirm"):
                return _err("PRECONDITION_FAILED", "action '%s' requires confirm=true" % action)
            self._client.set_mode(_SIMPLE[action])
            return _ok(action, {"mode": _SIMPLE[action]})
        # start/stop/info 生命周期
        if action in ("start",):
            return {"state": "ready"}
        if action in ("stop", "info"):
            return {"state": "idle" if action == "stop" else "running"}
        return None

    def _move(self, args):
        gait_name = GAIT_NAMES.get(self._client.get_gait(), "idle")
        rng = MOVE_RANGES.get(gait_name)
        if rng is None:
            # idle / trot_obstacle：拒绝非零 move（MT §4.1/§4.2）
            for k in ("vx_mps", "vy_mps", "yaw_speed_rad_s"):
                v = args.get(k)
                if v not in (None, 0, 0.0):
                    return _err("PRECONDITION_FAILED",
                                "non-zero move not allowed under gait '%s' (switch_gait to trot first)" % gait_name)
            self._client.move(0.0, 0.0, 0.0)
            return _ok("move", {"mode": 2, "gait": gait_name, "vx_mps": 0.0, "vy_mps": 0.0, "yaw_speed_rad_s": 0.0})
        vals = {}
        for k, (lo, hi) in rng.items():
            v = args.get(k, 0.0)
            try:
                v = float(v)
            except (TypeError, ValueError):
                return _err("INVALID_ARGUMENT", "'%s' must be a number" % k)
            if v < lo or v > hi:
                return _err("INVALID_ARGUMENT",
                            "'%s'=%s out of range [%s, %s] for gait '%s'" % (k, v, lo, hi, gait_name))
            vals[k] = v
        applied = self._client.move(vals["vx_mps"], vals["vy_mps"], vals["yaw_speed_rad_s"])
        return _ok("move", {"mode": 2, "gait": gait_name,
                            "vx_mps": applied["vx"], "vy_mps": applied["vy"],
                            "yaw_speed_rad_s": applied["vyaw"]})


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
