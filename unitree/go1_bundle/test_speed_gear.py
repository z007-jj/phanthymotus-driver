"""
test_speed_gear.py — Go1 速度档位卡(纯偏好设置，不下发任何 HighCmd)。

⚠️ 文件名 test_ 前缀 = **尚未真机验收**（团队约定：未验收卡片加 test 前缀，
   验收通过后去掉前缀 → speed_gear）。故当前卡名/工具名亦为 test_speed_gear。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。
经共享 client 的 set_speed_gear/speed_gear/gear_speed/gear_yaw_rate 读写档位——
这些方法只改一个内存偏好值，**不发任何命令、不让狗动**。

作用：把"走快点/走慢点"这类自然语言映射到 slow/normal/fast 档；闭环便捷动作
（未来 move_distance/turn_angle、转圈 spin 等）在**没有显式给速度**时按当前档取缺省速度。
不影响直接 loco.move（它自带 vx/vy/vyaw，显式优先）。

动作：slow / normal / fast（设档）、get（查当前档 + 解析出的速度值）。
"""

from __future__ import annotations

import time

CARD = "test_speed_gear"        # 未验收 → test 前缀；验收通过后改回 speed_gear（并同步 config/Dockerfile/文件名）
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
GEARS = ["slow", "normal", "fast"]
DESC = ("Go1 速度档位:slow(慢)/normal(中)/fast(快)/get(查当前)。把'走快点/走慢点'映射到档位;"
        "闭环便捷动作(move_distance/turn_angle、spin 等)在未显式给速度时按当前档取缺省速度。"
        "纯偏好设置,不下发命令、不让狗动;不影响显式 loco.move。")


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """速度档位卡：slow/normal/fast 设档，get 查当前档 + 解析速度。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": GEARS + ["get"]},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "slow":   {"params": [], "description": "设为慢档"},
                        "normal": {"params": [], "description": "设为中档(默认)"},
                        "fast":   {"params": [], "description": "设为快档"},
                        "get":    {"params": [], "description": "查当前档位与解析出的速度值"},
                    },
                }}

    def start(self):
        return {"state": "ready"}

    def stop(self):
        return {"state": "idle"}

    def _resolved(self) -> dict:
        """当前档位 + 解析出的缺省速度（供 get 与设档后回显）。"""
        return {"gear": self._client.speed_gear(),
                "default_speed_mps": round(float(self._client.gear_speed()), 3),
                "default_yaw_rate_rad_s": round(float(self._client.gear_yaw_rate()), 3)}

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action in ("info", "get"):
            return _ok("get", self._resolved())
        if action in GEARS:
            self._client.set_speed_gear(action)
            return _ok(action, self._resolved())
        return _err("INVALID_ARGUMENT", "unknown action '%s'（应为 slow/normal/fast/get）" % action)


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
