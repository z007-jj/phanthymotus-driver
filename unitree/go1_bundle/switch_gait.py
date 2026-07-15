"""
switch_gait.py — Go1 步态切换卡（HIGHLEVEL）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。
只设定【期望步态】——实际移动由 loco.move 触发（mode=2 时步态才生效）。
典型流程：switch_gait 'trot' → loco 'move'。

trot_run / climb_stair / trot_obstacle 需 confirm=true（更快/更专的步态）。
⚠️ 控制卡须上真机验证量程+安全后才能上架（见 CONTRIBUTING.md §4）。
"""

from __future__ import annotations

import time

CARD = "switch_gait"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 步态切换:idle(原地)/trot(小跑)/trot_run(快跑)/climb_stair(爬楼梯)/trot_obstacle(越障)。"
        "只设定【期望步态】——实际移动由 loco.move 触发。典型流程:switch_gait 'trot' → loco 'move'。"
        "trot_run/climb_stair/trot_obstacle 需 confirm=true。")

# 步态名 → gaitType（comm.h）
GAITS = {"idle": 0, "trot": 1, "trot_run": 2, "climb_stair": 3, "trot_obstacle": 4}
_CONFIRM = {"trot_run", "climb_stair", "trot_obstacle"}


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：设期望步态；经共享 client 的 set_gait。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": list(GAITS.keys()),
                                   "description": "目标步态名"},
                        "confirm": {"type": "boolean", "description": "trot_run/climb_stair/trot_obstacle 需要"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        g: ({"params": ["confirm"]} if g in _CONFIRM else {"params": []})
                        for g in GAITS
                    },
                }}

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        args = args or {}
        if action in ("start",):
            return {"state": "ready"}
        if action in ("stop", "info"):
            return {"state": "idle" if action == "stop" else "running"}
        if action not in GAITS:
            return _err("INVALID_ARGUMENT", "unknown gait '%s'" % action)
        if action in _CONFIRM and not args.get("confirm"):
            return _err("PRECONDITION_FAILED", "gait '%s' requires confirm=true" % action)
        self._client.set_gait(GAITS[action])
        return _ok(action, {"gait_type": GAITS[action], "gait": action})


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
