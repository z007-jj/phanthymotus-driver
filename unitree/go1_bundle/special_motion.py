"""
special_motion.py — Go1 官方特殊动作卡（HIGHLEVEL，高风险）。

自包含：一张卡 = 一个文件。main.py 按 config.yaml 自动 import 并 make_plugin()。
封装官方特殊模式为【异步动作序列】：持续发 mode=1 稳定 → 发 mode=10/11 执行 → 到超时回 mode=1。
不让调用方直接写 mode 或自行延长时长（动作时长是本卡白名单配置）。

动作：jump_yaw_left(mode=10 跳跃左转) / straight_hand(mode=11 前肢伸展)。均需 confirm=true。
⚠️ 大幅度动态动作 —— 必须确保狗周围空旷、已站立。控制卡须上真机验证后才能上架（CONTRIBUTING.md §4）。
"""

from __future__ import annotations

import threading
import time

CARD = "special_motion"
TYPE = "actuator"
CONTROL_LEVEL = "HIGHLEVEL"
DESC = ("Go1 官方特殊动作(高风险,需空旷+已站立):jump_yaw_left(跳跃左转)/straight_hand(前肢伸展/作揖)。"
        "驱动按序封装(稳定 mode1 → 执行 → 自动回站立)，异步立即返回;stop 或 damp 可打断。均需 confirm=true。")

# HighCmd.mode（comm.h）
_SPECIAL = {"jump_yaw_left": 10, "straight_hand": 11}
STAND_MODE = 1

STABILIZE_S = 1.5        # 执行前持续力控站立稳定
ACTION_S = 4.0           # 特殊模式持续时长（白名单；到时回站立）


def _ms() -> int:
    return int(time.time() * 1000)


def _ok(action, applied) -> dict:
    return {"ok": True, "card": CARD, "action": action, "control_level": CONTROL_LEVEL,
            "applied": applied, "timestamp_ms": _ms()}


def _err(code, message) -> dict:
    return {"ok": False, "code": code, "message": message}


class Plugin:
    """控制卡插件：异步特殊动作序列（单后台线程，可被 stop/damp 打断）。"""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        cfg = plugin_config or {}
        self._stabilize_s = float(cfg.get("stabilize_s", STABILIZE_S))
        self._action_s = float(cfg.get("action_s", ACTION_S))
        self._cancel = threading.Event()
        self._thread = None
        self._seq = 0

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {
                    "type": "object",
                    "properties": {
                        "action": {"type": "string", "enum": list(_SPECIAL.keys()) + ["stop"]},
                        "confirm": {"type": "boolean", "description": "jump_yaw_left/straight_hand 需要"},
                    },
                    "required": ["action"],
                    "x-action-params": {
                        "jump_yaw_left": {"params": ["confirm"], "description": "跳跃左转(mode10)。异步,需空旷。"},
                        "straight_hand": {"params": ["confirm"], "description": "前肢伸展/作揖(mode11)。异步。"},
                        "stop": {"params": [], "description": "打断进行中的特殊动作，回站立。"},
                    },
                }}

    def start(self):
        pass

    def stop(self):
        self._cancel.set()
        try:
            self._client.set_mode(STAND_MODE)
        except Exception:  # noqa: BLE001
            pass

    def dispatch(self, action, args):
        args = args or {}
        if action in ("start",):
            return {"state": "ready"}
        if action == "info":
            return {"state": "running"}
        if action == "stop":
            self._cancel.set()
            self._client.set_mode(STAND_MODE)
            return _ok("stop", {"stopped": True, "mode": STAND_MODE})
        if action not in _SPECIAL:
            return _err("INVALID_ARGUMENT", "unknown special '%s'" % action)
        if not args.get("confirm"):
            return _err("PRECONDITION_FAILED", "action '%s' requires confirm=true" % action)

        # 打断上一个未结束的序列，再起新的
        self._cancel.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._cancel = threading.Event()
        self._seq += 1
        seq = self._seq
        self._thread = threading.Thread(target=self._run, args=(_SPECIAL[action], self._cancel),
                                        daemon=True, name="go1_special_motion")
        self._thread.start()
        return _ok(action, {"sequence_id": seq, "state": "stabilizing", "mode": _SPECIAL[action]})

    def _run(self, mode, cancel):
        """稳定 → 执行特殊模式 → 回站立。分段 sleep，随时可被 cancel 打断。"""
        client = self._client
        try:
            client.set_mode(STAND_MODE)
            if self._wait(cancel, self._stabilize_s):
                return
            client.set_mode(mode)
            if self._wait(cancel, self._action_s):
                return
        finally:
            client.set_mode(STAND_MODE)

    @staticmethod
    def _wait(cancel, seconds) -> bool:
        """分段等待，返回 True 表示被 cancel 打断。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cancel.is_set():
                return True
            time.sleep(0.05)
        return cancel.is_set()


def make_plugin(plugin_config, namespace, executor, client):
    """main.py 装配入口。"""
    return Plugin(plugin_config, namespace, executor, client)
