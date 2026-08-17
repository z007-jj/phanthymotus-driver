"""Verified sensor and audio cards for the RobotEra Q5 bundle.

Direct base, arm, head, and hand cards live in ``direct_control.py``. This
module contains the verified state, battery, audio, and D455 camera cards.
"""

from __future__ import annotations

import io
import audioop
import base64
import binascii
import os
import re
import shlex
import struct
import subprocess
import threading
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import AudioPlay
from xbot_common_interfaces.srv import SetVolume

# main.py resolves all card classes through this module. Keep the direct
# control cards here as explicit exports while their implementation remains
# consolidated in direct_control.py.
from legacy_direct_control import (
    ArmControlPlugin,
)


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

def _q5_ssh_args(command: str):
    return [
        "sshpass", "-p", "developer", "ssh", "-p", "2222",
        "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
        "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
        "-o", "UserKnownHostsFile=/dev/null", "developer@192.168.8.100",
        f"bash -lc {shlex.quote(command)}",
    ]


def _q5_remote_command(command: str, timeout: float = 20.0, stdin=None):
    """Run a noninteractive command in Q5's documented developer container."""
    return subprocess.run(_q5_ssh_args(command), input=stdin, capture_output=True, timeout=timeout)


_Q5_MIC_PIDFILE = "/tmp/phanthymotus-q5-mic-capture.pid"
_Q5_SPEAKER_PIDFILE = "/tmp/phanthymotus-q5-speaker-playback.pid"


def _stop_remote_mic_capture() -> None:
    """Stop only the tagged microphone process left by this driver."""
    command = f"""pidfile={shlex.quote(_Q5_MIC_PIDFILE)}
if test -r \"$pidfile\"; then
  pid=$(cat \"$pidfile\" 2>/dev/null || true)
  if test -n \"$pid\" && test -r \"/proc/$pid/cmdline\" && grep -aq q5_mic_capture \"/proc/$pid/cmdline\"; then
    kill \"$pid\" 2>/dev/null || true
  fi
  rm -f \"$pidfile\"
fi"""
    try:
        _q5_remote_command(command, timeout=5.0)
    except Exception:
        pass


def _q5_mic_capture_shell(command: str) -> str:
    """Tag the remote PCM process so restart cleanup cannot leave it owning ALSA."""
    return f"""pidfile={shlex.quote(_Q5_MIC_PIDFILE)}
echo $$ > \"$pidfile\"
exec -a q5_mic_capture python3 -u -c {shlex.quote(command)}"""


def _stop_remote_speaker_playback() -> None:
    """Stop only a tagged direct-ALSA speaker left by this driver."""
    command = f"""pidfile={shlex.quote(_Q5_SPEAKER_PIDFILE)}
if test -r \"$pidfile\"; then
  pid=$(cat \"$pidfile\" 2>/dev/null || true)
  if test -n \"$pid\" && test -r \"/proc/$pid/cmdline\" && grep -aq q5_speaker_playback \"/proc/$pid/cmdline\"; then
    kill \"$pid\" 2>/dev/null || true
  fi
  rm -f \"$pidfile\"
fi"""
    try:
        _q5_remote_command(command, timeout=5.0)
    except Exception:
        pass


def _q5_speaker_playback_shell(command: str) -> str:
    """Tag the remote process so restarts cannot leave ALSA playback busy."""
    return f"""pidfile={shlex.quote(_Q5_SPEAKER_PIDFILE)}
echo $$ > \"$pidfile\"
exec -a q5_speaker_playback python3 -u -c {shlex.quote(command)}"""


def _q5_remote_playback_holders() -> str:
    """Return processes with an open Q5 playback PCM device for diagnostics."""
    probe = r"""from pathlib import Path
target = '/dev/snd/pcmC2D0p'
holders = []
for process in Path('/proc').glob('[0-9]*'):
    try:
        for fd in (process / 'fd').iterdir():
            if fd.resolve() == Path(target):
                cmdline = (process / 'cmdline').read_bytes().replace(b'\\0', b' ').decode(errors='replace').strip()
                holders.append('%s:%s' % (process.name, cmdline or '[no cmdline]'))
                break
    except OSError:
        continue
print('; '.join(holders) or 'none')"""
    try:
        result = _q5_remote_command("python3 -c " + shlex.quote(probe), timeout=5.0)
        if not result.returncode:
            return result.stdout.decode(errors="replace").strip() or "none"
    except Exception:
        pass
    return "unavailable"


def _raise_if_remote_process_exited(process, label: str) -> None:
    """Surface setup failures before a card falsely reports a running stream."""
    time.sleep(0.25)
    returncode = process.poll()
    if returncode is None:
        return
    detail = b""
    if process.stderr:
        try:
            detail = process.stderr.read()
        except Exception:
            pass
    message = detail.decode(errors="replace").strip() or f"remote process exited with code {returncode}"
    raise RuntimeError(f"Q5 {label} stream failed to start: {message}")


def _find_remote_mic_device() -> str:
    """Find the documented Q5 capture endpoint without PortAudio enumeration.

    The manual uses the full-duplex ``USB Audio Device`` (its device index is
    not stable), while POROSVOC is a capture-only fallback on some machines.
    Return ``plughw`` for the full-duplex card so ALSA can convert its native
    USB rate to the bridge's 16 kHz mono contract.
    """
    probe = r"""from pathlib import Path
sound = Path('/sys/class/sound')
candidates = []
for card in sound.glob('card*'):
    try:
        index = int(card.name[4:])
        name = (card / 'id').read_text().strip()
    except (OSError, ValueError):
        continue
    capture = Path('/dev/snd/pcmC%dD0c' % index).exists()
    playback = Path('/dev/snd/pcmC%dD0p' % index).exists()
    if capture:
        candidates.append((index, name, playback))
preferred = [item for item in candidates if item[2]]
preferred.sort(key=lambda item: 'usb audio' not in item[1].lower())
if not preferred:
    preferred = [item for item in candidates if 'porosvoc' in item[1].lower()]
if not preferred:
    preferred = candidates
if preferred:
    index, name, playback = preferred[0]
    print(('plughw:' if playback else 'hw:') + '%d,0' % index)
else:
    print('')"""
    result = _q5_remote_command("python3 -c " + shlex.quote(probe), timeout=15.0)
    if result.returncode:
        detail = (result.stderr or result.stdout).decode(errors="replace").strip()
        raise RuntimeError(f"unable to enumerate Q5 microphone devices: {detail}")
    try:
        selected = result.stdout.decode(errors="replace").strip()
        if not re.fullmatch(r"(?:hw|plughw):\d+,\d+", selected):
            raise ValueError(selected)
        return selected
    except ValueError as exc:
        detail = result.stdout.decode(errors="replace").strip()
        raise RuntimeError(f"no Q5 ALSA microphone capture device found: {detail}") from exc


def _q5_alsa_mic_command(device: str, sample_rate: int, channels: int) -> str:
    """Return a remote ALSA capture process which writes PCM16 to stdout."""
    return """import ctypes, sys
alsa = ctypes.CDLL('libasound.so.2')
alsa.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
alsa.snd_pcm_set_params.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint]
alsa.snd_pcm_readi.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
alsa.snd_pcm_prepare.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_close.argtypes = [ctypes.c_void_p]
pcm = ctypes.c_void_p()
rc = alsa.snd_pcm_open(ctypes.byref(pcm), b'%s', 1, 0)
if rc < 0: raise RuntimeError('snd_pcm_open(%s) failed: %%d' %% rc)
rc = alsa.snd_pcm_set_params(pcm, 2, 3, %d, %d, 1, 200000)
if rc < 0: raise RuntimeError('snd_pcm_set_params failed: %%d' %% rc)
frames = 1600
buffer = ctypes.create_string_buffer(frames * %d)
try:
  while True:
    count = alsa.snd_pcm_readi(pcm, buffer, frames)
    if count < 0:
      alsa.snd_pcm_prepare(pcm)
      continue
    sys.stdout.buffer.write(buffer.raw[:count * %d])
    sys.stdout.buffer.flush()
finally:
  alsa.snd_pcm_close(pcm)
""" % (device, device, channels, sample_rate, channels * 2, channels * 2)


def _q5_alsa_speaker_command(device: str, output_rate: int, output_channels: int) -> str:
    """Return the remote PCM16 stdin -> Q5 ALSA playback process.

    The Q5's playback card is visible as ``hw:2,0`` but its incomplete ALSA
    configuration means PortAudio does not enumerate it as an output device.
    It accepts S16_LE stereo at 44.1/48 kHz.  Keep the public contract at
    16 kHz mono and convert only at this hardware boundary.
    """
    return """import audioop, ctypes, sys
alsa = ctypes.CDLL('libasound.so.2')
alsa.snd_pcm_open.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
alsa.snd_pcm_set_params.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_int, ctypes.c_uint]
alsa.snd_pcm_writei.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
alsa.snd_pcm_prepare.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_drain.argtypes = [ctypes.c_void_p]
alsa.snd_pcm_close.argtypes = [ctypes.c_void_p]
pcm = ctypes.c_void_p()
rc = alsa.snd_pcm_open(ctypes.byref(pcm), b'%s', 0, 0)
if rc < 0: raise RuntimeError('snd_pcm_open(%s) failed: %%d' %% rc)
rc = alsa.snd_pcm_set_params(pcm, 2, 3, %d, %d, 1, 200000)
if rc < 0: raise RuntimeError('snd_pcm_set_params failed: %%d' %% rc)
state = None
pending = bytearray()
# Use the original verified live-speaker cadence. The remote DDS/SSH path can
# pause for over 250 ms; 600 ms blocks and a two-block prefill keep this USB
# ALSA device fed instead of repeatedly dropping into an inaudible underrun.
input_block_bytes = 9600
prefill_bytes = input_block_bytes * 2
try:
  while True:
    chunk = sys.stdin.buffer.read(input_block_bytes)
    if not chunk: break
    pending.extend(chunk)
    if len(pending) < prefill_bytes:
      continue
    raw = bytes(pending[:input_block_bytes])
    del pending[:input_block_bytes]
    mono, state = audioop.ratecv(raw, 2, 1, 16000, %d, state)
    stereo = audioop.tostereo(mono, 2, 1, 1)
    frames = len(stereo) // %d
    offset = 0
    while offset < frames:
      portion = stereo[offset * %d:]
      buf = ctypes.create_string_buffer(portion)
      written = alsa.snd_pcm_writei(pcm, buf, frames - offset)
      if written < 0:
        alsa.snd_pcm_prepare(pcm)
        continue
      offset += written
finally:
  alsa.snd_pcm_drain(pcm)
  alsa.snd_pcm_close(pcm)
""" % (device, device, output_channels, output_rate, output_rate, output_channels * 2,
       output_channels * 2)


class MicPlugin:
    """Q5 developer-container microphone as a 16 kHz PCM stream."""

    def __init__(self, plugin_config, namespace, executor, client):
        del executor
        self._client = client
        self._topic = f"/{namespace}/mic/audio"
        configured_device = plugin_config.get("device", "auto")
        configured_text = str(configured_device).lower()
        self._device = (
            None if configured_text == "auto"
            else f"hw:{configured_text},0" if configured_text.isdigit()
            else str(configured_device)
        )
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._process = None
        self._thread = None
        self._running = False
        self._frames_sent = 0
        self._lock = threading.RLock()
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 mic only supports the shared 16 kHz mono PCM contract")

    def get_tool(self):
        return {
            "name": "mic", "type": "sensor", "multiInstance": False,
            "description": "Q5 microphone, live PCM 16 kHz/16-bit/mono for ASR.",
            "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
        }

    def start(self):
        # The bundle start and a canvas sensor-start request may arrive nearly
        # simultaneously. ALSA allows only one capture owner for hw:1,0.
        with self._lock:
            if self._running:
                return
            try:
                device = self._device if self._device is not None else _find_remote_mic_device()
                command = _q5_alsa_mic_command(device, self._rate, self._channels)
                _stop_remote_mic_capture()
                self._process = subprocess.Popen(
                    _q5_ssh_args(_q5_mic_capture_shell(command)), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    bufsize=0)
                _raise_if_remote_process_exited(self._process, "microphone")
                self._running = True
                self._frames_sent = 0
                self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_mic_stream")
                self._thread.start()
                print(f"[MicPlugin] capture started from device {device} -> {self._topic}", flush=True)
            except Exception as exc:
                self.stop()
                print(f"[MicPlugin] capture unavailable: {exc}", flush=True)

    def _pump(self):
        # 100 ms frames are the same size emitted by perception TTS.
        while self._running and self._process and self._process.stdout:
            chunk = self._process.stdout.read(3200)
            if not chunk:
                break
            sender = getattr(self._client, "publish_audio", None)
            if callable(sender):
                sender(chunk)
                self._frames_sent += 1
                if self._frames_sent == 1:
                    print(f"[MicPlugin] first 100 ms frame published -> {self._topic}", flush=True)
                elif self._frames_sent % 100 == 0:
                    print(f"[MicPlugin] {self._frames_sent} PCM frames forwarded to bridge", flush=True)
        if self._running:
            print("[MicPlugin] remote capture stream ended", flush=True)

    def stop(self):
        with self._lock:
            self._running = False
            _stop_remote_mic_capture()
            if self._process is not None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                self._process = None

    def dispatch(self, action, args):
        del args
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        if action in ("start", "set_volume", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_out": [{"topic": self._topic, "format": "audio/pcm-16k"}],
                    "frames_sent": self._frames_sent}
        return None


class SpeakerPlugin:
    """Play any canvas-connected PCM AudioChunk stream on Q5 ALSA output."""

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace
        self._client = client
        self._topic = ""
        self._device = str(plugin_config.get("device", "hw:2,0"))
        self._rate = int(plugin_config.get("sample_rate_hz", 16000))
        self._channels = int(plugin_config.get("channels", 1))
        self._output_rate = int(plugin_config.get("output_sample_rate_hz", 44100))
        self._output_channels = int(plugin_config.get("output_channels", 2))
        self._volume = max(0, min(100, int(plugin_config.get("volume", 100))))
        self._input_gain = max(1.0, min(4.0, float(plugin_config.get("input_gain", 3.0))))
        self._system_volume = None
        self._node = Node("q5_speaker")
        executor.add_node(self._node)
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._process = None
        self._thread = None
        self._running = False
        self._frames_received = 0
        self._frames_written = 0
        if self._rate != 16000 or self._channels != 1:
            raise ValueError("Q5 speaker only supports the shared 16 kHz mono PCM contract")
        if self._output_rate not in (44100, 48000) or self._output_channels != 2:
            raise ValueError("Q5 speaker hardware requires 44.1/48 kHz stereo output")

    def get_tool(self):
        return {
            "name": "speaker", "type": "actuator", "multiInstance": False,
            "description": "Q5 speaker. Connect any audio/pcm-16k output (TTS, microphone, or other PCM source) to play it live. set_volume controls the Q5 system volume and the live PCM stream.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "set_volume", "stop", "info"]},
                "input_topic": {"type": "string", "description": "PCM 16 kHz AudioChunk topic from the canvas connection"},
                "volume": {"type": "integer", "title": "Speaker 音量", "minimum": 0, "maximum": 100,
                           "default": self._volume},
            }, "required": ["action"], "additionalProperties": False},
            "x-action-params": {
                "start": {"params": ["input_topic"], "description": "连接并开始实时播放 PCM。"},
                "set_volume": {"params": ["volume"], "description": "设置 Q5 系统音量和实时 speaker 音量，0 静音，100 最大。"},
                "stop": {"params": [], "description": "停止实时播放。"},
                "info": {"params": [], "description": "查看 speaker 状态。"},
            },
            # Leave the topic unresolved until canvas supplies input_topic.
            "topic_in": [{"format": "audio/pcm-16k"}],
        }

    def start(self, input_topic=None):
        del input_topic
        # The canvas calls dispatch(start, {input_topic}) when an audio output
        # is connected. Do not subscribe to a hard-coded TTS topic at boot.
        return

    def _start_for_topic(self, requested: str) -> None:
        if self._running and requested == self._topic:
            return
        self.stop()
        try:
            self._start_playback(requested)
            print(f"[SpeakerPlugin] playback subscribed <- {self._topic}", flush=True)
        except Exception as exc:
            self.stop()
            print(f"[SpeakerPlugin] playback unavailable: {exc}", flush=True)

    def _start_playback(self, requested: str) -> None:
        self._topic = requested
        self._set_system_volume(self._volume)
        # XOS owns the hardware mixer while its player is active.  This call
        # runs on the developer container, which is on the robot's ROS domain
        # and uses Cyclone DDS; inheriting the bridge's Domain 42/Fast DDS
        # environment made the call target no valid context.
        try:
            stopped = _q5_remote_command(
                "source /opt/ros/humble/setup.bash; "
                "export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; "
                "timeout 2 ros2 service call /audio_player/stop_play std_srvs/srv/Trigger '{}'",
                timeout=4.0)
            if stopped.returncode:
                detail = (stopped.stderr or stopped.stdout).decode(errors="replace").strip()
                print(f"[SpeakerPlugin] vendor player was not stopped: {detail}", flush=True)
        except Exception as exc:
            print(f"[SpeakerPlugin] vendor player stop timed out; continuing with ALSA: {exc}", flush=True)
        _stop_remote_speaker_playback()
        command = _q5_alsa_speaker_command(
            self._device, self._output_rate, self._output_channels)
        self._process = subprocess.Popen(
            _q5_ssh_args(_q5_speaker_playback_shell(command)), stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0)
        try:
            _raise_if_remote_process_exited(self._process, "speaker")
        except RuntimeError as exc:
            holders = _q5_remote_playback_holders()
            raise RuntimeError(f"{exc}; playback device holders: {holders}") from exc
        configure = getattr(self._client, "configure_speaker", None)
        if callable(configure):
            configure(self._topic)
        self._running = True
        self._frames_received = 0
        self._frames_written = 0
        self._thread = threading.Thread(target=self._pump, daemon=True, name="q5_speaker_stream")
        self._thread.start()

    def _pump(self):
        while self._running and self._process and self._process.stdin:
            getter = getattr(self._client, "pop_speaker_chunk", None)
            chunk = getter() if callable(getter) else None
            if chunk is None:
                time.sleep(0.005)
                continue
            self._frames_received += 1
            if self._frames_received == 1:
                print(f"[SpeakerPlugin] first PCM frame received from {self._topic}", flush=True)
            elif self._frames_received % 100 == 0:
                print(f"[SpeakerPlugin] {self._frames_received} PCM frames received from {self._topic}", flush=True)
            try:
                # The input stream is commonly quieter than Q5 stored audio.
                # Keep the user-facing 0-100 control linear, then apply a
                # bounded source-gain calibration. audioop clips PCM safely.
                gain = (self._volume / 100.0) * self._input_gain
                if gain != 1.0:
                    chunk = audioop.mul(chunk, 2, gain)
                self._process.stdin.write(chunk)
                self._process.stdin.flush()
                self._frames_written += 1
                if self._frames_written == 1:
                    print("[SpeakerPlugin] first PCM frame written to ALSA", flush=True)
            except (BrokenPipeError, OSError):
                detail = ""
                if self._process and self._process.stderr:
                    try:
                        detail = self._process.stderr.read().decode(errors="replace").strip()
                    except Exception:
                        pass
                print(f"[SpeakerPlugin] remote playback stream ended: {detail}", flush=True)
                self._running = False
                break

    def _set_system_volume(self, volume: int) -> None:
        """Set XOS's global audio route volume without starting its player."""
        if not self._srv_volume.wait_for_service(timeout_sec=2.0):
            self._system_volume = {"state": "unavailable", "volume": volume}
            print("[SpeakerPlugin] XOS volume service is unavailable", flush=True)
            return
        request = SetVolume.Request()
        request.volume = volume
        response = _wait_for_future(self._srv_volume.call_async(request), 2.0)
        if response is None:
            self._system_volume = {"state": "timeout", "volume": volume}
            print("[SpeakerPlugin] XOS volume request timed out", flush=True)
            return
        self._system_volume = {
            "state": "ok" if response.success else "error",
            "volume": volume,
            "message": response.message,
        }
        if not response.success:
            print(f"[SpeakerPlugin] XOS volume was not set: {response.message}", flush=True)

    def stop(self):
        self._running = False
        if self._process is not None:
            try:
                if self._process.stdin:
                    self._process.stdin.close()
                self._process.wait(timeout=3)
            except (subprocess.TimeoutExpired, OSError):
                self._process.terminate()
            self._process = None
        _stop_remote_speaker_playback()

    def dispatch(self, action, args):
        if action == "start":
            requested = str(args.get("input_topic") or "")
            if not requested:
                return {"ok": False, "code": "INPUT_TOPIC_REQUIRED",
                        "message": "Connect an audio/pcm-16k output to speaker before starting playback"}
            self._start_for_topic(requested)
        elif action == "set_volume":
            value = args.get("volume", self._volume)
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100:
                return {"ok": False, "code": "INVALID_VOLUME", "message": "volume must be an integer from 0 to 100"}
            self._volume = value
            self._set_system_volume(value)
        elif action == "stop":
            self.stop()
        if action in ("start", "set_volume", "stop", "info"):
            return {"state": "running" if self._running else "idle",
                    "topic_in": ([{"topic": self._topic, "format": "audio/pcm-16k"}]
                                 if self._topic else [{"format": "audio/pcm-16k"}]),
                    "playback": {"device": self._device, "sample_rate_hz": self._output_rate,
                                 "channels": self._output_channels, "volume": self._volume,
                                 "input_gain": self._input_gain,
                                 "system_volume": self._system_volume},
                    "frames_received": self._frames_received,
                    "frames_written": self._frames_written}
        return None


class _Q5MediaPlugin:
    """Base for read-only Domain-211 media subscriptions.

    The developer container owns the D455 and SLAM services. These cards only
    subscribe to their DDS output and send bounded, already-processed payloads
    to the existing Domain-42 bridge worker.
    """

    def __init__(self, plugin_config, namespace, executor, client):
        self._ns = namespace
        self._client = client
        self._executor = executor
        self._running = False
        self._last_sent = 0.0
        self._max_hz = max(0.1, float(plugin_config.get("max_hz", 10.0)))
        self._subscription = None
        self._node = Node(self._node_name)
        executor.add_node(self._node)

    def _send_media(self, payload):
        sender = getattr(self._client, "publish_media", None)
        if callable(sender):
            sender(payload)

    def stop(self):
        self._running = False

    def dispatch(self, action, args):
        del args
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start":
            self.start()
        if action in ("start", "info"):
            result = {
                "state": "running" if self._running else "idle",
                "source_topic": self._source_topic,
                "topic_out": [{"topic": self._topic, "format": self._format}],
            }
            if hasattr(self, "_frames_received"):
                result["diagnostics"] = {
                    "frames_received": self._frames_received,
                    "frames_sent": self._frames_sent,
                }
            return result
        return None


class CameraRgbPlugin(_Q5MediaPlugin):
    """D455 RGB to JPEG, throttled before crossing into Agent Core's DDS domain."""

    _node_name = "q5_camera_rgb"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/color/image_raw"))
        self._topic = f"/{namespace}/camera/rgb"
        self._jpeg_quality = max(20, min(95, int(plugin_config.get("jpeg_quality", 70))))
        self._latest = None
        self._frames_received = 0
        self._frames_sent = 0
        self._lock = threading.Lock()
        self._encoder = None
        self._remote_start = dict(plugin_config.get("remote_start") or {})
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_rgb", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 RGB camera. The developer-container RealSense driver must already be running.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from PIL import Image as PilImage
        from sensor_msgs.msg import Image

        self._start_remote_realsense_if_configured()
        self._pil_image, self._np = PilImage, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_image, _LATEST_QOS)
        self._encoder = threading.Thread(target=self._encode_loop, daemon=True, name="q5_rgb_encoder")
        self._encoder.start()
        print(f"[CameraRgbPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _start_remote_realsense_if_configured(self):
        """Optionally start the D455 on its owning developer container via SSH.

        This is intentionally opt-in: XOS can also own the camera, and the
        launch never restarts a live driver. Q5 documents this developer
        account as part of its external-development workflow.
        """
        if not self._remote_start.get("enabled", False):
            return
        host = str(self._remote_start.get("host", "192.168.8.100"))
        user = str(self._remote_start.get("user", "developer"))
        password = str(self._remote_start.get("password", ""))
        try:
            port = int(self._remote_start.get("port", 2222))
        except (TypeError, ValueError):
            raise ValueError("camera_rgb.remote_start.port must be an integer")
        profiles = (
            str(self._remote_start.get("depth_profile", "848x480x30")),
            str(self._remote_start.get("color_profile", "848x480x30")),
        )
        if (not password or not re.fullmatch(r"[A-Za-z0-9.-]+", host) or
                not re.fullmatch(r"[A-Za-z0-9_-]+", user) or
                any(not re.fullmatch(r"[0-9]+x[0-9]+x[0-9]+", value) for value in profiles)):
            raise ValueError("invalid camera_rgb.remote_start configuration")
        # `nohup ... &` alone is not evidence that the remote launch survived
        # the SSH session. Keep a PID/log file and synchronously verify the
        # process, otherwise the UI only sees a permanent black frame.
        remote = f'''#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=211 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if ! pgrep -f realsense2_camera_node >/dev/null; then
  nohup ros2 launch realsense2_camera rs_align_depth_launch.py \\
    depth_module.depth_profile:={profiles[0]} \\
    rgb_camera.color_profile:={profiles[1]} \\
    </dev/null >/tmp/q5-realsense.log 2>&1 &
  echo $! >/tmp/q5-realsense.pid
fi
sleep 5
if ! pgrep -f realsense2_camera_node >/dev/null; then
  echo 'RealSense process did not remain running'
  test -f /tmp/q5-realsense.log && tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
if ! ros2 topic info /camera/camera/color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo 'RealSense RGB publisher is unavailable'
  tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
if ! ros2 topic info /camera/camera/aligned_depth_to_color/image_raw 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  echo 'RealSense aligned-depth publisher is unavailable'
  tail -100 /tmp/q5-realsense.log || true
  exit 1
fi
echo 'RealSense process is running'
cat /tmp/q5-realsense.pid 2>/dev/null || true
'''
        command = ["sshpass", "-p", password, "ssh", "-p", str(port),
                   "-o", "PreferredAuthentications=password", "-o", "PubkeyAuthentication=no",
                   "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
                   "-o", "UserKnownHostsFile=/dev/null",
                   f"{user}@{host}", "bash", "-s"]
        result = subprocess.run(command, input=remote, capture_output=True, text=True, timeout=20)
        if result.returncode:
            detail = (result.stderr or result.stdout).strip()
            raise RuntimeError(f"remote RealSense launch failed: {detail}")
        print(f"[CameraRgbPlugin] remote D455 verified on {user}@{host}:{port}: "
              f"{result.stdout.strip()}", flush=True)

    def _on_image(self, msg):
        if self._running:
            with self._lock:
                self._latest = msg
                self._frames_received += 1

    def _encode_loop(self):
        while self._running:
            with self._lock:
                msg, self._latest = self._latest, None
            if msg is None or time.monotonic() - self._last_sent < 1.0 / self._max_hz:
                time.sleep(0.005)
                continue
            try:
                channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
                if channels is None or msg.step < msg.width * channels:
                    continue
                raw = self._np.frombuffer(msg.data, dtype=self._np.uint8)
                image = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * channels]
                image = image.reshape(msg.height, msg.width, channels)
                if msg.encoding == "bgr8":
                    image = image[:, :, ::-1]
                elif msg.encoding == "rgba8":
                    image = image[:, :, :3]
                elif msg.encoding == "bgra8":
                    image = image[:, :, [2, 1, 0]]
                image = self._np.ascontiguousarray(image)
                encoded = io.BytesIO()
                self._pil_image.fromarray(image, "RGB").save(
                    encoded, format="JPEG", quality=self._jpeg_quality)
                self._send_media({"kind": "rgb", "data": encoded.getvalue(),
                                  "width": int(msg.width), "height": int(msg.height),
                                  "encoding": msg.encoding, "timestamp_ms": int(time.time() * 1000)})
                self._frames_sent += 1
                self._last_sent = time.monotonic()
            except Exception as exc:
                self._node.get_logger().warn(f"RGB encode failed: {exc}")


class CameraDepthPlugin(_Q5MediaPlugin):
    """D455 aligned depth rendered as a dimension-preserving grayscale JPEG."""

    _node_name = "q5_camera_depth"
    _format = "image/jpeg"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get("source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        # Preserve the canonical raw depth topic's ROS message type. The card
        # exposes a separate JPEG preview topic so Agent Core never sees two
        # incompatible message types on one DDS path.
        self._topic = f"/{namespace}/camera/depth_preview"
        self._near_depth_mm = max(1.0, float(plugin_config.get("near_depth_m", 0.25)) * 1000.0)
        self._far_depth_mm = max(self._near_depth_mm + 1.0,
                                 float(plugin_config.get("far_depth_m", 4.0)) * 1000.0)
        self._depth_gamma = max(0.25, min(2.0, float(plugin_config.get("gamma", 0.70))))
        self._frames_received = 0
        self._frames_sent = 0
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_depth", "type": "sensor", "multiInstance": False,
            "description": "Q5 D455 aligned depth preview. Fixed-scale pseudo-color distance: near is yellow, far is violet; invalid depth is black.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
            "diagnostics": {"frames_received": self._frames_received, "frames_sent": self._frames_sent},
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from PIL import Image as PilImage
        from sensor_msgs.msg import Image
        self._pil_image, self._np = PilImage, np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        print(f"[CameraDepthPlugin] subscribed {self._source_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_depth(self, msg):
        if not self._running:
            return
        self._frames_received += 1
        if (msg.encoding not in ("16UC1", "mono16") or
                time.monotonic() - self._last_sent < 1.0 / self._max_hz):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        try:
            dtype = self._np.dtype(">u2" if msg.is_bigendian else "<u2")
            depth = self._np.frombuffer(msg.data[:needed], dtype=dtype).reshape(msg.height, msg.step // 2)
            depth = depth[:, :msg.width].astype(self._np.float32)
            # D455 Z16 is millimetres. Use a fixed range to keep distance
            # colors stable between frames, then apply a modest gamma so the
            # near/mid-field geometry remains legible indoors.
            normalized = self._np.clip(
                (depth - self._near_depth_mm) / (self._far_depth_mm - self._near_depth_mm), 0.0, 1.0)
            normalized = normalized ** self._depth_gamma
            stops = self._np.array([
                # Reversed Viridis: close is warm/high-visibility and far
                # recedes through teal into violet without rainbow banding.
                (253, 231, 37), (94, 201, 98), (33, 145, 140),
                (59, 82, 139), (68, 1, 84),
            ], dtype=self._np.float32)
            scaled = normalized * (len(stops) - 1)
            lower = self._np.floor(scaled).astype(self._np.intp)
            upper = self._np.minimum(lower + 1, len(stops) - 1)
            fraction = (scaled - lower)[..., None]
            color = ((1.0 - fraction) * stops[lower] + fraction * stops[upper]).astype(self._np.uint8)
            color[depth <= 0] = 0
            encoded = io.BytesIO()
            self._pil_image.fromarray(color, "RGB").save(encoded, format="JPEG", quality=75)
            self._send_media({"kind": "depth_jpeg", "data": encoded.getvalue()})
        except Exception as exc:
            self._node.get_logger().warn(f"Depth preview encode failed: {exc}")
            return
        self._frames_sent += 1
        self._last_sent = time.monotonic()


class CameraPointCloudPlugin(_Q5MediaPlugin):
    """Reconstruct a bounded XYZ cloud from D455 aligned depth and intrinsics."""

    _node_name = "q5_camera_pointcloud"
    _format = "sensor/pointcloud"

    def __init__(self, plugin_config, namespace, executor, client):
        self._source_topic = str(plugin_config.get(
            "source_topic", "/camera/camera/aligned_depth_to_color/image_raw"))
        self._info_topic = str(plugin_config.get(
            "camera_info_topic", "/camera/camera/color/camera_info"))
        self._topic = f"/{namespace}/camera/pointcloud"
        self._max_points = max(100, min(50000, int(plugin_config.get("max_points", 10000))))
        self._min_depth_m = max(0.0, float(plugin_config.get("min_depth_m", 0.25)))
        self._max_depth_m = max(self._min_depth_m, float(plugin_config.get("max_depth_m", 5.0)))
        self._camera_mount_pitch_rad = float(plugin_config.get("camera_mount_pitch_rad", 0.14655))
        self._floor_offset_m = max(0.0, float(plugin_config.get("floor_offset_m", 1.15)))
        self._intrinsics = None
        self._frames_received = 0
        self._frames_sent = 0
        self._info_subscription = None
        super().__init__(plugin_config, namespace, executor, client)

    def get_tool(self):
        return {
            "name": "camera_pointcloud", "type": "sensor", "multiInstance": False,
            "description": f"Q5 D455 aligned-depth XYZ point cloud, rendered as a forward-facing camera view. Limited to {self._max_points:,} points/frame; this is not 360-degree lidar.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["start", "stop", "info"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": [{"topic": self._topic, "format": self._format}],
        }

    def start(self):
        if self._running:
            return
        import numpy as np
        from sensor_msgs.msg import CameraInfo, Image
        self._np = np
        self._running = True
        if self._subscription is None:
            self._subscription = self._node.create_subscription(
                Image, self._source_topic, self._on_depth, _LATEST_QOS)
        if self._info_subscription is None:
            self._info_subscription = self._node.create_subscription(
                CameraInfo, self._info_topic, self._on_info, _RELIABLE_QOS)
        print(f"[CameraPointCloudPlugin] subscribed {self._source_topic} + {self._info_topic} -> {self._topic} <= {self._max_hz:g}Hz", flush=True)

    def _on_info(self, msg):
        fx, fy, cx, cy = float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])
        if fx > 0.0 and fy > 0.0:
            self._intrinsics = (fx, fy, cx, cy)

    def _on_depth(self, msg):
        if not self._running:
            return
        self._frames_received += 1
        intrinsics = self._intrinsics
        if (intrinsics is None or msg.encoding not in ("16UC1", "mono16") or
                time.monotonic() - self._last_sent < 1.0 / self._max_hz):
            return
        needed = int(msg.height) * int(msg.step)
        if msg.width <= 0 or msg.height <= 0 or msg.step < msg.width * 2 or len(msg.data) < needed:
            return
        try:
            dtype = self._np.dtype(">u2" if msg.is_bigendian else "<u2")
            depth = self._np.frombuffer(msg.data[:needed], dtype=dtype).reshape(msg.height, msg.step // 2)
            depth = depth[:, :msg.width].astype(self._np.float32) * 0.001
            stride = max(1, int(((msg.width * msg.height) / self._max_points) ** 0.5 + 0.999))
            z = depth[::stride, ::stride]
            valid = (z >= self._min_depth_m) & (z <= self._max_depth_m)
            if not valid.any():
                return
            rows, cols = self._np.indices(z.shape, dtype=self._np.float32)
            rows *= stride
            cols *= stride
            fx, fy, cx, cy = intrinsics
            camera_x = (cols - cx) * z / fx  # right
            camera_y = (rows - cy) * z / fy  # down
            # The depth image is in the D455 optical frame.  Render it in a
            # Q5 body-level frame instead: account for the fixed D455 mount
            # angle and the current neck pitch, then put the camera origin at
            # its configured height over the floor.
            joints = self._client.snapshot().get("joints") or {}
            neck_pitch = float(joints.get("neck_pitch_joint", 0.0))
            pitch = self._camera_mount_pitch_rad + neck_pitch
            cosine, sine = self._np.cos(pitch), self._np.sin(pitch)
            camera_up = -camera_y
            body_up = cosine * camera_up - sine * z + self._floor_offset_m
            body_forward = sine * camera_up + cosine * z
            # Agent Core's point-cloud renderer maps packet (x, y, z) to
            # display (y, -z, -x). The renderer's horizontal convention is
            # opposite the D455 optical axis, so mirror camera-right here.
            points = self._np.stack((-body_forward, -camera_x, -body_up), axis=-1)[valid]
            if not len(points):
                return
            points = self._np.ascontiguousarray(points.astype("<f4", copy=False))
            payload = struct.pack("<II", 12, len(points)) + points.tobytes()
            self._send_media({"kind": "pointcloud", "data": payload})
        except Exception as exc:
            self._node.get_logger().warn(f"Camera point-cloud encode failed: {exc}")
            return
        self._frames_sent += 1
        self._last_sent = time.monotonic()


def _wait_for_future(future, timeout_sec: float):
    """Wait for work completed by main.py's shared executor thread."""
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


class AudioPlugin:
    """Vendor audio playback via /audio_player/play and paired services."""

    _upload_directory = "/xos/xos/data/audio/replay_wav"

    def __init__(self, plugin_config, namespace, executor, client):
        del namespace, client
        self._node = Node("q5_audio")
        executor.add_node(self._node)
        self._action_client = ActionClient(self._node, AudioPlay, "/audio_player/play")
        self._srv_volume = self._node.create_client(SetVolume, "/audio_player/set_volume")
        self._srv_stop = self._node.create_client(Trigger, "/audio_player/stop_play")
        self._srv_is_play = self._node.create_client(Trigger, "/audio_player/is_play")
        self._device = plugin_config.get("device", "plughw:2,0")
        self._library_dir = os.path.realpath(str(plugin_config.get(
            "library_dir", "/opt/phanthy-motus/data/audios")))
        self._upload_max_bytes = max(1, int(plugin_config.get("upload_max_bytes", 20 * 1024 * 1024)))

    def get_tool(self):
        play_actions = {
            "play_by_id": {"mode": 0, "title": "按内置音频 ID 播放", "param": "id"},
            "play_by_path": {"mode": 1, "title": "按设备路径播放", "param": "path"},
            "play_by_item": {"mode": 2, "title": "按 item JSON 播放", "param": "item"},
            "play_by_file_name": {"mode": 3, "title": "按文件名播放", "param": "file_name"},
        }
        return {
            "name": "audio", "type": "actuator", "multiInstance": False,
            "description": "Q5 vendor stored-audio playback, upload-to-path, volume, stop, and status. Live PCM speaker volume is controlled on the speaker card.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": [
                    *play_actions, "list_library", "list_robot_audio_files", "upload_from_library", "upload_base64", "set_volume", "stop_audio", "is_play", "stop"],
                    "oneOf": [
                        *[{"const": action, "title": detail["title"]}
                          for action, detail in play_actions.items()],
                        {"const": "list_library", "title": "查看挂载音频库"},
                        {"const": "list_robot_audio_files", "title": "查看机器人音频文件"},
                        {"const": "upload_from_library", "title": "从挂载音频库上传"},
                        {"const": "upload_base64", "title": "上传音频到机器人"},
                        {"const": "set_volume", "title": "设置音量"},
                        {"const": "stop_audio", "title": "停止播放"},
                        {"const": "is_play", "title": "查询播放状态"},
                        {"const": "stop", "title": "停止音频卡"},
                    ]},
                "id": {"type": "integer", "title": "内置音频 ID"},
                "path": {"type": "string", "title": "设备音频路径", "minLength": 1},
                "item": {"type": "string", "title": "item JSON", "minLength": 1},
                "file_name": {"type": "string", "title": "音频文件名", "minLength": 1},
                "content_base64": {"type": "string", "title": "WAV/MP3 文件内容 (Base64)", "minLength": 1},
                "force_play": {"type": "boolean", "title": "强制打断当前播放"},
                "timeout": {"type": "integer", "title": "超时 (s)", "minimum": 0},
                "channel": {"type": "string", "title": "播放通道",
                            "enum": ["default", "channel1", "channel2", "channel3"]},
                "version": {"type": "string", "title": "音频版本", "enum": ["v1", "v2"]},
                "volume": {"type": "integer", "title": "音量", "minimum": 0, "maximum": 100},
            }, "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    **{action: {"params": [detail["param"], "force_play", "timeout", "channel", "version"],
                                  "description": f"模式 {detail['mode']}；只接受 {detail['param']} 作为播放来源。"}
                       for action, detail in play_actions.items()},
                    "list_library": {"params": [], "description": "列出 /opt/phanthy-motus/data/audios 中可上传的 WAV/MP3 文件。"},
                    "list_robot_audio_files": {"params": [], "description": "列出 Q5 XOS replay_wav 目录内的文件和可供 play_by_path 使用的完整路径；厂商未提供 ID 映射查询。"},
                    "upload_from_library": {"params": ["file_name"],
                                            "description": "从挂载音频库按文件名上传 WAV/MP3；返回的 path 可传给 play_by_path。"},
                    "upload_base64": {"params": ["file_name", "content_base64"],
                                      "description": "上传 WAV/MP3 到机器人 replay_wav 目录；返回的 path 可传给 play_by_path。不会创建 XOS 音频库 ID。"},
                    "set_volume": {"params": ["volume"], "description": "设置厂商 AudioPlay 音量 0 到 100；不控制 live speaker。"},
                    "stop_audio": {"params": [], "description": "停止当前厂商音频播放。"},
                    "is_play": {"params": [], "description": "查询当前是否正在播放。"},
                    "stop": {"params": [], "description": "停止音频卡并停止当前播放。"},
                }},
        }

    def start(self):
        pass

    def stop(self):
        self._stop_audio()

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready", "action_server": "/audio_player/play", "device": self._device}
        play_modes = {"play_by_id": 0, "play_by_path": 1,
                      "play_by_item": 2, "play_by_file_name": 3}
        if action in play_modes:
            return self._play(args, play_modes[action])
        if action == "list_library":
            return self._list_library()
        if action == "list_robot_audio_files":
            return self._list_robot_audio_files()
        if action == "upload_from_library":
            return self._upload_from_library(args)
        if action == "upload_base64":
            return self._upload_base64(args)
        if action == "set_volume":
            return self._set_volume(args.get("volume", 50))
        if action == "stop_audio":
            return self._stop_audio()
        if action == "is_play":
            return self._is_playing()
        if action == "stop":
            self._stop_audio()
            return {"state": "idle"}
        return None

    @staticmethod
    def _valid_upload_name(file_name) -> bool:
        return (isinstance(file_name, str) and file_name == os.path.basename(file_name) and
                bool(re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_. -]*\.(?:wav|mp3)", file_name, re.IGNORECASE)))

    def _list_library(self):
        try:
            entries = []
            for entry in sorted(os.scandir(self._library_dir), key=lambda item: item.name.lower()):
                if entry.is_file() and self._valid_upload_name(entry.name):
                    entries.append({"file_name": entry.name, "bytes": entry.stat().st_size})
            return {"state": "ok", "library_dir": self._library_dir, "files": entries[:100],
                    "truncated": len(entries) > 100}
        except FileNotFoundError:
            return {"state": "ok", "library_dir": self._library_dir, "files": [],
                    "message": "audio library directory does not exist yet"}
        except OSError as exc:
            return {"state": "error", "message": f"cannot read audio library: {exc}"}

    def _upload_from_library(self, args):
        file_name = args.get("file_name")
        if not self._valid_upload_name(file_name):
            return {"state": "error", "message": "file_name must be a simple .wav or .mp3 filename"}
        source_path = os.path.realpath(os.path.join(self._library_dir, file_name))
        if os.path.commonpath((self._library_dir, source_path)) != self._library_dir:
            return {"state": "error", "message": "file_name is outside the configured audio library"}
        try:
            size = os.path.getsize(source_path)
            if size <= 0:
                return {"state": "error", "message": "audio file is empty"}
            if size > self._upload_max_bytes:
                return {"state": "error", "message": f"audio file exceeds {self._upload_max_bytes} byte upload limit"}
            with open(source_path, "rb") as source:
                payload = source.read()
        except FileNotFoundError:
            return {"state": "error", "message": f"audio file not found in library: {file_name}"}
        except OSError as exc:
            return {"state": "error", "message": f"cannot read audio file: {exc}"}
        return self._upload_payload(file_name, payload)

    def _list_robot_audio_files(self):
        # The manual documents this replay directory, but exposes no service
        # for enumerating the XOS audio-library database or its numeric IDs.
        command = (
            f"if [ -d {shlex.quote(self._upload_directory)} ]; then "
            f"find {shlex.quote(self._upload_directory)} -maxdepth 1 -type f "
            "\\( -iname '*.wav' -o -iname '*.mp3' \\) -printf '%f\\t%s\\n' | sort; "
            "fi"
        )
        try:
            result = _q5_remote_command(command, timeout=15.0)
        except Exception as exc:
            return {"state": "error", "message": f"cannot list Q5 audio files: {exc}"}
        if result.returncode:
            detail = (result.stderr or result.stdout).decode(errors="replace").strip()
            return {"state": "error", "message": f"cannot list Q5 audio files: {detail or 'remote command failed'}"}
        files = []
        for line in result.stdout.decode(errors="replace").splitlines():
            name, separator, size = line.partition("\t")
            if self._valid_upload_name(name):
                files.append({"file_name": name, "path": f"{self._upload_directory}/{name}",
                              "bytes": int(size) if separator and size.isdigit() else None})
        return {
            "state": "ok", "directory": self._upload_directory, "files": files,
            "play_action": "play_by_path", "id_mapping_available": False,
            "note": "The Q5 manual exposes no API to list XOS audio-library IDs; use each returned path with play_by_path.",
        }

    def _upload_base64(self, args):
        file_name = args.get("file_name")
        encoded = args.get("content_base64")
        if not self._valid_upload_name(file_name):
            return {"state": "error", "message": "file_name must be a simple .wav or .mp3 filename"}
        if not isinstance(encoded, str) or not encoded:
            return {"state": "error", "message": "content_base64 is required"}
        try:
            payload = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError):
            return {"state": "error", "message": "content_base64 is not valid Base64"}
        return self._upload_payload(file_name, payload)

    def _upload_payload(self, file_name: str, payload: bytes):
        if not payload:
            return {"state": "error", "message": "audio file is empty"}
        if len(payload) > self._upload_max_bytes:
            return {"state": "error", "message": f"audio file exceeds {self._upload_max_bytes} byte upload limit"}
        remote_path = f"{self._upload_directory}/{file_name}"
        temporary_path = f"{remote_path}.part"
        command = (
            f"mkdir -p {shlex.quote(self._upload_directory)} && "
            f"base64 -d > {shlex.quote(temporary_path)} && "
            f"mv {shlex.quote(temporary_path)} {shlex.quote(remote_path)}"
        )
        try:
            result = _q5_remote_command(command, timeout=45.0, stdin=base64.b64encode(payload))
        except Exception as exc:
            return {"state": "error", "message": f"audio upload failed: {exc}"}
        if result.returncode:
            detail = (result.stderr or result.stdout).decode(errors="replace").strip()
            return {"state": "error", "message": f"audio upload failed: {detail or 'remote command failed'}"}
        return {
            "state": "ok", "file_name": file_name, "path": remote_path,
            "bytes_uploaded": len(payload), "next_action": "play_by_path",
            "note": "File is copied to the replay path; this does not create an XOS audio-library id.",
        }

    def _play(self, args, mode: int):
        source_fields = {0: "id", 1: "path", 2: "item", 3: "file_name"}
        source_field = source_fields[mode]
        if source_field not in args:
            return {"state": "error", "message": f"mode {mode} requires {source_field}"}
        unrelated = sorted(field for field in source_fields.values()
                           if field != source_field and field in args)
        if unrelated:
            return {"state": "error", "message": (
                f"mode {mode} only accepts {source_field}; do not provide {', '.join(unrelated)}")}
        value = args.get(source_field)
        if mode == 0:
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                return {"state": "error", "message": "id must be an integer greater than 0"}
        elif not isinstance(value, str) or not value.strip():
            return {"state": "error", "message": f"{source_field} must be a non-empty string"}
        if not self._action_client.wait_for_server(timeout_sec=3.0):
            return {"state": "error", "message": "/audio_player/play is unavailable"}
        goal = AudioPlay.Goal()
        goal.mode = mode
        goal.force_play = bool(args.get("force_play", False))
        goal.id = int(args.get("id", 0))
        goal.path = str(args.get("path", ""))
        goal.item = str(args.get("item", ""))
        goal.file_name = str(args.get("file_name", ""))
        goal.channel = str(args.get("channel", "default"))
        goal.timeout = int(args.get("timeout", 0))
        goal.version = str(args.get("version", "v1"))
        goal_handle = _wait_for_future(self._action_client.send_goal_async(goal), 5.0)
        if goal_handle is None:
            return {"state": "error", "message": "audio goal timed out"}
        if not goal_handle.accepted:
            return {"state": "error", "message": "audio goal rejected"}
        response = _wait_for_future(goal_handle.get_result_async(), max(10.0, goal.timeout + 2.0))
        if response is None:
            return {"state": "error", "message": "audio result timed out"}
        return {"state": "ok" if response.result.success else "error", "message": response.result.message}

    def _set_volume(self, value):
        if not self._srv_volume.service_is_ready():
            return {"state": "error", "message": "/audio_player/set_volume is unavailable"}
        req = SetVolume.Request()
        req.volume = max(0, min(100, int(value)))
        response = _wait_for_future(self._srv_volume.call_async(req), 2.0)
        if response is None:
            return {"state": "error", "message": "set-volume request timed out"}
        return {"state": "ok" if response.success else "error", "volume": req.volume, "message": response.message}

    def _stop_audio(self):
        if not self._srv_stop.service_is_ready():
            return {"state": "error", "message": "/audio_player/stop_play is unavailable"}
        response = _wait_for_future(self._srv_stop.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "stop-audio request timed out"}
        return {"state": "ok" if response.success else "error", "message": response.message}

    def _is_playing(self):
        if not self._srv_is_play.service_is_ready():
            return {"state": "error", "message": "/audio_player/is_play is unavailable"}
        response = _wait_for_future(self._srv_is_play.call_async(Trigger.Request()), 2.0)
        if response is None:
            return {"state": "error", "message": "is-play request timed out"}
        return {"state": "ok", "is_playing": response.success, "message": response.message}
