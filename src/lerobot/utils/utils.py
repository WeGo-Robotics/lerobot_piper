#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import logging
import math
import os
import os.path as osp
import platform
import re
import shutil
import select
import subprocess
import sys
import time
import wave
from copy import copy, deepcopy
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean

import numpy as np
import torch


def none_or_int(value):
    if value == "None":
        return None
    return int(value)


def inside_slurm():
    """Check whether the python process was launched through slurm"""
    # TODO(rcadene): return False for interactive mode `--pty bash`
    return "SLURM_JOB_ID" in os.environ


def auto_select_torch_device() -> torch.device:
    """Tries to select automatically a torch device."""
    if torch.cuda.is_available():
        logging.info("Cuda backend detected, using cuda.")
        return torch.device("cuda")
    elif torch.backends.mps.is_available():
        logging.info("Metal backend detected, using mps.")
        return torch.device("mps")
    else:
        logging.warning("No accelerated backend detected. Using default cpu, this will be slow.")
        return torch.device("cpu")


# TODO(Steven): Remove log. log shouldn't be an argument, this should be handled by the logger level
def get_safe_torch_device(try_device: str, log: bool = False) -> torch.device:
    """Given a string, return a torch.device with checks on whether the device is available."""
    try_device = str(try_device)
    match try_device:
        case "cuda":
            assert torch.cuda.is_available()
            device = torch.device("cuda")
        case "mps":
            assert torch.backends.mps.is_available()
            device = torch.device("mps")
        case "cpu":
            device = torch.device("cpu")
            if log:
                logging.warning("Using CPU, this will be slow.")
        case _:
            device = torch.device(try_device)
            if log:
                logging.warning(f"Using custom {try_device} device.")

    return device


def get_safe_dtype(dtype: torch.dtype, device: str | torch.device):
    """
    mps is currently not compatible with float64
    """
    if isinstance(device, torch.device):
        device = device.type
    if device == "mps" and dtype == torch.float64:
        return torch.float32
    else:
        return dtype


def is_torch_device_available(try_device: str) -> bool:
    try_device = str(try_device)  # Ensure try_device is a string
    if try_device == "cuda":
        return torch.cuda.is_available()
    elif try_device == "mps":
        return torch.backends.mps.is_available()
    elif try_device == "cpu":
        return True
    else:
        raise ValueError(f"Unknown device {try_device}. Supported devices are: cuda, mps or cpu.")


def is_amp_available(device: str):
    if device in ["cuda", "cpu"]:
        return True
    elif device == "mps":
        return False
    else:
        raise ValueError(f"Unknown device '{device}.")


def init_logging(
    log_file: Path | None = None,
    display_pid: bool = False,
    console_level: str = "INFO",
    file_level: str = "DEBUG",
):
    def custom_format(record: logging.LogRecord) -> str:
        dt = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        fnameline = f"{record.pathname}:{record.lineno}"

        # NOTE: Display PID is useful for multi-process logging.
        if display_pid:
            pid_str = f"[PID: {os.getpid()}]"
            message = f"{record.levelname} {pid_str} {dt} {fnameline[-15:]:>15} {record.getMessage()}"
        else:
            message = f"{record.levelname} {dt} {fnameline[-15:]:>15} {record.getMessage()}"
        return message

    formatter = logging.Formatter()
    formatter.format = custom_format

    logger = logging.getLogger()
    logger.setLevel(logging.NOTSET)  # Set the logger to the lowest level to capture all messages

    # Remove unused default handlers
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)

    # Write logs to console
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    console_handler.setLevel(console_level.upper())
    logger.addHandler(console_handler)

    # Additionally write logs to file
    if log_file is not None:
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        file_handler.setLevel(file_level.upper())
        logger.addHandler(file_handler)


def format_big_number(num, precision=0):
    suffixes = ["", "K", "M", "B", "T", "Q"]
    divisor = 1000.0

    for suffix in suffixes:
        if abs(num) < divisor:
            return f"{num:.{precision}f}{suffix}"
        num /= divisor

    return num


def _relative_path_between(path1: Path, path2: Path) -> Path:
    """Returns path1 relative to path2."""
    path1 = path1.absolute()
    path2 = path2.absolute()
    try:
        return path1.relative_to(path2)
    except ValueError:  # most likely because path1 is not a subpath of path2
        common_parts = Path(osp.commonpath([path1, path2])).parts
        return Path(
            "/".join([".."] * (len(path2.parts) - len(common_parts)) + list(path1.parts[len(common_parts) :]))
        )


def print_cuda_memory_usage():
    """Use this function to locate and debug memory leak."""
    import gc

    gc.collect()
    # Also clear the cache if you want to fully release the memory
    torch.cuda.empty_cache()
    print(f"Current GPU Memory Allocated: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB")
    print(f"Maximum GPU Memory Allocated: {torch.cuda.max_memory_allocated(0) / 1024**2:.2f} MB")
    print(f"Current GPU Memory Reserved: {torch.cuda.memory_reserved(0) / 1024**2:.2f} MB")
    print(f"Maximum GPU Memory Reserved: {torch.cuda.max_memory_reserved(0) / 1024**2:.2f} MB")


def capture_timestamp_utc():
    return datetime.now(timezone.utc)


def say(text: str, blocking: bool = False):
    system = platform.system()

    if system == "Darwin":
        cmd = ["say", text]

    elif system == "Linux":
        if shutil.which("espeak-ng"):
            cmd = ["espeak-ng", text]
        elif shutil.which("espeak"):
            cmd = ["espeak", text]
        else:
            cmd = ["spd-say", text]
            if blocking:
                cmd.append("--wait")

    elif system == "Windows":
        cmd = [
            "PowerShell",
            "-Command",
            "Add-Type -AssemblyName System.Speech; "
            f"(New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{text}')",
        ]

    else:
        raise RuntimeError("Unsupported operating system for text-to-speech.")

    if blocking:
        subprocess.run(cmd, check=True)
    else:
        subprocess.Popen(cmd, creationflags=subprocess.CREATE_NO_WINDOW if system == "Windows" else 0)


def _truthy_env(name: str) -> bool:
    return os.environ.get(name, "").lower() in {"1", "true", "yes", "on"}


def _voice_prompt_key(text: str) -> str | None:
    match = re.fullmatch(r"Recording episode (\d+)", text)
    if match:
        return f"recording_episode_{match.group(1)}"

    mapping = {
        "Reset the environment": "reset_the_environment",
        "Re-record episode": "re_record_episode",
        "Stop recording": "stop_recording",
        "Exiting": "exiting",
    }
    return mapping.get(text)


def play_cached_voice_prompt(text: str) -> bool:
    prompt_dir = os.environ.get("LEROBOT_VOICE_PROMPT_DIR")
    if not prompt_dir:
        return False

    key = _voice_prompt_key(text)
    if key is None:
        return False

    wav_path = Path(prompt_dir) / f"{key}.wav"
    if wav_path.exists() and shutil.which("aplay"):
        subprocess.run(["aplay", "-q", str(wav_path)], check=False)
        return True

    path = Path(prompt_dir) / f"{key}.mp3"
    if not path.exists():
        return False

    try:
        import pygame

        if not pygame.mixer.get_init():
            pygame.mixer.init()
        pygame.mixer.music.load(str(path))
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            time.sleep(0.02)
        return True
    except Exception as exc:
        logging.warning("Failed to play cached voice prompt %s: %s", path, exc)
        return False


def _play_tone(freq_hz: int, duration_s: float, volume: float = 0.35):
    if not shutil.which("aplay"):
        return

    sample_rate = 16_000
    samples = int(sample_rate * duration_s)
    amplitude = int(32767 * volume)
    path = Path("/tmp/lerobot_phase_beep.wav")

    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        frames = bytearray()
        for i in range(samples):
            sample = int(amplitude * math.sin(2 * math.pi * freq_hz * i / sample_rate))
            frames.extend(sample.to_bytes(2, byteorder="little", signed=True))
        wav.writeframes(frames)

    subprocess.run(["aplay", "-q", str(path)], check=False)


def play_phase_beep(text: str):
    if not _truthy_env("LEROBOT_PHASE_BEEP"):
        return

    lower = text.lower()
    if "recording episode" in lower:
        pattern = [(880, 0.16)]
    elif "reset" in lower:
        pattern = [(440, 0.12), (440, 0.12)]
    elif "re-record" in lower:
        pattern = [(660, 0.1), (660, 0.1), (660, 0.1)]
    elif "stop" in lower or "exiting" in lower:
        pattern = [(220, 0.3)]
    else:
        pattern = [(660, 0.1)]

    for index, (freq, duration) in enumerate(pattern):
        if index > 0:
            time.sleep(0.08)
        _play_tone(freq, duration)


def log_say(text: str, play_sounds: bool = True, blocking: bool = False):
    logging.info(text)

    if play_sounds:
        if not play_cached_voice_prompt(text):
            say(text, blocking)
    if blocking:
        play_phase_beep(text)


def get_channel_first_image_shape(image_shape: tuple) -> tuple:
    shape = copy(image_shape)
    if shape[2] < shape[0] and shape[2] < shape[1]:  # (h, w, c) -> (c, h, w)
        shape = (shape[2], shape[0], shape[1])
    elif not (shape[0] < shape[1] and shape[0] < shape[2]):
        raise ValueError(image_shape)

    return shape


def has_method(cls: object, method_name: str) -> bool:
    return hasattr(cls, method_name) and callable(getattr(cls, method_name))


def is_valid_numpy_dtype_string(dtype_str: str) -> bool:
    """
    Return True if a given string can be converted to a numpy dtype.
    """
    try:
        # Attempt to convert the string to a numpy dtype
        np.dtype(dtype_str)
        return True
    except TypeError:
        # If a TypeError is raised, the string is not a valid dtype
        return False


def enter_pressed() -> bool:
    if platform.system() == "Windows":
        import msvcrt

        if msvcrt.kbhit():
            key = msvcrt.getch()
            return key in (b"\r", b"\n")  # enter key
        return False
    else:
        return select.select([sys.stdin], [], [], 0)[0] and sys.stdin.readline().strip() == ""


def move_cursor_up(lines):
    """Move the cursor up by a specified number of lines."""
    print(f"\033[{lines}A", end="")


def get_elapsed_time_in_days_hours_minutes_seconds(elapsed_time_s: float):
    days = int(elapsed_time_s // (24 * 3600))
    elapsed_time_s %= 24 * 3600
    hours = int(elapsed_time_s // 3600)
    elapsed_time_s %= 3600
    minutes = int(elapsed_time_s // 60)
    seconds = elapsed_time_s % 60
    return days, hours, minutes, seconds


class TimerManager:
    """
    Lightweight utility to measure elapsed time.

    Examples
    --------
    ```python
    # Example 1: Using context manager
    timer = TimerManager("Policy", log=False)
    for _ in range(3):
        with timer:
            time.sleep(0.01)
    print(timer.last, timer.fps_avg, timer.percentile(90))  # Prints: 0.01 100.0 0.01
    ```

    ```python
    # Example 2: Using start/stop methods
    timer = TimerManager("Policy", log=False)
    timer.start()
    time.sleep(0.01)
    timer.stop()
    print(timer.last, timer.fps_avg, timer.percentile(90))  # Prints: 0.01 100.0 0.01
    ```
    """

    def __init__(
        self,
        label: str = "Elapsed-time",
        log: bool = True,
        logger: logging.Logger | None = None,
    ):
        self.label = label
        self.log = log
        self.logger = logger
        self._start: float | None = None
        self._history: list[float] = []

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def start(self):
        self._start = time.perf_counter()
        return self

    def stop(self) -> float:
        if self._start is None:
            raise RuntimeError("Timer was never started.")
        elapsed = time.perf_counter() - self._start
        self._history.append(elapsed)
        self._start = None
        if self.log:
            if self.logger is not None:
                self.logger.info(f"{self.label}: {elapsed:.6f} s")
            else:
                logging.info(f"{self.label}: {elapsed:.6f} s")
        return elapsed

    def reset(self):
        self._history.clear()

    @property
    def last(self) -> float:
        return self._history[-1] if self._history else 0.0

    @property
    def avg(self) -> float:
        return mean(self._history) if self._history else 0.0

    @property
    def total(self) -> float:
        return sum(self._history)

    @property
    def count(self) -> int:
        return len(self._history)

    @property
    def history(self) -> list[float]:
        return deepcopy(self._history)

    @property
    def fps_history(self) -> list[float]:
        return [1.0 / t for t in self._history]

    @property
    def fps_last(self) -> float:
        return 0.0 if self.last == 0 else 1.0 / self.last

    @property
    def fps_avg(self) -> float:
        return 0.0 if self.avg == 0 else 1.0 / self.avg

    def percentile(self, p: float) -> float:
        """
        Return the p-th percentile of recorded times.
        """
        if not self._history:
            return 0.0
        return float(np.percentile(self._history, p))

    def fps_percentile(self, p: float) -> float:
        """
        FPS corresponding to the p-th percentile time.
        """
        val = self.percentile(p)
        return 0.0 if val == 0 else 1.0 / val
