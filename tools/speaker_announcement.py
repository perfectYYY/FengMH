#!/usr/bin/env python3
"""通过 Ubuntu 的 USB 默认音频设备播报两段任务提示。"""

import shutil
import subprocess
import sys
import time


def speak(text: str) -> None:
    """使用 Speech Dispatcher 播报普通话，并等待播报结束。"""
    if shutil.which("spd-say") is None:
        raise RuntimeError("未找到 spd-say，请先安装 speech-dispatcher-utils")

    print(f"播报：{text}", flush=True)
    subprocess.run(
        [
            "spd-say",
            "--wait",
            "--language",
            "cmn",
            "--voice-type",
            "child_female",
            "--pitch",
            "35",
            "--rate",
            "8",
            text,
        ],
        check=True,
    )


def main() -> int:
    try:
        speak("正在进行智力题计算")
        print("等待 5 秒……", flush=True)
        time.sleep(5)
        speak("取模为2")
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"播报失败：{exc}", file=sys.stderr)
        return 1

    print("播报完成", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
