#!/usr/bin/env python3
"""CubeMX 重新生成 CMakeLists.txt / CMakeLists_template.txt 后,
把结尾的 `include(...)` 行恢复进去。幂等, 可反复运行。

用途:
    python3 tools/patch_cmakelists.py           # 修复
    python3 tools/patch_cmakelists.py --check   # 仅校验, 缺失则非零退出
"""
import argparse, pathlib, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]

TARGETS = [
    (ROOT / "CMakeLists.txt",
     "include(${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)",
     "\n# === 工程定制: cmake/firmware.cmake 不会被 CubeMX 覆盖 ===\n"
     "include(${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)\n"),
    (ROOT / "CMakeLists_template.txt",
     "include($${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)",
     "\n# === 工程定制: 必须保留这一行 ===\n"
     "include($${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)\n"),
]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    rc = 0
    for path, marker, snippet in TARGETS:
        if not path.exists():
            print(f"[skip] {path} not found"); continue
        txt = path.read_text(encoding="utf-8")
        if marker in txt:
            print(f"[ok]    {path.name}")
        elif args.check:
            print(f"[MISS]  {path.name}: missing `{marker}`"); rc = 1
        else:
            if not txt.endswith("\n"): txt += "\n"
            path.write_text(txt + snippet, encoding="utf-8")
            print(f"[fixed] {path.name}")
    sys.exit(rc)

if __name__ == "__main__":
    main()
