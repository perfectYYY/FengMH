#!/usr/bin/env python3
"""patch_cmakelists.py — 修复 CubeMX 重新生成后的 CMakeLists (幂等)

两件事:
  1. 确保 CMakeLists.txt / CMakeLists_template.txt 末尾有
         include(${CMAKE_SOURCE_DIR}/cmake/firmware.cmake)
     (模板里是 $${CMAKE_SOURCE_DIR} 形式)
  2. 强制从 GLOB_RECURSE SOURCES 里移除 "App/*.*" 与 "App/test/*" 等脏 glob
     (App/ 由 cmake/firmware.cmake 统一显式纳入)

用法:
    python3 tools/patch_cmakelists.py           # 修复
    python3 tools/patch_cmakelists.py --check   # 仅校验, 缺失则非零退出
"""
import argparse, pathlib, re, sys

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


# 匹配 GLOB_RECURSE 行, 并清掉其中落在 App/ 下的模式
_GLOB_RE = re.compile(
    r'file\s*\(\s*GLOB(?:_RECURSE)?\s+SOURCES\b[^)]*\)',
    re.DOTALL
)

def _strip_app_patterns(glob_line: str) -> str:
    """从一行 file(GLOB_RECURSE SOURCES ...) 里移除所有含 App 的 "xxx" 项。"""
    # 保留 "Core/*.*" "Drivers/*.*" 这类, 去掉 "App/*.*"/"App/test/*" 等
    return re.sub(r'\s*"App[^"]*"', '', glob_line)


def patch_text(txt: str) -> tuple[str, list[str]]:
    changes: list[str] = []
    # 1) 清 glob
    def _sub_glob(m):
        old = m.group(0)
        new = _strip_app_patterns(old)
        if new != old:
            changes.append("stripped App/* from GLOB_RECURSE SOURCES")
        return new
    txt = _GLOB_RE.sub(_sub_glob, txt)
    return txt, changes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args()
    rc = 0
    for path, marker, snippet in TARGETS:
        if not path.exists():
            print(f"[skip] {path} not found"); continue
        txt = path.read_text(encoding="utf-8")
        new_txt, changes = patch_text(txt)

        has_include = marker in new_txt
        needs_include = not has_include
        needs_clean   = bool(changes)

        if args.check:
            if needs_include:
                print(f"[MISS] {path.name}: missing `{marker}`"); rc = 1
            if needs_clean:
                print(f"[MISS] {path.name}: App/* leaked into GLOB_RECURSE"); rc = 1
            if not (needs_include or needs_clean):
                print(f"[ok]   {path.name}")
            continue

        if needs_include:
            if not new_txt.endswith("\n"): new_txt += "\n"
            new_txt += snippet
            changes.append("appended include(firmware.cmake)")
        if new_txt != txt:
            path.write_text(new_txt, encoding="utf-8")
            print(f"[fixed] {path.name}: " + "; ".join(changes))
        else:
            print(f"[ok]    {path.name}")
    sys.exit(rc)


if __name__ == "__main__":
    main()
