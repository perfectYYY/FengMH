#!/usr/bin/env python3
"""sync_app_sources.py — 双工具链 (CMake/CLion & STM32CubeIDE) 源/include 同步助手

用法:
    python3 tools/sync_app_sources.py            # 只打印应有的清单
    python3 tools/sync_app_sources.py --check    # 校验 CMakeLists.txt 与 .cproject
                                                 # 是否覆盖了 App/ 下所有子目录,
                                                 # 不一致则非零退出 (适合做 CI / pre-commit)

设计前提:
    - App/ 一级或二级子目录 = 一个"模块",对应一个 include 路径与一组 *.c 源
    - App/test/ 仅 host 端单测使用,两端固件构建均跳过
    - 当前 CMake 用 file(GLOB) 自动收集 *.c, 故"新增 .c"对 CMake 无感;
      但 CMake 的 target_include_directories(...) 是显式列表, 新增"目录"必须改
    - .cproject 既需要在 includepaths 中追加新目录, 也需要在 sourceEntries
      里通过 <entry name="App"/> + excluding=test 让 CubeIDE 自动扫到所有 .c

输出:
    1) CMake 片段: 可粘贴到 CMakeLists.txt 中 "App/ 分层" 块的
       target_include_directories / file(GLOB APP_SOURCES) 列表
    2) .cproject 片段: <listOptionValue ... value="../App/xxx"/> 行
       适合粘贴到 includepaths 区域 (assembler & c.compiler 各一份)
"""
from __future__ import annotations
import argparse, pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
APP = ROOT / "App"
SKIP_TOP = {"test"}  # host-only

def discover_dirs() -> list[pathlib.Path]:
    """返回所有需要纳入固件构建的 App 子目录 (相对 App/)。
       规则: 一级目录里若含 .c/.h 则自身入选; 否则递归一层。"""
    out: list[pathlib.Path] = []
    for top in sorted(APP.iterdir()):
        if not top.is_dir() or top.name in SKIP_TOP or top.name.startswith("."):
            continue
        has_src = any(p.suffix in (".c", ".h") for p in top.iterdir() if p.is_file())
        if has_src:
            out.append(top.relative_to(APP))
        sub_added = False
        for sub in sorted(top.iterdir()):
            if sub.is_dir() and not sub.name.startswith("."):
                if any(p.suffix in (".c", ".h") for p in sub.iterdir() if p.is_file()):
                    out.append(sub.relative_to(APP))
                    sub_added = True
        # 即使空目录 (e.g. service/kinematics) 也保留 include, 方便后续放代码
        if not has_src and not sub_added:
            for sub in sorted(top.iterdir()):
                if sub.is_dir():
                    out.append(sub.relative_to(APP))
    # 去重 + 保持顺序
    seen, uniq = set(), []
    for d in out:
        s = d.as_posix()
        if s not in seen:
            seen.add(s); uniq.append(d)
    return uniq

def cmake_snippet(dirs: list[pathlib.Path]) -> str:
    inc = "\n".join(f"        ${{CMAKE_SOURCE_DIR}}/App/{d.as_posix()}" for d in dirs)
    glb = "\n".join(f"        ${{CMAKE_SOURCE_DIR}}/App/{d.as_posix()}/*.c" for d in dirs)
    return (
        "target_include_directories(${PROJECT_NAME}.elf PRIVATE\n"
        f"{inc}\n)\n\n"
        "file(GLOB APP_SOURCES\n"
        f"{glb}\n)\n"
        "target_sources(${PROJECT_NAME}.elf PRIVATE ${APP_SOURCES})\n"
        "target_compile_definitions(${PROJECT_NAME}.elf PRIVATE APP_TARGET_HOST=0)\n"
    )

def cproject_snippet(dirs: list[pathlib.Path]) -> str:
    return "\n".join(
        f'<listOptionValue builtIn="false" value="../App/{d.as_posix()}"/>'
        for d in dirs
    )

def check() -> int:
    """对比 CMakeLists.txt 和 .cproject 是否包含所有应有的 include 路径。"""
    dirs = discover_dirs()
    needed = {f"App/{d.as_posix()}" for d in dirs}

    cm_files = [ROOT / "CMakeLists.txt", ROOT / "cmake" / "firmware.cmake"]
    cm = "\n".join(p.read_text(encoding="utf-8") for p in cm_files if p.exists())
    cm_have = set(re.findall(r"App/[A-Za-z0-9_/]+", cm))
    # firmware.cmake 里用 APP_DIRS 列表 (相对 App/) 的方式, 需要拼回前缀
    m = re.search(r"set\(APP_DIRS([^)]*)\)", cm, re.DOTALL)
    if m:
        for d in re.findall(r"[A-Za-z0-9_/]+", m.group(1)):
            cm_have.add("App/" + d)

    cp = (ROOT / ".cproject").read_text(encoding="utf-8")
    cp_have = set(p[3:] for p in re.findall(r'\.\./App/[A-Za-z0-9_/]+', cp))

    missing_cm = sorted(needed - cm_have)
    missing_cp = sorted(needed - cp_have)
    rc = 0
    if missing_cm:
        print("[CMakeLists.txt] missing App dirs:", *missing_cm, sep="\n  - ")
        rc = 1
    if missing_cp:
        print("[.cproject]      missing App dirs:", *missing_cp, sep="\n  - ")
        rc = 1
    if rc == 0:
        print(f"OK: both build files cover all {len(needed)} App/ dirs.")
    return rc

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="exit non-zero if drift detected")
    args = ap.parse_args()
    if args.check:
        sys.exit(check())
    dirs = discover_dirs()
    print("# === Discovered App/ dirs ===")
    for d in dirs: print("  App/" + d.as_posix())
    print("\n# === CMake snippet (paste into CMakeLists.txt App section) ===\n")
    print(cmake_snippet(dirs))
    print("# === .cproject snippet (paste into includepaths option of each config) ===\n")
    print(cproject_snippet(dirs))

if __name__ == "__main__":
    main()
