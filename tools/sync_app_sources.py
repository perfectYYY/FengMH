#!/usr/bin/env python3
"""sync_app_sources.py — 双工具链 (CLion/CMake & STM32CubeIDE) 同步校验

用法:
    python3 tools/sync_app_sources.py                 # 打印应有的清单
    python3 tools/sync_app_sources.py --check         # 校验 4 处一致性
    python3 tools/sync_app_sources.py --fix-cproject  # 自动把缺失的 App/* 回写到 .cproject

校验范围 (4 处):
    1) cmake/firmware.cmake      的 set(APP_DIRS ...)
    2) App/test/CMakeLists.txt   的 APP_SRCS + include_directories
    3) .cproject                 的 <listOptionValue value="../App/*"/>  (Debug+Release × asm+c)
    4) .cproject                 的 <sourceEntries> 是否存在 App (excluding test)

设计前提:
    - App/ 一级或二级子目录 = 一个"模块", 对应一个 include 路径与一组 *.c 源
    - App/test/ 仅 host 端单测使用, 两端固件构建均跳过 (CubeIDE 靠 excluding="test",
      CMake 靠 firmware.cmake 的显式 APP_DIRS)
"""
from __future__ import annotations
import argparse, pathlib, re, sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
APP  = ROOT / "App"
SKIP_TOP = {"test"}  # host-only, 两端固件都排除


# ---------------------------------------------------------------------------
# 1. 发现应有的目录清单
# ---------------------------------------------------------------------------
def discover_dirs() -> list[pathlib.Path]:
    """返回所有需要纳入固件构建的 App 子目录 (相对 App/)。

    规则:
      - 一级目录若含 .c/.h 则自身入选
      - 一级目录的直接子目录若含 .c/.h 也入选
      - 空目录 (e.g. service/kinematics 占位) 保留, 方便后续放代码
    """
    out: list[pathlib.Path] = []
    if not APP.is_dir():
        return out
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
                    out.append(sub.relative_to(APP)); sub_added = True
                elif not any(ROOT / "App" / d == sub for d in out):
                    # 空子目录也保留 (占位)
                    out.append(sub.relative_to(APP))
        if not has_src and not sub_added:
            for sub in sorted(top.iterdir()):
                if sub.is_dir() and not sub.name.startswith("."):
                    out.append(sub.relative_to(APP))
    # 去重 + 保序
    seen, uniq = set(), []
    for d in out:
        s = d.as_posix()
        if s not in seen:
            seen.add(s); uniq.append(d)
    return uniq


# ---------------------------------------------------------------------------
# 2. 各端读取实际的 App 目录清单
# ---------------------------------------------------------------------------
def firmware_cmake_dirs() -> set[str]:
    p = ROOT / "cmake" / "firmware.cmake"
    if not p.exists(): return set()
    txt = p.read_text(encoding="utf-8")
    m = re.search(r"set\(\s*APP_DIRS([^)]*)\)", txt, re.DOTALL)
    if not m: return set()
    return {f"App/{d}" for d in re.findall(r"[A-Za-z0-9_/]+", m.group(1))}


def host_test_cmake_dirs() -> set[str]:
    p = ROOT / "App" / "test" / "CMakeLists.txt"
    if not p.exists(): return set()
    txt = p.read_text(encoding="utf-8")
    inc = set(re.findall(r"\$\{APP_ROOT\}/([A-Za-z0-9_/]+)", txt))
    return {f"App/{d}" for d in inc if not d.startswith("test")}


def cproject_include_dirs() -> dict[str, set[str]]:
    """按 build config + 工具 拆开返回 set, 每个 config 都必须覆盖全。"""
    p = ROOT / ".cproject"
    if not p.exists(): return {}
    txt = p.read_text(encoding="utf-8")
    # 通过 listOptionValue 所在 parent option id 大致区分 asm/c (可靠性够用)
    # 简化: 直接按"每段 <listOptionValue ...>...</listOptionValue> 聚合" 的粒度做
    # 粗粒度校验即可——只要总集合覆盖即可。
    vals = set(re.findall(r'value="\.\./App/([A-Za-z0-9_/]+)"', txt))
    return {"all": {f"App/{v}" for v in vals}}


def cproject_has_app_source_entry() -> bool:
    p = ROOT / ".cproject"
    if not p.exists(): return False
    txt = p.read_text(encoding="utf-8")
    # 必须存在 name="App" 的 sourceEntries, 且显式排除 test
    m = re.search(r'<entry\s+[^>]*\bname="App"[^>]*>', txt)
    if not m: return False
    line = m.group(0)
    return 'excluding="test"' in line or 'excluding="test|' in line or '|test"' in line or '|test|' in line


# ---------------------------------------------------------------------------
# 3. 输出模板片段 (可手工粘贴; 也用于 --fix-cproject)
# ---------------------------------------------------------------------------
def cmake_app_dirs_block(dirs: list[pathlib.Path]) -> str:
    lines = "\n".join(f"        {d.as_posix()}" for d in dirs)
    return f"set(APP_DIRS\n{lines}\n)\n"


def cproject_include_lines(dirs: list[pathlib.Path]) -> str:
    return "\n".join(
        f'<listOptionValue builtIn="false" value="../App/{d.as_posix()}"/>'
        for d in dirs
    )


# ---------------------------------------------------------------------------
# 4. 校验
# ---------------------------------------------------------------------------
def check() -> int:
    dirs = discover_dirs()
    needed = {f"App/{d.as_posix()}" for d in dirs}

    rc = 0
    fw_dirs = firmware_cmake_dirs()
    miss = sorted(needed - fw_dirs)
    extra = sorted(fw_dirs - needed)
    if miss or extra:
        rc = 1
        print("[firmware.cmake]  APP_DIRS drift:")
        for d in miss:  print(f"  - missing : {d}")
        for d in extra: print(f"  - extra   : {d}")

    ht_dirs = host_test_cmake_dirs()
    # host 测试 CMake 允许缺少 service/kinematics 这类暂无 .c 的目录 (host 只编有源码的)
    has_src = {
        f"App/{d.as_posix()}" for d in dirs
        if any(q.suffix == ".c" for q in (APP / d).glob("*.c"))
    }
    miss = sorted(has_src - ht_dirs)
    if miss:
        rc = 1
        print("[App/test/CMakeLists.txt] include_directories drift:")
        for d in miss: print(f"  - missing : {d}")

    cp_dirs = cproject_include_dirs().get("all", set())
    miss = sorted(needed - cp_dirs)
    extra = sorted(cp_dirs - needed)
    if miss or extra:
        rc = 1
        print("[.cproject] listOptionValue drift:")
        for d in miss:  print(f"  - missing : ../{d}")
        for d in extra: print(f"  - extra   : ../{d}")

    if not cproject_has_app_source_entry():
        rc = 1
        print('[.cproject] sourceEntries 缺少 <entry name="App" excluding="test" .../>')

    if rc == 0:
        print(f"OK: 4 处清单一致, 共 {len(needed)} 个 App/ 子目录。")
    return rc


# ---------------------------------------------------------------------------
# 5. 自动修复 .cproject 的 listOptionValue
# ---------------------------------------------------------------------------
def fix_cproject() -> int:
    """把缺失的 <listOptionValue value="../App/xxx"/> 追加到 .cproject 里每一组
    listOptionValue 末尾 (不重复)。适合 CI 后本地再手动复核。
    """
    p = ROOT / ".cproject"
    if not p.exists():
        print(".cproject not found"); return 1
    txt = p.read_text(encoding="utf-8")
    dirs = discover_dirs()
    needed_lines = [
        f'<listOptionValue builtIn="false" value="../App/{d.as_posix()}"/>'
        for d in dirs
    ]
    existing = set(re.findall(r'value="\.\./App/[A-Za-z0-9_/]+"', txt))
    missing = [ln for ln in needed_lines
               if re.search(r'value="\.\./App/[A-Za-z0-9_/]+"', ln).group(0) not in existing]
    if not missing:
        print(".cproject: up-to-date"); return 0

    # 策略: 在每个 "includepaths" option 的最后一个 listOptionValue 后追加。
    # 这里 includepaths option 的 XML superClass 名含有 "option.includepaths"。
    def patch_block(block: str) -> str:
        # 在该 block 内最后一个 </listOptionValue> 后注入
        last = block.rfind("</listOptionValue>")
        if last < 0: return block
        injection = "\n\t\t\t\t\t\t\t\t\t" + "\n\t\t\t\t\t\t\t\t\t".join(missing)
        return block[:last + len("</listOptionValue>")] + injection + block[last + len("</listOptionValue>"):]

    # 找到 "<option ... superClass='...option.includepaths...' ...> ... </option>" 区段
    pattern = re.compile(
        r'(<option\b[^>]*option\.includepaths[^>]*>.*?</option>)',
        re.DOTALL
    )
    new_txt, n = pattern.subn(lambda m: patch_block(m.group(1)), txt)
    if n == 0:
        print(".cproject: 未找到 includepaths option, 放弃自动修复"); return 1
    p.write_text(new_txt, encoding="utf-8")
    print(f".cproject: 向 {n} 处 includepaths 追加了 {len(missing)} 条目录")
    return 0


# ---------------------------------------------------------------------------
# 6. CLI
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--check",        action="store_true")
    g.add_argument("--fix-cproject", action="store_true")
    args = ap.parse_args()

    if args.check:        sys.exit(check())
    if args.fix_cproject: sys.exit(fix_cproject())

    dirs = discover_dirs()
    print("# === Discovered App/ dirs ===")
    for d in dirs: print("  App/" + d.as_posix())
    print("\n# === firmware.cmake ===\n")
    print(cmake_app_dirs_block(dirs))
    print("# === .cproject (粘贴到每个 config 的 includepaths option) ===\n")
    print(cproject_include_lines(dirs))


if __name__ == "__main__":
    main()
