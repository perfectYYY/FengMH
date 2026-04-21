#!/usr/bin/env python3
"""One-shot patch for .cproject:
  1) Remove the two legacy cconfiguration blocks (869564940 / 1685296011)
     plus their scannerConfigBuildInfo entries.
  2) In the two remaining configs (1349390055 Debug / 1640766556 Release):
     - add App/ sourceEntry with App/test excluded
     - append App/* include paths to C compiler & assembler
     - add APP_TARGET_HOST=0 to defined symbols
"""
import re, sys, pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
CPROJ = ROOT / ".cproject"
text = CPROJ.read_text(encoding="utf-8")

LEGACY_IDS = [
    "com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug.869564940",
    "com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.release.1685296011",
]
KEEP_IDS = [
    "com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.debug.1349390055",
    "com.st.stm32cube.ide.mcu.gnu.managedbuild.config.exe.release.1640766556",
]

APP_INCLUDES = [
    "../App/common",
    "../App/bsp",
    "../App/device",
    "../App/service/pid",
    "../App/service/protocol",
    "../App/service/gait",
    "../App/service/leg",
    "../App/service/kinematics",
    "../App/app",
]
APP_DEFINE = "APP_TARGET_HOST=0"

# ---------- 1) remove legacy <cconfiguration ...> ... </cconfiguration> ----------
def drop_cconfig(txt, cid):
    # Match a <cconfiguration id="cid"...> ... </cconfiguration>, non-greedy.
    pat = re.compile(
        r'[\t ]*<cconfiguration id="' + re.escape(cid) + r'".*?</cconfiguration>\s*',
        re.DOTALL,
    )
    new, n = pat.subn("", txt)
    if n > 1:
        raise SystemExit(f"[drop_cconfig] unexpected matches for {cid}: {n}")
    return new

for cid in LEGACY_IDS:
    text = drop_cconfig(text, cid)

# also drop matching scannerConfigBuildInfo blocks referring to those ids
def drop_scanner(txt, cid):
    pat = re.compile(
        r'[\t ]*<scannerConfigBuildInfo instanceId="' + re.escape(cid) + r';.*?</scannerConfigBuildInfo>\s*',
        re.DOTALL,
    )
    new, n = pat.subn("", txt)
    if n not in (0, 1):
        raise SystemExit(f"[drop_scanner] unexpected matches for {cid}: {n}")
    return new

for cid in LEGACY_IDS:
    text = drop_scanner(text, cid)

# ---------- 2) patch remaining configs ----------
def slice_config(txt, cid):
    """Return (start, end) of a <cconfiguration id="cid" ...>...</cconfiguration>"""
    pat = re.compile(
        r'<cconfiguration id="' + re.escape(cid) + r'".*?</cconfiguration>',
        re.DOTALL,
    )
    m = pat.search(txt)
    if not m:
        raise SystemExit(f"[slice] not found: {cid}")
    return m.start(), m.end()

def patch_block(block):
    # a) include paths for C compiler (tool.c.compiler.option.includepaths)
    def add_includes(m):
        header, body, tail = m.group(1), m.group(2), m.group(3)
        existing = set(re.findall(r'value="([^"]+)"', body))
        extra = "".join(
            f'\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value="{p}"/>'
            for p in APP_INCLUDES if p not in existing
        )
        return f"{header}{body}{extra}\n\t\t\t\t\t\t\t\t{tail}"
    block = re.sub(
        r'(<option[^>]*id="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.c\.compiler\.option\.includepaths[^>]*valueType="includePath">)(.*?)(</option>)',
        add_includes, block, flags=re.DOTALL,
    )
    # b) include paths for assembler
    block = re.sub(
        r'(<option[^>]*id="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.assembler\.option\.includepaths[^>]*valueType="includePath">)(.*?)(</option>)',
        add_includes, block, flags=re.DOTALL,
    )
    # c) defined symbols for C compiler: add APP_TARGET_HOST=0
    def add_define(m):
        header, body, tail = m.group(1), m.group(2), m.group(3)
        if APP_DEFINE in body:
            return m.group(0)
        extra = f'\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value="{APP_DEFINE}"/>'
        return f"{header}{body}{extra}\n\t\t\t\t\t\t\t\t{tail}"
    block = re.sub(
        r'(<option[^>]*id="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.c\.compiler\.option\.definedsymbols[^>]*valueType="definedSymbols">)(.*?)(</option>)',
        add_define, block, flags=re.DOTALL,
    )
    # d) sourceEntries: append <entry ... name="App" excluding="test"/>
    def patch_sources(m):
        inner = m.group(1)
        if 'name="App"' in inner:
            return m.group(0)
        new_entry = '\t\t\t\t\t\t<entry excluding="test" flags="VALUE_WORKSPACE_PATH|RESOLVED" kind="sourcePath" name="App"/>\n'
        return f"<sourceEntries>\n{inner}{new_entry}\t\t\t\t\t</sourceEntries>"
    block = re.sub(
        r'<sourceEntries>\s*(.*?)\s*</sourceEntries>',
        patch_sources, block, flags=re.DOTALL,
    )
    return block

for cid in KEEP_IDS:
    s, e = slice_config(text, cid)
    new_block = patch_block(text[s:e])
    text = text[:s] + new_block + text[e:]

CPROJ.write_text(text, encoding="utf-8")
print("patched .cproject OK")
