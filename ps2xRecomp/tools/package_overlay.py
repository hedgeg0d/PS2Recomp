#!/usr/bin/env python3
"""Package a separate per-function PS2Recomp export as a runtime code overlay.

No guest writes. The identity source supplies immutable instruction words
which select this variant after the game loads it. Existing generated files
are rewritten only when their content changes.
"""

import argparse
from pathlib import Path
import re


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--generated", required=True, type=Path)
    parser.add_argument("--identity-source", required=True, type=Path)
    parser.add_argument("--begin", required=True, type=lambda v: int(v, 0))
    parser.add_argument("--end", required=True, type=lambda v: int(v, 0))
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    try:
        generated = args.generated.resolve(strict=True)
        output = args.output.resolve()
        if output == generated:
            raise ValueError("output must differ from the original generated directory")
        if not 0 <= args.begin < args.end <= 0x2000000 or (args.begin | args.end) & 3:
            raise ValueError("invalid physical EE code range")
        for path in (generated, output):
            if any(c in str(path) for c in '\n\r";$'):
                raise ValueError("path contains characters unsupported by generated CMake/C++")
        registration = (generated / "register_functions.cpp").read_text()
        entries = {}
        for name, address in re.findall(
                r"g_ps2RecompiledFunctionTable\[\d+\]\s*=\s*(\w+);\s*//\s*0x([0-9a-fA-F]+)", registration):
            address = int(address, 16)
            if not args.begin <= address < args.end:
                raise ValueError(f"export entry {address:#x} outside overlay range")
            if address in entries:
                raise ValueError(f"duplicate export entry {address:#x}")
            entries[address] = name
        if not entries:
            raise ValueError("no generated registrations found")
        identity = {}
        for address, value in re.findall(
                r"^\s*//\s*0x([0-9a-fA-F]+):\s*0x([0-9a-fA-F]+)\s+", args.identity_source.read_text(), re.M):
            address, value = int(address, 16), int(value, 16)
            if not args.begin <= address <= args.end - 4 or address & 3 or value > 0xffffffff:
                raise ValueError("identity instruction outside overlay range")
            if address in identity and identity[address] != value:
                raise ValueError("conflicting identity instruction")
            identity[address] = value
        if not identity:
            raise ValueError("identity source contains no instruction words")
        names = sorted(set(entries.values()))
        for name in names:
            if not (generated / f"{name}.cpp").is_file():
                raise ValueError(f"missing generated source for {name}")
        output.mkdir(parents=True, exist_ok=True)

        def write(name, content):
            path = output / name
            if not path.exists() or path.read_text() != content:
                path.write_text(content)

        prefix = ('#include "ps2_runtime.h"\n#include "ps2_runtime_macros.h"\n'
                  '#include "ps2_syscalls.h"\n#include "ps2_stubs.h"\n#include <stdexcept>\n'
                  f'#include "{generated}/ps2_recompiled_functions.h"\n'
                  f'#include "{generated}/ps2_recompiled_stubs.h"\n'
                  '#include "overlay_declarations.h"\n')
        declarations = '#pragma once\n#include "ps2_runtime.h"\nnamespace CompiledOverlay {\n'
        declarations += ''.join(f'void {name}(uint8_t *, R5900Context *, PS2Runtime *);\n' for name in names)
        declarations += '}\nvoid installGeneratedOverlay(PS2Runtime *, const uint8_t *);\n'
        write("overlay_declarations.h", declarations)
        files = []
        for index in range(0, len(names), 16):
            name = f"overlay_unit_{index // 16}.cpp"
            body = prefix + 'namespace CompiledOverlay {\n'
            body += ''.join(f'#include "{generated}/{function}.cpp"\n' for function in names[index:index + 16])
            write(name, body + '}\n')
            files.append(name)
        body = ('#include "overlay_declarations.h"\n#include "runtime/code_overlays.h"\n'
                '#include <utility>\n'
                'void installGeneratedOverlay(PS2Runtime *runtime, const uint8_t *ram) {\n'
                f'    ps2x::CodeOverlay overlay{{0x{args.begin:x}u, 0x{args.end:x}u, {{\n')
        body += ''.join(f'        {{0x{address:x}u, 0x{value:08x}u}},\n' for address, value in sorted(identity.items()))
        body += '    }, {\n'
        body += ''.join(f'        {{0x{address:x}u, CompiledOverlay::{name}}},\n' for address, name in sorted(entries.items()))
        body += '    }};\n    ps2x::setCodeOverlays(runtime, ram, {std::move(overlay)});\n}\n'
        write("overlay_registration.cpp", body)
        files.append("overlay_registration.cpp")
        manifest = 'set(PS2_OVERLAY_SOURCES\n'
        manifest += ''.join(f'    "${{CMAKE_CURRENT_LIST_DIR}}/{name}"\n' for name in files)
        write("overlay_sources.cmake", manifest + ')\n')
        print(f"{len(names)} functions, {len(entries)} entries, {len(identity)} identity words -> {output}")
    except (OSError, UnicodeError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
