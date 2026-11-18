#!/usr/bin/env python3
"""Convert a binary file into a C header with a const array.

Usage: bindata.py <input> <output> <symbol>

The generated array is placed in the .flashdata section on the Pico SDK
(where __in_flash is available) so it is not copied to RAM by the
copy_to_ram binary type.
"""

import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 4:
        print(f"Usage: {sys.argv[0]} <input> <output> <symbol>", file=sys.stderr)
        return 1

    input_path, output_path, symbol = sys.argv[1:]
    data = Path(input_path).read_bytes()

    with open(output_path, "w", encoding="ascii") as out:
        out.write(f"/* Generated from {input_path} by scripts/bindata.py - do not edit */\n\n")
        out.write("#include <stdint.h>\n\n")
        out.write(f"const uint32_t {symbol}_len = {len(data)};\n\n")
        out.write(f"const uint8_t {symbol}[]\n")
        out.write("#if defined(PICO_ON_DEVICE) && PICO_ON_DEVICE\n")
        out.write('__in_flash("embedded_d64")\n')
        out.write("#endif\n")
        out.write("= {\n")
        for offset in range(0, len(data), 16):
            chunk = data[offset : offset + 16]
            out.write("  " + ", ".join(f"0x{byte:02x}" for byte in chunk) + ",\n")
        out.write("};\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
