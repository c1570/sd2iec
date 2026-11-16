#!/usr/bin/env python3
"""Create the disk images used by the host-side REL test harness.

- blank.d64 : freshly formatted, empty D64 (built by d64lib)
- multi.d64 : D64 with a multi-record REL file (built by d64lib)
- blank.d71 : freshly formatted D71 (c1541)
- blank.d81 : freshly formatted D81 (c1541)
- c1541.d64 : D64 with a REL file written by c1541, used as a
              cross-check for the on-disk structures

All images are written to the directory given as the first argument.
"""

import os
import sys

import d64lib

C1541 = os.environ.get("C1541", "c1541")
OUT = (os.path.abspath(sys.argv[1]) if len(sys.argv) > 1
       else os.path.join(os.path.dirname(os.path.abspath(__file__)), "testdata"))

# files written by generate(), removed again on failure
IMAGES = ("blank.d64", "multi.d64", "blank.d81", "blank.d71", "c1541.d64",
          "data.bin")

# c1541 takes the record length as a single character after ",l,"
C1541_RECLEN = 50


def c1541(args):
    rc, out = d64lib.run_c1541(C1541, args)
    if rc != 0:
        raise RuntimeError("c1541 %s failed: %s" % (" ".join(args), out))
    return out


def generate():
    # 1. pristine, empty D64
    blank = d64lib.make_d64()
    with open(os.path.join(OUT, "blank.d64"), "wb") as f:
        f.write(blank)
    print("blank.d64 written")

    # 2. multi-record REL file for external validation
    img = d64lib.make_d64()
    records = [bytes((r * 11 + i * 3) & 0xFF if (r + i) % 5 else 0
                     for i in range(17))
               for r in range(40)]
    used = d64lib.create_rel(img, "MULTIREL", 17, records)
    d64lib.check_allocated(img, used)
    with open(os.path.join(OUT, "multi.d64"), "wb") as f:
        f.write(img)
    print("multi.d64 written (40 records of length 17, the last data "
          "sector holds four more empty ones)")

    # 3. blank D81 for the super side sector tests
    c1541(["-format", "d81test,00", "d81", os.path.join(OUT, "blank.d81")])
    print("blank.d81 written")

    # 4. blank D71 for the side sector limit test
    c1541(["-format", "d71test,00", "d71", os.path.join(OUT, "blank.d71")])
    print("blank.d71 written")

    # 5. REL file created by c1541 itself: one full record
    data = bytes((i * 7 + 3) & 0xFF for i in range(C1541_RECLEN))
    datafile = os.path.join(OUT, "data.bin")
    with open(datafile, "wb") as f:
        f.write(data)
    image = os.path.join(OUT, "c1541.d64")
    try:
        c1541(["-format", "reltest,00", "d64", image])
        c1541(["-attach", image, "-write", datafile,
               "MYREL,L," + chr(C1541_RECLEN)])
    finally:
        os.remove(datafile)
    print("c1541.d64 written (REL file created by c1541)")

    # sanity check: c1541 must list the file as REL
    out = c1541(["-attach", image, "-list"])
    if not any('"myrel"' in l and l.split()[-1] == "rel"
               for l in out.lower().splitlines()):
        raise RuntimeError("c1541 does not list MYREL as REL file: " + out)
    print("c1541 listing confirms REL file")


def main():
    os.makedirs(OUT, exist_ok=True)
    try:
        generate()
    except (RuntimeError, AssertionError, OSError, ValueError) as e:
        # leave no partial set behind, make would treat it as up to date
        for name in IMAGES:
            try:
                os.remove(os.path.join(OUT, name))
            except FileNotFoundError:
                pass
        print("genimages: %s" % e)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
