#!/usr/bin/env python3
"""Validate the disk images left behind by the harness test run.

Checks the on-disk REL structures (side sector and data sector chains,
directory entries, BAM allocation) and the record contents of the
files written by the CRUD tests, independently of the code under test.
c1541 reads selected files back and must not find anything to fix with
-validate.
"""

import os
import shutil
import sys

import d64lib

C1541 = os.environ.get("C1541", "c1541")

FAILURES = 0


def check(cond, msg):
    global FAILURES
    if not cond:
        FAILURES += 1
        print("FAIL: " + msg)


def testbyte(i, seed):
    """Replicate the harness fill_data() pattern."""
    b = (seed + i * 13) & 0xFF
    return b if b else 0x55


def c1541_validate_unchanged(image, tmp, label):
    """Run c1541 -validate on a copy of image, it must not change it."""
    shutil.copy(image, tmp)
    try:
        rc, out = d64lib.run_c1541(C1541, ["-attach", tmp, "-validate"])
        check(rc == 0, "%s: c1541 -validate failed: %s" % (label, out.strip()))
        if rc != 0:
            return
        with open(tmp, "rb") as f:
            after = f.read()
        with open(image, "rb") as f:
            before = f.read()
        check(before == after, "c1541 -validate changed the %s image" % label)
    finally:
        os.remove(tmp)


def main():
    datadir = sys.argv[1] if len(sys.argv) > 1 else "testdata"

    # the harness writes its working image into the current directory
    d = d64lib.D64("work.d64")

    # --- T1: record length 20, records 1-4 ----------------------------
    reclen, records, data_ts, ss_blocks = d.read_rel("T1")
    check(reclen == 20, "T1 record length %d" % reclen)
    ent = d.find("T1")
    check(ent[3][0] == 0x84, "T1 dir type %02x" % ent[3][0])
    check(ent[3][28] + 256 * ent[3][29] == len(data_ts) + len(ss_blocks),
          "T1 dir size %d, allocated %d" % (ent[3][28] + 256 * ent[3][29],
                                            len(data_ts) + len(ss_blocks)))

    expect = bytes(testbyte(i, 1) for i in range(20))
    check(records[0] == expect, "T1 record 1 mismatch")
    # record 2: 5 bytes of seed-200 data written first, then 3 bytes of
    # seed-7 data overwritten at offset 3, rest zero-filled
    r2 = records[1]
    check(r2[0:3] == bytes(testbyte(i, 200) for i in range(3)),
          "T1 record 2 prefix mismatch")
    check(r2[3:6] == bytes(testbyte(i, 7) for i in range(3)),
          "T1 record 2 overwrite mismatch")
    check(r2[6:20] == bytes(14), "T1 record 2 tail not zero-filled")
    check(records[2] == bytes(testbyte(i, 100) for i in range(20)),
          "T1 record 3 mismatch")
    # record 4 is untouched: single 0xff byte plus the fill pattern
    check(records[3][0] == 0xff and records[3][1:] == bytes(19),
          "T1 record 4 not fresh")
    # record 13 was appended by the persistence test
    check(records[12] == bytes(testbyte(i, 150) for i in range(20)),
          "T1 record 13 mismatch")


    # --- T3: five 200-byte records spanning sectors --------------------
    reclen, records, data_ts, ss_blocks = d.read_rel("T3")
    check(reclen == 200, "T3 record length %d" % reclen)
    for i in (0, 2, 3, 4):  # record 2 was modified at offset 179
        expect = bytes(testbyte(j, i * 30 + 1) for j in range(200))
        check(records[i] == expect, "T3 record %d mismatch" % (i + 1))
    r2 = records[1]
    check(r2[0:179] == bytes(testbyte(j, 31) for j in range(179)),
          "T3 record 2 prefix mismatch")
    check(r2[179:189] == bytes(testbyte(j, 99) for j in range(10)),
          "T3 record 2 cross-sector write mismatch")
    check(r2[189:200] == bytes(11), "T3 record 2 tail not zero-filled")

    # --- T5: 300 records of 7 bytes ------------------------------------
    reclen, records, data_ts, ss_blocks = d.read_rel("T5")
    check(reclen == 7, "T5 record length %d" % reclen)
    check(len(records) >= 300, "T5 has %d records" % len(records))
    for i in range(min(300, len(records))):
        expect = bytes([((i + 1) & 0xFF)] +
                       [testbyte(j + 1, i) for j in range(6)])
        if records[i] != expect:
            check(False, "T5 record %d mismatch" % (i + 1))
            break

    # --- T2 must be scratched ------------------------------------------
    check(d.find("T2") is None, "T2 still present after scratch")

    # --- T4: three full-sector records ----------------------------------
    reclen, records, data_ts, ss_blocks = d.read_rel("T4")
    check(reclen == 254, "T4 record length %d" % reclen)
    for i in range(3):
        expect = bytes(testbyte(j, 10 * i + 2) for j in range(254))
        check(records[i] == expect, "T4 record %d mismatch" % (i + 1))

    # --- every REL file on the image is allocated in the BAM -----------
    for name in ("T1", "T3", "T4", "T5", "M1"):
        if d.find(name) is not None:
            _, _, data_ts, ss_blocks = d.read_rel(name)
            try:
                d.bam_check_allocated(data_ts +
                                      [(b[0], b[1]) for b in ss_blocks])
            except AssertionError as e:
                check(False, "%s: %s" % (name, e))
        else:
            check(False, "%s missing on work.d64" % name)

    # --- multi.d64 (created by genimages.py) ----------------------------
    d = d64lib.D64(os.path.join(datadir, "multi.d64"))
    reclen, records, data_ts, ss_blocks = d.read_rel("MULTIREL",
                                                     expect_reclen=17,
                                                     expect_records=40)
    for r in range(40):
        expect = bytes((r * 11 + i * 3) & 0xFF if (r + i) % 5 else 0
                       for i in range(17))
        check(records[r] == expect, "MULTIREL record %d mismatch" % (r + 1))

    # --- VICE acceptance: read the firmware-written REL files with
    #     c1541 and compare them record by record. The harness stores
    #     ASCII uppercase names, so pass them in lowercase. c1541
    #     delivers the full untrimmed records concatenated.
    def rel_read_bytes(image, name):
        outname = os.path.join(datadir, "extract_%s.bin" % name)
        try:
            rc, out = d64lib.run_c1541(C1541, ["-attach", image, "-read",
                                               "%s,l" % name.lower(), outname])
            if rc != 0:
                return None, "c1541 -read failed: %s" % out.strip()
            with open(outname, "rb") as f:
                return f.read(), None
        except OSError as e:
            return None, str(e)
        finally:
            if os.path.exists(outname):
                os.remove(outname)

    image = "work.d64"
    # M1: c1541 delivers full untrimmed records: 129 fresh records
    # (0xff followed by zeros) and the written record 130. Writing the
    # last record does not grow the file, so there is no extra record.
    fresh = b"\xff" + b"\x00" * 253
    expected = fresh * 129
    expected += bytes(testbyte(i, 77) for i in range(254))
    got, err = rel_read_bytes(image, "M1")
    check(err is None, "M1: %s" % err)
    if got is not None:
        check(got == expected, "M1: %d bytes differ from expected %d bytes"
              % (len(got), len(expected)))
    c1541_validate_unchanged(image, os.path.join(datadir, "validate_tmp.d64"),
                             "D64")

    # G1 on the D81 image: the harness writes work.d81 into the
    # current directory. c1541 delivers full untrimmed 4-byte records,
    # fresh ones as ff 00 00 00.
    image81 = "work.d81"
    check(os.path.exists(image81), "work.d81 is missing")
    if os.path.exists(image81):
        fresh4 = b"\xff\x00\x00\x00"
        expected = fresh4 * 45719
        expected += bytes(testbyte(i, 88) for i in range(4))
        got, err = rel_read_bytes(image81, "G1")
        check(err is None, "G1: %s" % err)
        if got is not None:
            check(got == expected,
                  "G1: %d bytes differ from expected %d bytes"
                  % (len(got), len(expected)))

        # c1541 -validate must not find anything to fix
        c1541_validate_unchanged(image81,
                                 os.path.join(datadir, "validate_tmp.d81"),
                                 "D81")

    # D71 cross-check: read D2 (720 records of 254 bytes) and verify
    # that -validate leaves the image untouched
    image71 = "work.d71"
    check(os.path.exists(image71), "work.d71 is missing")
    if os.path.exists(image71):
        fresh = b"\xff" + b"\x00" * 253
        expected = fresh * 719
        expected += bytes(testbyte(i, 13) for i in range(254))
        got, err = rel_read_bytes(image71, "D2")
        check(err is None, "D2: %s" % err)
        if got is not None:
            check(got == expected,
                  "D2: %d bytes differ from expected %d bytes"
                  % (len(got), len(expected)))

        c1541_validate_unchanged(image71,
                                 os.path.join(datadir, "validate_tmp.d71"),
                                 "D71")

    if FAILURES:
        print("%d validation failures" % FAILURES)
        return 1
    print("validation OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (AssertionError, OSError) as e:
        print("FAIL: %s" % e)
        sys.exit(1)
