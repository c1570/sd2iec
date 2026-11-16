#!/usr/bin/env python3
"""Minimal D64/D71/D81 image inspection and REL file decoding.
The BAM helpers and the image builder handle 35-track D64 images only.

Implements the CBM DOS REL structures per the 1541 ROM/VICE references:
- dir entry: 0=type, 1-2=first data sector, 3-18=name, 19-20=first side sector,
  21=record length, 28-29=block count
- side sector: 0-1=next t/s, 2=SS number, 3=record length, 4-15=t/s of SS0..SS5,
  16-255=120 data sector pointers
- super side sector (D81): 0-1=first side sector, 2=0xfe, 3-254=group heads
- data sectors: 0-1=t/s links, payload records concatenated across links
"""

import subprocess

TRACK_SECTORS = [21]*17 + [19]*7 + [18]*6 + [17]*5

def lba(track, sector, d81=False):
    if d81:
        return (track - 1)*40 + sector
    off = 0
    t = track - 1
    if t >= 35:
        off = 683
        t -= 35
    if t < 17: return off + t*21 + sector
    if t < 24: return off + 17*21 + (t-17)*19 + sector
    if t < 30: return off + 17*21 + 7*19 + (t-24)*18 + sector
    return off + 17*21 + 7*19 + 6*18 + (t-30)*17 + sector

class D64:
    def __init__(self, fn, d81=False):
        with open(fn, "rb") as f:
            self.data = bytearray(f.read())
        # strip the error info block if present
        for size, errinfo in ((174848, 683), (349696, 1366), (819200, 3200)):
            if len(self.data) == size + errinfo:
                self.data = self.data[:size]
        self.d81 = d81 or len(self.data) == 819200

    def read_sector(self, track, sector):
        o = 256 * lba(track, sector, self.d81)
        return self.data[o:o+256]

    def dir_entries(self):
        if self.d81:
            t, s = 40, 3
        else:
            t, s = 18, 1
        while t:
            sec = self.read_sector(t, s)
            for i in range(8):
                e = sec[2+32*i:34+32*i]
                if e[0] != 0:
                    yield (t, s, i, e)
            t, s = sec[0], sec[1]

    @staticmethod
    def _petscii_to_ascii(b):
        """Map both PETSCII letter ranges to ASCII letters; callers
        compare case-insensitively."""
        if 0xC1 <= b <= 0xDA:
            return b - 0x80
        if 0x41 <= b <= 0x5A:
            return b + 0x20
        return b

    def find(self, name):
        want = name.upper().ljust(16, '\xa0').encode('latin-1')
        for (t, s, i, e) in self.dir_entries():
            got = bytes(self._petscii_to_ascii(b) for b in e[3:19]).upper()
            if got == want:
                return (t, s, i, bytearray(e))
        return None

    def read_rel(self, name, expect_reclen=None, expect_records=None):
        """Walk the REL structures of a file.

        Returns (record_length, records, data_ts, ss_blocks): the record
        contents, the data sector t/s list and the side sector blocks as
        (track, sector, bytes). Without expect_records the record count
        is derived from the used bytes of the last data sector, like the
        drive does it.
        """
        ent = self.find(name)
        assert ent, "file not found: %s" % name
        e = ent[3]
        assert e[0] & 0x07 == 4, "%s is not a REL file (type %02x)" % (name, e[0])
        reclen = e[21]
        if expect_reclen is not None:
            assert reclen == expect_reclen, "record length %d != expected %d" % (reclen, expect_reclen)
        ss_t, ss_s = e[19], e[20]
        assert ss_t != 0, "no side sector pointer in dir entry"

        # a super side sector points at the first group, the side
        # sector chain continues across all groups
        first = self.read_sector(ss_t, ss_s)
        if first[2] == 0xfe:
            ss_t, ss_s = first[0], first[1]

        # collect data sector t/s list by walking side sector chain
        data_ts = []
        ss_blocks = []
        t, s = ss_t, ss_s
        while t:
            ss = self.read_sector(t, s)
            ss_blocks.append((t, s, bytes(ss)))
            # side sector numbers restart at 0 in every group of six
            assert ss[2] == (len(ss_blocks)-1) % 6, "side sector number mismatch"
            assert ss[3] == reclen, "side sector record length mismatch"
            for i in range(120):
                dt, ds = ss[16+2*i], ss[17+2*i]
                if dt == 0:
                    break
                data_ts.append((dt, ds))
            t, s = ss[0], ss[1]

        # read all data sectors, check their chain, concatenate payloads
        assert data_ts, "REL file without data sectors"
        blob = bytearray()
        for i, (dt, ds) in enumerate(data_ts):
            sec = self.read_sector(dt, ds)
            if i + 1 < len(data_ts):
                assert (sec[0], sec[1]) == data_ts[i+1], \
                    "data sector %d/%d does not link to the next one" % (dt, ds)
            else:
                assert sec[0] == 0, "last data sector %d/%d has a link" % (dt, ds)
                used = sec[1] - 1 if sec[1] >= 2 else 0
            blob += sec[2:256]

        if expect_records is not None:
            assert len(blob) >= expect_records * reclen, \
                "file too small: %d bytes < %d bytes for %d records" % (
                    len(blob), expect_records * reclen, expect_records)
            n = expect_records
        else:
            n = ((len(data_ts) - 1) * 254 + used) // reclen

        records = []
        for i in range(n):
            records.append(bytes(blob[i*reclen:(i+1)*reclen]))
        return reclen, records, data_ts, ss_blocks

    def bam_check_allocated(self, sectors):
        """Assert every (track,sector) in list is marked allocated in the BAM."""
        check_allocated(self.data, sectors)


def check_allocated(img, sectors):
    """Assert every (track,sector) is marked allocated in a D41 BAM."""
    bam = img[256*lba(18, 0):256*lba(18, 0)+256]
    for (t, s) in sectors:
        assert 1 <= t <= 35, "track %d outside the D41 BAM" % t
        free = bam[4*t+1 + (s >> 3)] & (1 << (s & 7))
        if free:
            raise AssertionError("sector %d/%d marked free in BAM" % (t, s))


def make_d64(label=b'RELTEST', id_=b'00'):
    """Return a freshly formatted D41 image as bytearray."""
    img = bytearray(174848)
    # BAM at 18/0
    bam = bytearray(256)
    bam[0], bam[1] = 18, 1      # dir link
    bam[2] = 0x41               # DOS version 'A'
    bam[3] = 0x00               # single-sided
    # BAM entries: 18/0 (BAM) and 18/1 (directory) are in use
    def set_track(bam, track, free_count, freebits):
        base = 4*track
        bam[base] = free_count
        bam[base+1:base+4] = freebits.to_bytes(3, 'little')
    for t in range(1, 36):
        n = TRACK_SECTORS[t-1]
        if t == 18:
            set_track(bam, t, n-2, ((1 << n) - 1) & ~0x3)
        else:
            set_track(bam, t, n, (1 << n) - 1)
    # disk name, ID and DOS type
    bam[0x90:0xab] = b'\xa0' * 0x1b
    bam[0x90:0x90+len(label[:16])] = label[:16]
    bam[0xa2:0xa4] = id_[:2]
    bam[0xa5:0xa7] = b'2A'
    img[256*lba(18,0):256*lba(18,0)+256] = bam
    # empty dir sector at 18/1
    d = bytearray(256)
    d[0], d[1] = 0, 0xFF
    img[256*lba(18,1):256*lba(18,1)+256] = d
    return img

def alloc_sector(img):
    """Allocate the first free sector in the BAM and return its t/s."""
    o = 256*lba(18, 0)
    for t in range(1, 36):
        for s in range(TRACK_SECTORS[t-1]):
            if img[o + 4*t+1 + (s >> 3)] & (1 << (s & 7)):
                img[o + 4*t+1 + (s >> 3)] &= ~(1 << (s & 7))
                img[o + 4*t] -= 1
                return t, s
    raise RuntimeError("disk full")

def create_rel(img, name, reclen, records, insert_at=(18, 1, 0)):
    """Create a REL file per CBM DOS layout. Returns list of used sectors."""
    used = []
    def data_byte(pos):
        # empty record pattern: first byte of each record 0xFF, rest 0x00
        return 0xFF if (pos % reclen) == 0 else 0x00

    total = len(records) * reclen
    ndata = (total + 253) // 254
    # allocate data sectors first, then side sectors (per VICE add_sector order)
    data_ts = [alloc_sector(img) for _ in range(ndata)]
    used += data_ts
    nss = (ndata + 119) // 120
    ss_ts = [alloc_sector(img) for _ in range(nss)]
    used += ss_ts

    # data sectors
    blob = bytearray()
    for r in records:
        assert len(r) <= reclen
        blob += r + bytes(reclen - len(r))   # zero-fill remainder
    for i in range(ndata):
        sec = bytearray(256)
        chunk = blob[i*254:(i+1)*254]
        # empty records fill the space after the last written record
        sec[2:256] = chunk + bytes(data_byte((i*254)+j) for j in range(len(chunk), 254))
        if i+1 < ndata:
            sec[0], sec[1] = data_ts[i+1]
        else:
            # used bytes end with the last complete record in the sector
            sec[0] = 0
            sec[1] = 255 - (ndata*254) % reclen
        img[256*lba(*data_ts[i]):256*lba(*data_ts[i])+256] = sec

    # side sectors: numbers restart at 0 in every group of six and all
    # members of a group share the pointer table at bytes 4-15
    for g in range(nss):
        ss = bytearray(256)
        ss[1] = 16 + 2*min(120, ndata - g*120) - 1
        ss[2] = g % 6
        ss[3] = reclen
        head = g - g % 6
        for j in range(min(6, nss - head)):
            ss[4+2*j], ss[5+2*j] = ss_ts[head+j]
        for j in range(120):
            idx = g*120 + j
            if idx >= ndata:
                break
            ss[16+2*j], ss[17+2*j] = data_ts[idx]
        if g+1 < nss:
            ss[0], ss[1] = ss_ts[g+1]
        img[256*lba(*ss_ts[g]):256*lba(*ss_ts[g])+256] = ss

    # dir entry
    nm = name.upper().ljust(16, '\xa0').encode('latin-1')
    dt, ds, idx = insert_at
    o = 256*lba(dt, ds) + 2 + 32*idx
    e = bytearray(32)
    e[0] = 0x84
    e[1], e[2] = data_ts[0]
    e[3:19] = nm
    e[19], e[20] = ss_ts[0]
    e[21] = reclen
    e[28] = (ndata + nss) & 0xFF
    e[29] = (ndata + nss) >> 8
    img[o:o+32] = e
    return used


C1541_NOISE = ("OPENCBM", "disk image", "Unit 8", "formatting", "attached",
               "detached", "copying", "validating")


def run_c1541(binary, args):
    """Run c1541, return (returncode, output without its status chatter).
    A missing binary is reported as returncode 127."""
    try:
        res = subprocess.run([binary] + list(args), stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
    except OSError as e:
        return 127, "cannot run %s: %s" % (binary, e)
    out = res.stdout.decode("latin-1", "replace")
    lines = [l for l in out.splitlines() if not any(n in l for n in C1541_NOISE)]
    return res.returncode, "\n".join(lines)
