/* harness.c: host-side REL tests for the D64/D81/D71 image code in
   d64ops.c (compiled by including d64ops.c to access its internals) and
   for the FAT path via the separately compiled file layer.

   The bus protocol is emulated the same way iec.c drives the buffers:
   writes land in buf->data at buf->position, the refill callback runs
   when the buffer wrapped or on EOI for REL files, reads stop at
   lastused and records always end with EOI.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only. */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "dirent.h"
#include "errormsg.h"
#include "wrapops.h"
#include "d64ops.h"
#include "doscmd.h"
#include "fatops.h"
#include "stub.h"

/* Access the internals of the code under test; the FAT file layer
   (fatops.c, fileops.c, parser.c, utils.c, buffers.c) is compiled
   separately */
#include "d64ops.c"

/* ------------------------------------------------------------------ */
/*  Tiny test framework                                               */
/* ------------------------------------------------------------------ */

static int checks_run;
static int failures;

/* Abort a loop once the current test has failed, without being
   affected by failures of earlier tests */
#define TEST_FAILED(start) (failures != (start))

/* directory with the test images and the FAT root, from argv[1] */
static char data_dir[512];
/* working copy of the D64 image shared by the D64 tests */
static char work_path[512];

#define CHECK(cond, ...) do {                                         \
    checks_run++;                                                     \
    if (!(cond)) {                                                    \
      failures++;                                                     \
      printf("  FAIL %s:%d: ", __func__, __LINE__);                   \
      printf(__VA_ARGS__);                                            \
      printf("\n");                                                   \
    }                                                                 \
  } while (0)

#define CHECK_ERR(err) do {                                           \
    checks_run++;                                                     \
    if (current_error != (err)) {                                     \
      failures++;                                                     \
      printf("  FAIL %s:%d: expected error %d, got %d\n",             \
             __func__, __LINE__, (err), current_error);               \
    }                                                                 \
  } while (0)

/* ------------------------------------------------------------------ */
/*  Bus emulation (mirrors the iec.c write/read loops)                */
/* ------------------------------------------------------------------ */

/* Write len bytes, EOI terminates the transfer (and the record) */
static int bus_write(buffer_t *buf, const uint8_t *data, int len) {
  int i;

  for (i = 0; i < len; i++) {
    uint8_t eoi = (i == len - 1);

    if (buf->mustflush) {
      if (buf->refill(buf))
        return -1;
    }

    buf->data[buf->position] = data[i];
    mark_buffer_dirty(buf);
    if (buf->lastused < buf->position)
      buf->lastused = buf->position;
    buf->position++;

    if (buf->position == 0)
      buf->mustflush = 1;

    if (buf->recordlen && eoi) {
      buf->mustflush = 0;
      if (buf->refill(buf))
        return -1;
    }
  }
  return 0;
}

/* Read one record: returns the number of bytes delivered.
   Mirrors the iec.c talk loop: for REL files the refill callback runs
   after every buffer pass regardless of EOI. */
static int bus_read_record(buffer_t *buf, uint8_t *out, int max) {
  int n = 0;

  while (1) {
    uint8_t eoi;

    do {
      if (n < max)
        out[n++] = buf->data[buf->position];
      eoi = buf->sendeoi && buf->position == buf->lastused;
    } while (buf->position++ < buf->lastused);

    if (buf->refill(buf))
      return -1;

    if (eoi)
      break;
  }
  return n;
}

/* "P" command: record numbers and positions are 1-based */
static void rel_position(buffer_t *buf, uint16_t record, uint8_t pos) {
  if (pos > buf->recordlen) {
    set_error(ERROR_RECORD_OVERFLOW);
    return;
  }

  if (record)
    record--;
  if (pos)
    pos--;

  buf->seek(buf, record * (uint32_t)buf->recordlen, pos);
}

/* ------------------------------------------------------------------ */
/*  File handling helpers                                             */
/* ------------------------------------------------------------------ */

/* Directory location of the mounted image */
static void dir_ts(uint8_t *track, uint8_t *sector) {
  *track  = get_param(0, DIR_TRACK);
  *sector = get_param(0, DIR_START_SECTOR);
}

static void mount(const char *filename) {
  path_t path;

  /* release everything from the previous image */
  d64_unmount(0);
  free_multiple_buffers(FMB_ALL_CLEAN);
  /* d64_unmount leaves the static BAM buffer pointers set; reset them
     so the next d64_mount allocates fresh slots instead of reusing
     slots that free_multiple_buffers just released to file buffers */
  bam_buffer = NULL;
  bam_buffer2 = NULL;
  bam_refcount = 0;
  max_part = 1;

  memset(&partition[0], 0, sizeof(partition[0]));
  image_load(filename);

  path.part = 0;
  if (d64_mount(&path, (uint8_t *)filename)) {
    printf("mount of %s failed\n", filename);
    exit(1);
  }
}

static void copy_file(const char *src, const char *dst);

/* Copy the blank image to a named working image and return its path */
static void fresh_image(char *path, size_t size, const char *name) {
  char blank[512];

  snprintf(path,  size,  "%s/%s", data_dir, name);
  snprintf(blank, sizeof(blank), "%s/blank.d64", data_dir);
  copy_file(blank, path);
}

/* Mount a generated image read-only so that the tests leave it intact */
static void mount_fixture(const char *filename) {
  mount(filename);
  partition[0].imagehandle.flag &= (uint8_t)~FA_WRITE;
}

/* Compare a directory name with an ASCII name, ignoring the bit that
   distinguishes shifted PETSCII letters (0xc1-0xda) from ASCII ones */
static uint8_t ucasequal(const uint8_t *a, const char *b) {
  while (*a || *b) {
    if ((*a++ & 0x7f) != ((uint8_t)*b++ & 0x7f))
      return 0;
  }
  return 1;
}

/* Point path at the root directory of the mounted image */
static void root_path(path_t *path) {
  memset(path, 0, sizeof(*path));
  path->part = 0;
  dir_ts(&path->dir.dxx.track, &path->dir.dxx.sector);
}

/* Look up a REL file in the root directory. Sets up path and dent for
   d64_open_rel/d64_delete, returns 1 if found */
static int find_rel(path_t *path, cbmdirent_t *dent, const char *name) {
  dh_t dh;

  root_path(path);
  if (d64_opendir(&dh, path) != 0)
    return 0;
  while (d64_readdir(&dh, dent) == 0) {
    if ((dent->typeflags & TYPE_MASK) == TYPE_REL &&
        ucasequal(dent->name, name))
      return 1;
  }
  return 0;
}

/* Count the REL files in the root directory */
static int count_rel(void) {
  path_t path;
  cbmdirent_t dent;
  dh_t dh;
  int count = 0;

  root_path(&path);
  if (d64_opendir(&dh, &path) == 0) {
    while (d64_readdir(&dh, &dent) == 0) {
      if ((dent.typeflags & TYPE_MASK) == TYPE_REL)
        count++;
    }
  }
  return count;
}

/* Open a REL file, emulating the relevant parts of file_open */
static buffer_t *rel_open(const char *name, uint8_t recordlen, uint8_t create) {
  path_t path;
  cbmdirent_t dent;
  buffer_t *buf;
  int found;

  buf = alloc_buffer();
  if (!buf)
    return NULL;

  found = find_rel(&path, &dent, name);

  if (!found && !create) {
    set_error(ERROR_FILE_NOT_FOUND);
    free_buffer(buf);
    return NULL;
  }

  if (!found) {
    memset(&dent, 0, sizeof(dent));
    ustrncpy(dent.name, (uint8_t *)name, CBM_NAME_LENGTH);
  }

  d64_open_rel(&path, &dent, buf, recordlen, found);

  if (!buf->read && !buf->write) {
    free_buffer(buf);
    return NULL;
  }

  return buf;
}

static void rel_close(buffer_t *buf) {
  buf->cleanup(buf);
}

/* Write the BAM back and flush the image file */
static void commit_image(void) {
  d64_bam_commit();
  image_flush();
}

/* ------------------------------------------------------------------ */
/*  FAT-specific helpers                                              */
/* ------------------------------------------------------------------ */

static char fatroot_base[512];

static void fat_mount(void) {
  free_multiple_buffers(FMB_ALL_CLEAN);
  /* the BAM buffers belonged to the last D64 image and were just
     released, d64ops must not see them again */
  bam_buffer   = NULL;
  bam_buffer2  = NULL;
  bam_refcount = 0;
  snprintf(fatroot_base, sizeof(fatroot_base), "%s/fatroot", data_dir);
  fatshim_set_base(fatroot_base);
  memset(&partition[0], 0, sizeof(partition[0]));
  partition[0].fop = &fatops;
  f_mount(0, &partition[0].fatfs);
  current_part = 0;
  max_part = 1;
  set_error(ERROR_OK);
}

/* Start a test from an empty FAT root. */
static void fat_reset(void) {
  fat_mount();
  fatshim_clear_base();
  file_extension_mode = 0;
}

/* Set up the command channel buffer and run file_open for a REL file.
   The record length is the raw byte after the second comma, like on a
   real drive. Returns the opened buffer or NULL. */
static buffer_t *fat_open(const char *name, uint8_t recordlen,
                          uint8_t secondary) {
  snprintf((char *)command_buffer, CONFIG_COMMAND_BUFFER_SIZE + 2, "%s", name);
  {
    uint8_t len = strlen(name);
    command_buffer[len] = recordlen;
    command_length = len + 1;
  }
  file_open(secondary);
  return find_buffer(secondary);
}

static void fat_close(buffer_t *buf) {
  cleanup_and_free_buffer(buf);
}

/* Fill a buffer with test data. Zero bytes are replaced because the
   drive trims trailing zeros from a record on reads. */
static void fill_data(uint8_t *buf, int len, uint8_t seed) {
  int i;
  for (i = 0; i < len; i++)
    buf[i] = (uint8_t)(seed + i * 13) ? (uint8_t)(seed + i * 13) : 0x55;
}

/* ------------------------------------------------------------------ */
/*  Tests                                                             */
/* ------------------------------------------------------------------ */

#define RECLEN 20

static void t_create_and_write(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_create_and_write\n");
  mount(work_path);

  /* create */
  buf = rel_open("T1", RECLEN, 1);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  CHECK(buf->recordlen == RECLEN, "recordlen %d", buf->recordlen);
  CHECK(buf->write && buf->read, "not opened read/write");
  CHECK_ERR(ERROR_OK);

  /* a fresh record reads as a single 0xff byte */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1, "fresh record delivered %d bytes", n);
  CHECK(back[0] == 0xff, "fresh record byte %02x", back[0]);

  /* writing after the read targets record 2, go back to record 1 */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);

  /* write three full records */
  fill_data(data, RECLEN, 1);
  CHECK(bus_write(buf, data, RECLEN) == 0, "write record 1 failed");
  fill_data(data, RECLEN, 50);
  CHECK(bus_write(buf, data, RECLEN) == 0, "write record 2 failed");
  fill_data(data, RECLEN, 100);
  CHECK(bus_write(buf, data, RECLEN) == 0, "write record 3 failed");
  CHECK_ERR(ERROR_OK);

  /* reads positioned by the P command deliver the requested record */
  fill_data(data, RECLEN, 1);
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN, "record 1 delivered %d bytes", n);
  CHECK(memcmp(back, data, RECLEN) == 0, "record 1 mismatch");

  fill_data(data, RECLEN, 50);
  rel_position(buf, 2, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN, "record 2 delivered %d bytes", n);
  CHECK(memcmp(back, data, RECLEN) == 0, "record 2 mismatch");

  fill_data(data, RECLEN, 100);
  rel_position(buf, 3, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN, "record 3 delivered %d bytes", n);
  CHECK(memcmp(back, data, RECLEN) == 0, "record 3 mismatch");

  /* sequential reads now walk through the fresh records */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1, "fresh record 4 delivered %d bytes", n);
  CHECK(back[0] == 0xff, "fresh record 4 byte %02x", back[0]);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "fresh record 5 delivered %d bytes (%02x)",
        n, back[0]);

  /* until the end of the file is reached */
  for (n = 0; n < 7; n++)
    bus_read_record(buf, back, sizeof(back));
  /* advancing past the last record already flags the end of the file */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1, "eof delivered %d bytes", n);
  CHECK(back[0] == 13, "eof marker %02x", back[0]);
  CHECK_ERR(ERROR_RECORD_MISSING);

  rel_close(buf);
  commit_image();
}

static void t_partial_and_seek(void) {
  uint8_t data[254], back[300], patch[254];
  buffer_t *buf;
  int n;

  printf("t_partial_and_seek\n");
  mount(work_path);

  buf = rel_open("T1", RECLEN, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 3, "record_max %d", buf->pvt.d64.record_max);

  /* a short write to record 2 replaces the whole record */
  rel_position(buf, 2, 1);
  CHECK_ERR(ERROR_OK);
  fill_data(data, 5, 200);
  CHECK(bus_write(buf, data, 5) == 0, "partial write failed");

  /* read it back */
  rel_position(buf, 2, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 5, "partial record delivered %d bytes", n);
  CHECK(memcmp(back, data, 5) == 0, "partial record mismatch");

  /* writing at offset 3 overwrites and zero-fills the rest */
  rel_position(buf, 2, 4);
  CHECK_ERR(ERROR_OK);
  fill_data(patch, 3, 7);
  CHECK(bus_write(buf, patch, 3) == 0, "overwrite failed");

  rel_position(buf, 2, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 6, "modified record delivered %d bytes", n);
  fill_data(data, 5, 200);
  CHECK(memcmp(back, data, 3) == 0, "first bytes changed");
  CHECK(memcmp(back + 3, patch, 3) == 0, "overwritten bytes mismatch");
  /* n == 6 implies that the rest of the record was zero-filled */

  /* record overflow: position beyond the record length */
  rel_position(buf, 1, RECLEN + 1);
  CHECK_ERR(ERROR_RECORD_OVERFLOW);

  /* record 1 still intact */
  rel_position(buf, 1, 1);
  fill_data(data, RECLEN, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN && memcmp(back, data, RECLEN) == 0, "record 1 damaged");

  /* record 3 intact */
  rel_position(buf, 3, 1);
  fill_data(data, RECLEN, 100);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN && memcmp(back, data, RECLEN) == 0, "record 3 damaged");

  rel_close(buf);
  commit_image();
}

static void t_seek_past_eof(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_seek_past_eof\n");
  mount(work_path);

  buf = rel_open("T2", RECLEN, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  /* seek to a record that doesn't exist */
  rel_position(buf, 100, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);

  /* reading there keeps delivering the EOF marker without growing */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 13, "eof marker wrong");
  CHECK_ERR(ERROR_RECORD_MISSING);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 13, "eof marker wrong (2)");
  CHECK_ERR(ERROR_RECORD_MISSING);

  /* writing at that record creates it and fills the gap */
  fill_data(data, RECLEN, 42);
  CHECK(bus_write(buf, data, RECLEN) == 0, "write past eof failed");
  /* the drive still reports error 50 for the seek, like a 1541 */
  CHECK_ERR(ERROR_RECORD_MISSING);

  /* the gap records read as a single 0xff byte */
  rel_position(buf, 50, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "gap record wrong");

  /* the written record is there */
  rel_position(buf, 100, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN, "written record delivered %d bytes", n);
  fill_data(data, RECLEN, 42);
  CHECK(memcmp(back, data, RECLEN) == 0, "written record mismatch");

  /* record 1 still reads as fresh */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "record 1 not fresh after growth");

  rel_close(buf);
  commit_image();
}

static void t_spanning_records(void) {
  const uint8_t L = 200;
  uint8_t data[254], back[300], patch[254];
  buffer_t *buf;
  int n, i;

  printf("t_spanning_records\n");
  mount(work_path);

  /* 200-byte records cross sector boundaries (2*200 > 254) */
  buf = rel_open("T3", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  for (i = 0; i < 5; i++) {
    fill_data(data, L, (uint8_t)(i * 30 + 1));
    CHECK(bus_write(buf, data, L) == 0, "write record %d failed", i + 1);
  }
  CHECK_ERR(ERROR_OK);

  /* read them back in reverse order */
  for (i = 4; i >= 0; i--) {
    rel_position(buf, i + 1, 1);
    CHECK_ERR(ERROR_OK);
    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == L, "record %d delivered %d bytes", i + 1, n);
    fill_data(data, L, (uint8_t)(i * 30 + 1));
    CHECK(memcmp(back, data, L) == 0, "record %d mismatch", i + 1);
  }

  /* seek into the middle of a record that starts in another sector */
  rel_position(buf, 2, 180);
  CHECK_ERR(ERROR_OK);
  fill_data(patch, 10, 99);
  CHECK(bus_write(buf, patch, 10) == 0, "cross-sector write failed");

  rel_position(buf, 2, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 189, "cross-sector record delivered %d bytes", n);
  fill_data(data, L, 31);
  CHECK(memcmp(back, data, 179) == 0, "cross-sector record prefix changed");
  CHECK(memcmp(back + 179, patch, 10) == 0, "cross-sector write misplaced");

  rel_close(buf);
  commit_image();
}

static void t_full_sector_records(void) {
  const uint8_t L = 254;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n, i;

  printf("t_full_sector_records\n");
  mount(work_path);

  /* record length 254: every record fills a complete sector payload */
  buf = rel_open("T4", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  for (i = 0; i < 3; i++) {
    fill_data(data, L, (uint8_t)(10 * i + 2));
    CHECK(bus_write(buf, data, L) == 0, "write record %d failed", i + 1);
  }
  CHECK_ERR(ERROR_OK);

  for (i = 0; i < 3; i++) {
    rel_position(buf, i + 1, 1);
    CHECK_ERR(ERROR_OK);
    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == L, "record %d delivered %d bytes", i + 1, n);
    fill_data(data, L, (uint8_t)(10 * i + 2));
    CHECK(memcmp(back, data, L) == 0, "record %d mismatch", i + 1);
  }

  rel_close(buf);
  commit_image();
}

static void t_many_records(void) {
  const uint8_t L = 7;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n, i;
  int start = failures;

  printf("t_many_records\n");
  mount(work_path);

  /* write 300 records sequentially: requires growth across 9 sectors */
  buf = rel_open("T5", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  for (i = 0; i < 300; i++) {
    fill_data(data, L, (uint8_t)i);
    data[0] = (uint8_t)(i + 1);
    CHECK(bus_write(buf, data, L) == 0, "write record %d failed", i + 1);
    if (TEST_FAILED(start))
      break;
  }
  CHECK_ERR(ERROR_OK);
  CHECK(buf->pvt.d64.record_max >= 300, "record_max %d", buf->pvt.d64.record_max);

  /* read every third record back via seeks */
  for (i = 0; i < 300; i += 3) {
    rel_position(buf, i + 1, 1);
    CHECK_ERR(ERROR_OK);
    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == L, "record %d delivered %d bytes", i + 1, n);
    fill_data(data, L, (uint8_t)i);
    data[0] = (uint8_t)(i + 1);
    CHECK(memcmp(back, data, L) == 0, "record %d mismatch", i + 1);
    if (TEST_FAILED(start))
      break;
  }

  rel_close(buf);
  commit_image();
}

static void t_reopen_persistence(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_reopen_persistence\n");
  mount(work_path);

  /* T1 was written by the earlier tests */
  buf = rel_open("T1", RECLEN, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 3, "record_max %d", buf->pvt.d64.record_max);

  rel_position(buf, 1, 1);
  fill_data(data, RECLEN, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN && memcmp(back, data, RECLEN) == 0, "record 1 lost");

  /* append record 13 (the file holds 12 records after creation) */
  CHECK(buf->pvt.d64.record_max == 12, "record_max %d", buf->pvt.d64.record_max);
  rel_position(buf, 13, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, RECLEN, 150);
  CHECK(bus_write(buf, data, RECLEN) == 0, "append failed");
  /* the drive still reports the seek error, like a 1541 */
  CHECK_ERR(ERROR_RECORD_MISSING);
  rel_close(buf);

  /* reopen with a mismatched record length */
  buf = rel_open("T1", RECLEN + 1, 0);
  if (!buf) {
    CHECK(0, "reopen with mismatch failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_RECORD_MISSING);
  rel_close(buf);

  /* verify the appended record */
  buf = rel_open("T1", RECLEN, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 13, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN, "appended record delivered %d bytes", n);
  fill_data(data, RECLEN, 150);
  CHECK(memcmp(back, data, RECLEN) == 0, "appended record mismatch");
  rel_close(buf);

  commit_image();
}

static void t_delete(void) {
  buffer_t *buf;
  path_t path;
  cbmdirent_t dent;
  int found;

  printf("t_delete\n");
  mount(work_path);

  buf = rel_open("T2", RECLEN, 0);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  rel_close(buf);

  /* scratch it */
  found = find_rel(&path, &dent, "T2");
  CHECK(found, "T2 not found in directory");
  if (found)
    CHECK(d64_delete(&path, &dent) == 1, "delete failed");

  CHECK(rel_open("T2", RECLEN, 0) == NULL, "deleted file still opens");
  CHECK_ERR(ERROR_FILE_NOT_FOUND);

  commit_image();
}


static void t_seq_after_short(void) {
  const uint8_t L = 100;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n, i;
  int start = failures;

  printf("t_seq_after_short\n");
  mount(work_path);

  /* L=100 records span sector boundaries; records 1, 2 and 4 are long,
     record 3 is a short write in the middle of the file */
  buf = rel_open("S1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  fill_data(data, L, 10);
  CHECK(bus_write(buf, data, L) == 0, "write 1 failed");
  fill_data(data, L, 20);
  CHECK(bus_write(buf, data, L) == 0, "write 2 failed");
  fill_data(data, 5, 30);
  CHECK(bus_write(buf, data, 5) == 0, "write 3 failed");
  fill_data(data, L, 40);
  CHECK(bus_write(buf, data, L) == 0, "write 4 failed");

  /* writing more than a record holds drops the excess bytes and
     reports error 51, like a 1541. Record 5 exists but is fresh. */
  rel_position(buf, 5, 1);
  CHECK_ERR(ERROR_OK);
  fill_data(data, 150, 50);
  CHECK(bus_write(buf, data, 150) == 0, "overflow write failed");
  CHECK_ERR(ERROR_RECORD_OVERFLOW);

  /* sequential reads from record 1, no P command in between */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);

  for (i = 0; i < 5; i++) {
    int len = (i == 2) ? 5 : L;

    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == len, "sequential record %d delivered %d bytes", i + 1, n);
    fill_data(data, len, (uint8_t)((i == 0) ? 10 : (i == 1) ? 20 :
                                   (i == 2) ? 30 :
                                   (i == 3) ? 40 : 50));
    CHECK(memcmp(back, data, len) == 0, "sequential record %d mismatch", i + 1);
    if (TEST_FAILED(start))
      break;
  }

  /* the end of the file is reached, error 50 is only reported when
     the marker is transferred */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 13, "eof marker wrong");
  CHECK_ERR(ERROR_RECORD_MISSING);

  rel_close(buf);
  commit_image();
}

static void t_seq_from_open(void) {
  const uint8_t lens[2] = { 254, 127 };
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n, i, j;
  int start = failures;

  printf("t_seq_from_open\n");
  mount(work_path);

  /* records ending exactly at a sector boundary must be terminated
     with EOI when read sequentially right after opening */
  for (j = 0; j < 2; j++) {
    const uint8_t L = lens[j];

    buf = rel_open((j == 0) ? "S2" : "S3", L, 1);
    if (!buf) {
      CHECK(0, "create failed (error %d)", current_error);
      return;
    }

    for (i = 0; i < 3; i++) {
      fill_data(data, L, (uint8_t)(60 + i));
      CHECK(bus_write(buf, data, L) == 0, "write %d failed", i + 1);
    }

    rel_close(buf);
    commit_image();
    mount(work_path);
    buf = rel_open((j == 0) ? "S2" : "S3", L, 0);
    if (!buf) {
      CHECK(0, "reopen failed (error %d)", current_error);
      return;
    }

    for (i = 0; i < 3; i++) {
      n = bus_read_record(buf, back, sizeof(back));
      CHECK(n == L, "record %d delivered %d bytes", i + 1, n);
      fill_data(data, L, (uint8_t)(60 + i));
      CHECK(memcmp(back, data, L) == 0, "record %d mismatch", i + 1);
      if (TEST_FAILED(start))
        break;
    }

    rel_close(buf);
  }

  commit_image();
}

static void t_multiss(void) {
  const uint8_t L = 254;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_multiss\n");
  mount(work_path);

  /* 130 records of 254 bytes = 130 data sectors: two side sector
     blocks, the second one is member 1 of the first group */
  buf = rel_open("M1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 130, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, L, 77);
  CHECK(bus_write(buf, data, L) == 0, "write at 130 failed");
  /* the seek error stays until the next error, like on a 1541 */
  CHECK_ERR(ERROR_RECORD_MISSING);
  rel_close(buf);
  commit_image();
  mount(work_path);
  buf = rel_open("M1", L, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 130, "record_max %d", buf->pvt.d64.record_max);

  rel_position(buf, 130, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == L, "record 130 delivered %d bytes", n);
  fill_data(data, L, 77);
  CHECK(memcmp(back, data, L) == 0, "record 130 mismatch");

  /* the gap records read as fresh */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "record 1 not fresh after growth");

  rel_close(buf);
  commit_image();
}

static void t_d81_groups(const char *img) {
  const uint8_t L = 4;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_d81_groups\n");
  mount(img);

  /* 45720 records of 4 bytes fill 720 data sectors, the first six
     side sector blocks. Writing record 45721 needs a 721st data
     sector, whose side sector block is the first of a second group
     registered in the super side sector */
  buf = rel_open("G1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 45721, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, L, 88);
  CHECK(bus_write(buf, data, L) == 0, "write at 45721 failed");
  rel_close(buf);
  commit_image();

  mount(img);
  buf = rel_open("G1", L, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 45721, "record_max %d", buf->pvt.d64.record_max);

  rel_position(buf, 45721, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == L, "record 45721 delivered %d bytes", n);
  fill_data(data, L, 88);
  CHECK(memcmp(back, data, L) == 0, "record 45721 mismatch");

  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "record 1 not fresh after growth");

  rel_close(buf);
  commit_image();
}

static void t_freeblocks(void) {
  uint8_t data[254];
  buffer_t *buf;
  uint16_t base, after;

  printf("t_freeblocks\n");
  mount(work_path);

  base = d64_freeblocks(0);

  buf = rel_open("F1", 20, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  fill_data(data, 20, 1);
  CHECK(bus_write(buf, data, 20) == 0, "write failed");
  rel_position(buf, 30, 1);
  fill_data(data, 20, 2);
  CHECK(bus_write(buf, data, 20) == 0, "write 30 failed");
  rel_close(buf);

  after = d64_freeblocks(0);
  CHECK(after < base, "no blocks allocated (base %u after %u)", base, after);

  /* scratch it: the side sectors must be freed as well */
  {
    path_t path;
    cbmdirent_t dent;
    int found = find_rel(&path, &dent, "F1");

    CHECK(found, "F1 not found");
    if (found)
      CHECK(d64_delete(&path, &dent) == 1, "delete failed");
  }

  after = d64_freeblocks(0);
  CHECK(after == base, "scratch leaked blocks (base %u after %u)", base, after);

  commit_image();
}

static void t_multirel_open(void) {
  uint8_t back[300];
  buffer_t *buf;
  int n, r, i;
  char path[512];
  int start = failures;

  printf("t_multirel_open\n");
  snprintf(path, sizeof(path), "%s/multi.d64", data_dir);
  mount_fixture(path);

  buf = rel_open("MULTIREL", 17, 0);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 40, "record_max %d", buf->pvt.d64.record_max);

  for (r = 1; r <= 40; r++) {
    int len;
    uint8_t expect[17];

    for (i = 0; i < 17; i++)
      expect[i] = ((r - 1) * 11 + i * 3) & 0xff;
    for (i = 0; i < 17; i++) {
      if (((r - 1) + i) % 5 == 0)
        expect[i] = 0;
    }

    /* the record tail may be zero-filled by the creator */
    len = 17;
    while (len > 0 && expect[len - 1] == 0)
      len--;
    if (len == 0)
      len = 1;

    rel_position(buf, r, 1);
    CHECK_ERR(ERROR_OK);
    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == len, "record %d delivered %d bytes (expected %d)", r, n, len);
    CHECK(memcmp(back, expect, len) == 0, "record %d mismatch", r);
    if (TEST_FAILED(start))
      break;
  }

  rel_close(buf);
}

static void t_append_after_eof(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_append_after_eof\n");
  mount(work_path);

  buf = rel_open("A1", 20, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  /* fill the 12 records created with the file */
  {
    int i;
    fill_data(data, 20, 5);
    for (i = 0; i < 12; i++)
      CHECK(bus_write(buf, data, 20) == 0, "write %d failed", i + 1);
  }

  /* the file is full: a read without a P command hits the end */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 13, "eof marker wrong (%d bytes, %02x)",
        n, back[0]);
  CHECK_ERR(ERROR_RECORD_MISSING);

  /* writing now appends record 13 without corrupting anything */
  fill_data(data, 20, 90);
  CHECK(bus_write(buf, data, 20) == 0, "append failed");
  rel_close(buf);
  commit_image();

  mount(work_path);
  buf = rel_open("A1", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max >= 13, "record_max %d", buf->pvt.d64.record_max);

  rel_position(buf, 13, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20, "record 13 delivered %d bytes", n);
  fill_data(data, 20, 90);
  CHECK(memcmp(back, data, 20) == 0, "record 13 mismatch");
  rel_close(buf);

  commit_image();
}

static void t_read_after_write(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_read_after_write\n");
  mount(work_path);

  buf = rel_open("A2", 20, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  fill_data(data, 5, 42);
  CHECK(bus_write(buf, data, 5) == 0, "write failed");

  /* reading directly after the write delivers the next record */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 0xff, "read after write delivered %d bytes", n);

  /* and the written record is intact */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 5, "written record delivered %d bytes", n);
  fill_data(data, 5, 42);
  CHECK(memcmp(back, data, 5) == 0, "written record mismatch");

  rel_close(buf);
  commit_image();
}


static void t_scratch_dir(void) {
  uint8_t data[254];
  buffer_t *buf;
  path_t path;
  cbmdirent_t dent;
  uint8_t tt, ss;
  uint8_t link_before[2], link_after[2];
  int count, before, found, i;
  char name[3];
  char scratch[512];

  printf("t_scratch_dir\n");
  fresh_image(scratch, sizeof(scratch), "scratch.d64");
  mount(scratch);

  /* create ten REL files so the directory spans two sectors */
  before = count_rel();
  fill_data(data, 20, 1);
  for (i = 0; i < 10; i++) {
    name[0] = 'F';
    name[1] = '0' + i;
    name[2] = 0;
    buf = rel_open(name, 20, 1);
    if (!buf) {
      CHECK(0, "create %s failed (error %d)", name, current_error);
      return;
    }
    CHECK(bus_write(buf, data, 20) == 0, "write %s failed", name);
    rel_close(buf);
  }
  commit_image();
  mount(scratch);

  /* scratch the first file: F0..F7 sit in the first directory sector,
     F8/F9 in the second one. Its link to the next sector must survive. */
  dir_ts(&tt, &ss);
  CHECK(checked_read(0, tt, ss, link_before, 2, ERROR_ILLEGAL_TS_LINK) == 0,
        "directory read failed");
  CHECK(link_before[0] == tt && link_before[1] != 0,
        "directory not extended: link %d/%d", link_before[0], link_before[1]);

  found = find_rel(&path, &dent, "F0");
  CHECK(found, "F0 not found");
  if (found)
    CHECK(d64_delete(&path, &dent) == 1, "delete F0 failed");

  CHECK(checked_read(0, tt, ss, link_after, 2, ERROR_ILLEGAL_TS_LINK) == 0,
        "directory read failed");
  CHECK(link_after[0] == link_before[0] && link_after[1] == link_before[1],
        "directory link changed to %d/%d", link_after[0], link_after[1]);

  /* count the remaining REL entries */
  count = count_rel();
  CHECK(count == before + 9, "directory holds %d REL entries, expected %d",
        count, before + 9);

  /* the files in the second directory sector must still open */
  for (i = 8; i < 10; i++) {
    name[0] = 'F';
    name[1] = '0' + i;
    name[2] = 0;
    buf = rel_open(name, 20, 0);
    CHECK(buf != NULL, "%s lost by scratch", name);
    if (buf)
      rel_close(buf);
  }

  commit_image();
}

static void t_eoi_at_sector_end(void) {
  const uint8_t L = 100;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_eoi_at_sector_end\n");
  mount(work_path);

  /* record 3 starts at file offset 200, a 54-byte write ends exactly
     on the last byte of the sector; the following write must start a
     new record */
  buf = rel_open("E1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 3, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, 54, 11);
  CHECK(bus_write(buf, data, 54) == 0, "sector-end write failed");
  fill_data(data, 3, 99);
  CHECK(bus_write(buf, data, 3) == 0, "next write failed");
  rel_close(buf);
  commit_image();
  mount(work_path);
  buf = rel_open("E1", L, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 3, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 54, "record 3 delivered %d bytes", n);
  fill_data(data, 54, 11);
  CHECK(memcmp(back, data, 54) == 0, "record 3 mismatch");

  rel_position(buf, 4, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 3, "record 4 delivered %d bytes", n);
  fill_data(data, 3, 99);
  CHECK(memcmp(back, data, 3) == 0, "record 4 mismatch");

  rel_close(buf);
  commit_image();
}

static void t_spurious_record(void) {
  const uint8_t L = 100;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n, i;
  int start = failures;

  printf("t_spurious_record\n");
  mount(work_path);

  /* record 3 holds 54 data bytes, the rest of the record (which
     spans into the next sector) is zero-filled */
  buf = rel_open("E2", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  for (i = 0; i < 5; i++) {
    if (i == 2) {
      rel_position(buf, 3, 1);
      CHECK_ERR(ERROR_RECORD_MISSING);
      fill_data(data, 54, 71);
      CHECK(bus_write(buf, data, 54) == 0, "short write failed");
    } else {
      fill_data(data, L, (uint8_t)(i * 17));
      CHECK(bus_write(buf, data, L) == 0, "write %d failed", i + 1);
    }
  }
  rel_close(buf);
  commit_image();
  mount(work_path);
  buf = rel_open("E2", L, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }

  /* sequential read: no spurious one-byte record between 3 and 4 */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  for (i = 0; i < 5; i++) {
    int len = (i == 2) ? 54 : L;

    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == len, "record %d delivered %d bytes", i + 1, n);
    fill_data(data, len, (uint8_t)((i == 2) ? 71 : i * 17));
    CHECK(memcmp(back, data, len) == 0, "record %d mismatch", i + 1);
    if (TEST_FAILED(start))
      break;
  }

  rel_close(buf);
  commit_image();
}

static void t_overflow_then_write(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_overflow_then_write\n");
  mount(work_path);

  buf = rel_open("E3", 20, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  /* overflow record 1 */
  fill_data(data, 30, 7);
  CHECK(bus_write(buf, data, 30) == 0, "overflow write failed");
  CHECK_ERR(ERROR_RECORD_OVERFLOW);

  /* normal writes without a P command fill records 2 and 3 */
  fill_data(data, 20, 21);
  CHECK(bus_write(buf, data, 20) == 0, "write 2 failed");
  CHECK_ERR(ERROR_RECORD_OVERFLOW);
  fill_data(data, 20, 22);
  CHECK(bus_write(buf, data, 20) == 0, "write 3 failed");
  rel_close(buf);
  commit_image();
  mount(work_path);
  buf = rel_open("E3", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 2, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20, "record 2 delivered %d bytes", n);
  fill_data(data, 20, 21);
  CHECK(memcmp(back, data, 20) == 0, "record 2 mismatch");

  rel_position(buf, 3, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20, "record 3 delivered %d bytes", n);
  fill_data(data, 20, 22);
  CHECK(memcmp(back, data, 20) == 0, "record 3 mismatch");

  rel_close(buf);
  commit_image();
}

static void t_d71_limit(const char *img) {
  const uint8_t L = 254;
  uint8_t data[254];
  buffer_t *buf;

  printf("t_d71_limit\n");
  mount(img);

  buf = rel_open("D1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  /* record 750 needs a seventh side sector block: error 52 aborts
     the transfer */
  rel_position(buf, 750, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, L, 5);
  CHECK(bus_write(buf, data, L) != 0, "write past limit succeeded");
  CHECK_ERR(ERROR_FILE_TOO_LARGE);

  rel_close(buf);
  commit_image();
}


static void t_last_record_size(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_last_record_size\n");
  mount(work_path);

  /* a fresh L=20 file holds 12 records in one data sector */
  buf = rel_open("L1", 20, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  CHECK(buf->pvt.d64.record_max == 12, "record_max %d", buf->pvt.d64.record_max);

  /* rewriting the last record must not grow the file */
  rel_position(buf, 12, 1);
  CHECK_ERR(ERROR_OK);
  fill_data(data, 20, 33);
  CHECK(bus_write(buf, data, 20) == 0, "write 12 failed");
  CHECK(buf->pvt.d64.record_max == 12, "record_max grew to %d",
        buf->pvt.d64.record_max);
  rel_close(buf);
  commit_image();
  mount(work_path);

  /* the written record is there, reading past it reports error 50 */
  buf = rel_open("L1", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 12, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20, "record 12 delivered %d bytes", n);
  fill_data(data, 20, 33);
  CHECK(memcmp(back, data, 20) == 0, "record 12 mismatch");

  rel_position(buf, 13, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1 && back[0] == 13, "marker wrong");
  CHECK_ERR(ERROR_RECORD_MISSING);
  rel_close(buf);

  /* appending without a P command grows the file again */
  mount(work_path);
  buf = rel_open("L1", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 12, 1);
  fill_data(data, 20, 44);
  CHECK(bus_write(buf, data, 20) == 0, "rewrite failed");
  fill_data(data, 20, 55);
  CHECK(bus_write(buf, data, 20) == 0, "append failed");
  CHECK(buf->pvt.d64.record_max >= 13, "record_max %d", buf->pvt.d64.record_max);
  rel_close(buf);
  commit_image();

  mount(work_path);
  buf = rel_open("L1", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 13, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20, "appended record delivered %d bytes", n);
  fill_data(data, 20, 55);
  CHECK(memcmp(back, data, 20) == 0, "appended record mismatch");
  rel_close(buf);

  commit_image();
}

static void t_d71_last_record(const char *img) {
  const uint8_t L = 254;
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_d71_last_record\n");
  mount(img);

  /* grow to the 720 data sector limit: record 720 is the last one */
  buf = rel_open("D2", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 720, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  fill_data(data, L, 12);
  CHECK(bus_write(buf, data, L) == 0, "write 720 failed");
  /* the seek error stays until the next error, like on a 1541 */
  CHECK_ERR(ERROR_RECORD_MISSING);
  CHECK(buf->pvt.d64.record_max >= 720, "record_max %d", buf->pvt.d64.record_max);

  /* rewriting the last record at the limit must not hit error 52 */
  rel_position(buf, 720, 1);
  CHECK_ERR(ERROR_OK);
  fill_data(data, L, 13);
  CHECK(bus_write(buf, data, L) == 0, "rewrite at limit failed");
  rel_close(buf);
  commit_image();

  mount(img);
  buf = rel_open("D2", L, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 720, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == L, "record 720 delivered %d bytes", n);
  fill_data(data, L, 13);
  CHECK(memcmp(back, data, L) == 0, "record 720 mismatch");
  rel_close(buf);

  commit_image();
}

static void t_readonly(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;

  printf("t_readonly\n");
  mount(work_path);

  /* make the image read-only */
  partition[0].imagehandle.flag &= (uint8_t)~FA_WRITE;

  /* an existing file opens for reading only */
  buf = rel_open("L1", 20, 0);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  CHECK(buf->read && !buf->write, "not opened read-only");

  /* reading works */
  rel_position(buf, 12, 1);
  CHECK_ERR(ERROR_OK);
  {
    int n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == 20, "read delivered %d bytes", n);
    fill_data(data, 20, 44);
    CHECK(memcmp(back, data, 20) == 0, "record 12 mismatch");
  }

  /* seeking past the end reports error 50 without a growth request */
  rel_position(buf, 40, 1);
  CHECK_ERR(ERROR_RECORD_MISSING);
  CHECK(!(buf->pvt.d64.relflags & REL_FLAG_GROW),
        "growth requested on a read-only image");
  rel_close(buf);

  /* creating a file reports error 26 */
  CHECK(rel_open("NEWFILE", 20, 1) == NULL, "create on read-only succeeded");
  CHECK_ERR(ERROR_WRITE_PROTECT);

  /* the image is unchanged: reads still work after closing */
  partition[0].imagehandle.flag |= FA_WRITE;
  buf = rel_open("L1", 20, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 12, 1);
  fill_data(data, 20, 44);
  {
    int n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == 20 && memcmp(back, data, 20) == 0, "record 12 damaged");
  }
  rel_close(buf);
}

static void t_read_after_overflow(void) {
  const uint8_t L = 20;
  uint8_t data[254], big[300], back[300];
  buffer_t *buf;
  int n, i;
  int start = failures;

  printf("t_read_after_overflow\n");
  mount(work_path);

  /* five full records, then an overlong write to record 1 that wraps
     across sector boundaries */
  buf = rel_open("O1", L, 1);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  for (i = 0; i < 5; i++) {
    fill_data(data, L, (uint8_t)(i * 9 + 3));
    CHECK(bus_write(buf, data, L) == 0, "write %d failed", i + 1);
  }

  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  fill_data(big, 300, 60);
  CHECK(bus_write(buf, big, 300) == 0, "overflow write failed");
  CHECK_ERR(ERROR_RECORD_OVERFLOW);

  /* reading without a P command delivers record 2, not the marker */
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == L, "record 2 delivered %d bytes", n);
  fill_data(data, L, 12);
  CHECK(memcmp(back, data, L) == 0, "record 2 mismatch");

  /* the following records follow in sequence */
  for (i = 2; i < 5; i++) {
    n = bus_read_record(buf, back, sizeof(back));
    CHECK(n == L, "record %d delivered %d bytes", i + 1, n);
    fill_data(data, L, (uint8_t)(i * 9 + 3));
    CHECK(memcmp(back, data, L) == 0, "record %d mismatch", i + 1);
    if (TEST_FAILED(start))
      break;
  }

  /* record 1 holds the first 20 bytes of the overflowing write */
  rel_position(buf, 1, 1);
  CHECK_ERR(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == L, "record 1 delivered %d bytes", n);
  fill_data(data, L, 60);
  CHECK(memcmp(back, data, L) == 0, "record 1 mismatch");

  rel_close(buf);
  commit_image();
}

/* ------------------------------------------------------------------ */
/*  FAT REL tests (characterization tests marked QUIRK assert the     */
/*  current behaviour even where it differs from a 1541)              */
/* ------------------------------------------------------------------ */

static void t_fat_create_write_read(void) {
  uint8_t data[254], back[300];
  char freln[512];
  buffer_t *buf;
  int n;
  FILE *f;

  printf("t_fat_create_write_read\n");
  fat_reset();
  file_extension_mode = 1;
  snprintf(freln, sizeof(freln), "%s/frel.R00", fatroot_base);

  buf = fat_open("FREL,L,", 20, 2);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  CHECK(buf->recordlen == 20, "recordlen %d", buf->recordlen);
  CHECK(buf->write, "not opened for write");

  fill_data(data, 20, 1);
  CHECK(bus_write(buf, data, 20) == 0, "write 1 failed");
  fill_data(data, 10, 2);
  CHECK(bus_write(buf, data, 10) == 0, "write 2 failed");
  fill_data(data, 20, 3);
  CHECK(bus_write(buf, data, 20) == 0, "write 3 failed");

  fat_close(buf);

  /* QUIRK: a short written record is zero-filled to the record
     length; the file holds the 26-byte header plus three records */
  f = fopen(freln, "rb");
  CHECK(f != NULL, "host file frel.R00 missing");
  if (f) {
    uint8_t raw[100];
    size_t got = fread(raw, 1, sizeof(raw), f);
    fclose(f);
    CHECK(got == 86, "file size %d, expected 86", (int)got);
    CHECK(memcmp(raw, "C64File", 7) == 0, "marker mismatch");
    CHECK(raw[25] == 20, "header record length %d", raw[25]);
    fill_data(data, 20, 1);
    CHECK(memcmp(raw + 26, data, 20) == 0, "record 1 mismatch");
    fill_data(data, 10, 2);
    CHECK(memcmp(raw + 46, data, 10) == 0, "record 2 mismatch");
    {
      static const uint8_t zeros[10];
      CHECK(memcmp(raw + 56, zeros, 10) == 0, "record 2 not zero-filled");
    }
    fill_data(data, 20, 3);
    CHECK(memcmp(raw + 66, data, 20) == 0, "record 3 mismatch");
  }

  /* reopen and read back */
  fat_mount();
  buf = fat_open("FREL,L,", 20, 2);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  rel_position(buf, 1, 1);
  fill_data(data, 20, 1);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20 && memcmp(back, data, 20) == 0, "record 1 readback");

  rel_position(buf, 2, 1);
  fill_data(data, 10, 2);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 10 && memcmp(back, data, 10) == 0, "record 2 readback");

  /* sequential read advances to record 3 */
  fill_data(data, 20, 3);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20 && memcmp(back, data, 20) == 0, "record 3 readback");

  /* QUIRK: reading a record past the end of the file delivers one
     stale byte, and the refill towards the next record latches
     ERROR_RECORD_MISSING (the D64 path reports it on the seek) */
  rel_position(buf, 4, 1);
  set_error(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1, "past-eof delivered %d bytes", n);
  CHECK_ERR(ERROR_RECORD_MISSING);

  fat_close(buf);
  file_extension_mode = 0;
}

/* QUIRK: with extension mode 0 a REL file is stored without a type
   marker, so reopening it by name reports FILE EXISTS (the scan lists
   it as PRG) instead of reopening as REL. */
static void t_fat_raw_exists(void) {
  uint8_t data[254];
  char frawln[512];
  buffer_t *buf;
  FILE *f;

  printf("t_fat_raw_exists\n");
  fat_reset();
  snprintf(frawln, sizeof(frawln), "%s/fraw", fatroot_base);

  buf = fat_open("FRAW,L,", 20, 2);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  fill_data(data, 20, 12);
  CHECK(bus_write(buf, data, 20) == 0, "write failed");
  fat_close(buf);

  /* the raw file holds the record length byte plus one record */
  f = fopen(frawln, "rb");
  CHECK(f != NULL, "host file fraw missing");
  if (f) {
    uint8_t raw[32];
    size_t got = fread(raw, 1, sizeof(raw), f);
    fclose(f);
    CHECK(got == 21, "file size %d, expected 21", (int)got);
    CHECK(raw[0] == 20, "record length byte %d", raw[0]);
    fill_data(data, 20, 12);
    CHECK(memcmp(raw + 1, data, 20) == 0, "record 1 mismatch");
  }

  fat_mount();
  buf = fat_open("FRAW,L,", 20, 2);
  if (!buf) {
    CHECK_ERR(ERROR_FILE_EXISTS);
    return;
  }
  CHECK(0, "raw reopen unexpectedly succeeded");
  fat_close(buf);
}

static void t_fat_reopen_mismatch(void) {
  uint8_t data[254], back[300];
  buffer_t *buf;
  int n;

  printf("t_fat_reopen_mismatch\n");
  fat_reset();
  file_extension_mode = 1;

  buf = fat_open("FMIS,L,", 20, 2);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  fill_data(data, 20, 66);
  CHECK(bus_write(buf, data, 20) == 0, "write failed");
  fat_close(buf);

  fat_mount();

  /* QUIRK: reopening with a different record length reports error 50
     but uses the REQUESTED record length, not the stored one */
  buf = fat_open("FMIS,L,", 25, 2);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_RECORD_MISSING);
  CHECK(buf->recordlen == 25, "recordlen %d", buf->recordlen);

  rel_position(buf, 1, 1);
  fill_data(data, 20, 66);
  n = bus_read_record(buf, back, sizeof(back));
  /* the stored record is 20 bytes, so only 20 are delivered even
     though the buffer uses the requested record length of 25 */
  CHECK(n == 20 && memcmp(back, data, 20) == 0, "record 1 read");
  fat_close(buf);
  file_extension_mode = 0;
}

static void t_fat_spill(void) {
  uint8_t data[254], back[300];
  char fspillln[512];
  buffer_t *buf;
  FILE *f;
  int n;

  printf("t_fat_spill\n");
  fat_reset();
  file_extension_mode = 1;
  snprintf(fspillln, sizeof(fspillln), "%s/fspill.R00", fatroot_base);

  buf = fat_open("FSPILL,L,", 20, 2);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }

  /* QUIRK: unlike the D64 path, an overlong FAT REL write reports no
     overflow error. write_data truncates the transfer to one record,
     so the trailing 10 bytes are silently dropped. */
  fill_data(data, 30, 8);
  CHECK(bus_write(buf, data, 30) == 0, "overlong write failed");
  CHECK_ERR(ERROR_OK);
  fat_close(buf);

  f = fopen(fspillln, "rb");
  CHECK(f != NULL, "host file fspill.R00 missing");
  if (f) {
    uint8_t raw[100];
    size_t got = fread(raw, 1, sizeof(raw), f);
    fclose(f);
    CHECK(got == 46, "file size %d, expected 46 (header + one record)", (int)got);
    fill_data(data, 20, 8);
    CHECK(memcmp(raw + 26, data, 20) == 0, "record 1 mismatch");
  }

  fat_mount();
  buf = fat_open("FSPILL,L,", 20, 2);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }

  rel_position(buf, 1, 1);
  fill_data(data, 20, 8);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 20 && memcmp(back, data, 20) == 0, "record 1 mismatch");

  /* the dropped bytes left record 2 absent: reading past EOF delivers
     one stale byte and the refill towards the next record latches
     ERROR_RECORD_MISSING */
  rel_position(buf, 2, 1);
  set_error(ERROR_OK);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 1, "record 2 delivered %d bytes", n);
  CHECK_ERR(ERROR_RECORD_MISSING);

  fat_close(buf);
  file_extension_mode = 0;
}

/* The x00 detection identifies an R00 file by the CBM name in its
   header, independent of the host file name */
static void t_fat_x00(void) {
  uint8_t data[254], back[300];
  char xreln[512], renamed[512];
  buffer_t *buf;
  int n;
  FILE *f;
  uint8_t header[26];

  printf("t_fat_x00\n");
  fat_reset();
  snprintf(xreln, sizeof(xreln), "%s/xrel.R00", fatroot_base);
  snprintf(renamed, sizeof(renamed), "%s/other.r00", fatroot_base);

  /* extension mode 1: the REL file is stored as an R00 file with the
     P00-style header and the record length at offset 25 */
  file_extension_mode = 1;

  buf = fat_open("XREL,L,", 15, 2);
  if (!buf) {
    CHECK(0, "create failed (error %d)", current_error);
    return;
  }
  fill_data(data, 15, 5);
  CHECK(bus_write(buf, data, 15) == 0, "write failed");
  fat_close(buf);

  f = fopen(xreln, "rb");
  CHECK(f != NULL, "host file xrel.R00 missing");
  if (f) {
    size_t got = fread(header, 1, sizeof(header), f);
    fclose(f);
    CHECK(got == 26, "header size %d", (int)got);
    CHECK(memcmp(header, "C64File", 7) == 0, "marker mismatch");
    CHECK(memcmp(header + 8, "XREL", 5) == 0, "internal name mismatch");
    CHECK(header[25] == 15, "record length byte %d", header[25]);
  }

  /* reopen under a different host name: the scan matches the file by
     the internal name in the header */
  /* parenthesized: the firmware headers define a rename() macro */
  CHECK((rename)(xreln, renamed) == 0, "rename failed");
  fat_mount();
  buf = fat_open("XREL,L,", 15, 2);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK(buf->recordlen == 15, "recordlen %d", buf->recordlen);

  rel_position(buf, 1, 1);
  fill_data(data, 15, 5);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 15 && memcmp(back, data, 15) == 0, "record 1 readback");
  fat_close(buf);

  file_extension_mode = 0;
}

static void t_c1541_image(const char *image) {
  uint8_t back[300], expect[50];
  buffer_t *buf;
  int n, i;

  printf("t_c1541_image\n");
  mount_fixture(image);

  buf = rel_open("MYREL", 50, 0);
  if (!buf) {
    CHECK(0, "c1541-created file failed to open (error %d)", current_error);
    return;
  }

  /* record 1 holds the data written by genimages.py */
  for (i = 0; i < 50; i++)
    expect[i] = (uint8_t)((i * 7 + 3) & 0xff);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == 50, "record delivered %d bytes", n);
  CHECK(memcmp(back, expect, 50) == 0, "record content mismatch");

  rel_close(buf);
}

/* Create a REL file and strip its only data sector, leaving a file that
   consists of the directory entry and an empty side sector block. CBM
   DOS never writes such a file, but other tools may. */
static int make_dataless_rel(const char *name, uint8_t recordlen) {
  uint8_t ss[256];
  path_t path;
  cbmdirent_t dent;
  buffer_t *buf;
  uint8_t dt, ds, sst, sss;

  buf = rel_open(name, recordlen, 1);
  if (!buf)
    return 0;
  rel_close(buf);

  if (!find_rel(&path, &dent, name) ||
      read_entry(0, &dent.pvt.dxx.dh, ops_scratch))
    return 0;

  dt  = ops_scratch[DIR_OFS_TRACK];
  ds  = ops_scratch[DIR_OFS_SECTOR];
  sst = ops_scratch[DIR_OFS_SIDESECTOR_TRACK];
  sss = ops_scratch[DIR_OFS_SIDESECTOR_SECTOR];

  /* no data sector: an empty pointer table and no data chain */
  if (image_read(0, sector_offset(0, sst, sss), ss, 256))
    return 0;
  ss[SS_OFS_NEXT_SECTOR] = SS_OFS_POINTER - 1;
  ss[SS_OFS_POINTER]     = 0;
  ss[SS_OFS_POINTER + 1] = 0;
  if (image_write(0, sector_offset(0, sst, sss), ss, 256, 1))
    return 0;

  free_sector(0, dt, ds);

  ops_scratch[DIR_OFS_TRACK]    = 0;
  ops_scratch[DIR_OFS_SECTOR]   = 0;
  ops_scratch[DIR_OFS_SIZE_LOW] = 1;
  ops_scratch[DIR_OFS_SIZE_HI]  = 0;
  if (write_entry(0, &dent.pvt.dxx.dh, ops_scratch, 1))
    return 0;

  commit_image();
  return 1;
}

/* Opening a REL file without data sectors, also with a mismatched
   record length */
static void t_dataless_open(void) {
  char path[512];
  buffer_t *buf;

  printf("t_dataless_open\n");
  fresh_image(path, sizeof(path), "dataless.d64");
  mount(path);
  CHECK(make_dataless_rel("Z0", RECLEN), "setup failed (error %d)", current_error);

  /* an empty file opens without an error */
  set_error(ERROR_OK);
  buf = rel_open("Z0", RECLEN, 0);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_OK);
  CHECK(buf->pvt.d64.record_max == 0, "record_max %d", buf->pvt.d64.record_max);
  rel_close(buf);

  /* opening without a record length is not a mismatch */
  set_error(ERROR_OK);
  buf = rel_open("Z0", 0, 0);
  if (!buf) {
    CHECK(0, "open without length failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_OK);
  CHECK(buf->recordlen == RECLEN, "recordlen %d", buf->recordlen);
  rel_close(buf);

  /* a mismatched record length reports error 50 on the empty file too */
  set_error(ERROR_OK);
  buf = rel_open("Z0", RECLEN + 1, 0);
  if (!buf) {
    CHECK(0, "open with mismatch failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_RECORD_MISSING);
  CHECK(buf->recordlen == RECLEN, "recordlen %d", buf->recordlen);
  CHECK(buf->pvt.d64.record_max == 0, "record_max %d", buf->pvt.d64.record_max);
  rel_close(buf);
}

/* The first write to a REL file without data sectors allocates one */
static void t_dataless_grow(void) {
  char path[512];
  uint8_t data[254], back[300];
  path_t p;
  cbmdirent_t dent;
  buffer_t *buf;
  uint8_t ss[256];
  uint16_t before, after;
  int n;

  printf("t_dataless_grow\n");
  fresh_image(path, sizeof(path), "dataless.d64");
  mount(path);
  CHECK(make_dataless_rel("Z1", RECLEN), "setup failed (error %d)", current_error);
  before = d64_freeblocks(0);

  buf = rel_open("Z1", RECLEN, 0);
  if (!buf) {
    CHECK(0, "open failed (error %d)", current_error);
    return;
  }
  fill_data(data, RECLEN, 77);
  CHECK(bus_write(buf, data, RECLEN) == 0, "write failed (error %d)", current_error);
  CHECK(buf->pvt.d64.record_max == 254 / RECLEN,
        "record_max %d", buf->pvt.d64.record_max);
  rel_close(buf);
  commit_image();

  after = d64_freeblocks(0);
  CHECK(before - after == 1, "allocated %d blocks", before - after);

  /* reopen: the record survives and the file holds one sector of records */
  buf = rel_open("Z1", RECLEN, 0);
  if (!buf) {
    CHECK(0, "reopen failed (error %d)", current_error);
    return;
  }
  CHECK_ERR(ERROR_OK);
  CHECK(buf->pvt.d64.record_max == 254 / RECLEN,
        "record_max %d after reopen", buf->pvt.d64.record_max);
  n = bus_read_record(buf, back, sizeof(back));
  CHECK(n == RECLEN && memcmp(back, data, RECLEN) == 0, "record 1 lost");
  rel_close(buf);

  /* the directory entry and the side sector agree on the new sector,
     which is allocated in the BAM */
  if (!find_rel(&p, &dent, "Z1") ||
      read_entry(0, &dent.pvt.dxx.dh, ops_scratch)) {
    CHECK(0, "directory entry not found");
    return;
  }
  CHECK(ops_scratch[DIR_OFS_SIZE_LOW] == 2 && ops_scratch[DIR_OFS_SIZE_HI] == 0,
        "directory size %d", ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI]);
  CHECK(image_read(0, sector_offset(0, ops_scratch[DIR_OFS_SIDESECTOR_TRACK],
                                    ops_scratch[DIR_OFS_SIDESECTOR_SECTOR]),
                   ss, 256) == 0, "side sector read failed");
  CHECK(ss[SS_OFS_NEXT_SECTOR] == SS_OFS_POINTER + 1,
        "side sector holds %d entries", (ss[SS_OFS_NEXT_SECTOR] + 1 - SS_OFS_POINTER) / 2);
  CHECK(ss[SS_OFS_POINTER] != 0 && is_free(0, ss[SS_OFS_POINTER], ss[SS_OFS_POINTER + 1]) == 0,
        "data sector %d/%d not allocated", ss[SS_OFS_POINTER], ss[SS_OFS_POINTER + 1]);
  CHECK(ops_scratch[DIR_OFS_TRACK]  == ss[SS_OFS_POINTER] &&
        ops_scratch[DIR_OFS_SECTOR] == ss[SS_OFS_POINTER + 1],
        "directory entry points to %d/%d, side sector to %d/%d",
        ops_scratch[DIR_OFS_TRACK], ops_scratch[DIR_OFS_SECTOR],
        ss[SS_OFS_POINTER], ss[SS_OFS_POINTER + 1]);

  /* scratch follows the data chain from the directory entry and frees
     the data and side sector blocks */
  CHECK(d64_delete(&p, &dent) == 1, "delete failed (error %d)", current_error);
  commit_image();
  CHECK(d64_freeblocks(0) == before + 1,
        "free blocks %d after scratch, expected %d", d64_freeblocks(0), before + 1);
}

/* ------------------------------------------------------------------ */

static void copy_file(const char *src, const char *dst) {
  FILE *in, *out;
  char buf[4096];
  size_t n;

  in = fopen(src, "rb");
  out = fopen(dst, "wb");
  if (!in || !out) {
    perror("copy");
    exit(1);
  }
  while ((n = fread(buf, 1, sizeof(buf), in)) > 0)
    fwrite(buf, 1, n, out);
  fclose(in);
  fclose(out);
}

int main(int argc, char **argv) {
  char blank[512];
  char ref[512];

  buffers_init();
  set_error(ERROR_OK);

  snprintf(data_dir, sizeof(data_dir), "%s", argc > 1 ? argv[1] : "testdata");
  snprintf(blank, sizeof(blank), "%s/blank.d64", data_dir);
  snprintf(ref,   sizeof(ref),   "%s/c1541.d64", data_dir);
  snprintf(work_path, sizeof(work_path), "%s/work.d64", data_dir);

  /* the D64 tests share one working copy of the blank image, later
     tests and validate.py build on the files written by earlier ones */
  copy_file(blank, work_path);

  t_create_and_write();
  t_partial_and_seek();
  t_seek_past_eof();
  t_spanning_records();
  t_full_sector_records();
  t_many_records();
  t_reopen_persistence();
  t_delete();
  t_seq_after_short();
  t_seq_from_open();
  t_multiss();
  t_freeblocks();
  t_append_after_eof();
  t_read_after_write();
  t_scratch_dir();
  t_eoi_at_sector_end();
  t_spurious_record();
  t_overflow_then_write();
  t_last_record_size();
  t_readonly();
  t_read_after_overflow();
  t_dataless_open();
  t_dataless_grow();
  t_fat_create_write_read();
  t_fat_reopen_mismatch();
  t_fat_spill();
  t_fat_raw_exists();
  t_fat_x00();

  /* open the python-created multi-record image */
  t_multirel_open();

  /* D81 exercises the super side sector paths */
  {
    char blank81[512], work81[512];
    snprintf(blank81, sizeof(blank81), "%s/blank.d81", data_dir);
    snprintf(work81,  sizeof(work81),  "%s/work.d81", data_dir);
    copy_file(blank81, work81);
    t_d81_groups(work81);
  }

  /* D71: growth limit and last-record handling at the limit */
  {
    char blank71[512], work71[512];
    snprintf(blank71, sizeof(blank71), "%s/blank.d71", data_dir);
    snprintf(work71,  sizeof(work71),  "%s/work.d71", data_dir);
    copy_file(blank71, work71);
    t_d71_limit(work71);
    copy_file(blank71, work71);
    t_d71_last_record(work71);
  }

  /* cross-check against an image created by the c1541 tool */
  t_c1541_image(ref);

  printf("\n%d checks, %d failures\n", checks_run, failures);
  return failures ? 1 : 0;
}
