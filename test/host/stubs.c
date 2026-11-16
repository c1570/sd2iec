/* stubs.c: host implementations of the firmware environment for the
   d64ops REL test harness.

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
#include "ff.h"
#include "wrapops.h"
#include "d64ops.h"
#include "parser.h"
#include "stub.h"

/* ---------------- in-memory disk image ---------------- */

static uint8_t  *image_data;
static uint32_t  image_size;

void image_load(const char *filename) {
  FILE *f = fopen(filename, "rb");

  if (!f) {
    perror(filename);
    exit(1);
  }
  fseek(f, 0, SEEK_END);
  image_size = ftell(f);
  fseek(f, 0, SEEK_SET);
  free(image_data);
  image_data = malloc(image_size);
  if (fread(image_data, 1, image_size, f) != image_size) {
    perror(filename);
    exit(1);
  }
  fclose(f);

  /* d64_mount() reads the image size from the FatFs file handle and
     d64_open_rel() checks the write flag */
  partition[0].imagehandle.fsize = image_size;
  partition[0].imagehandle.flag  = FA_READ | FA_WRITE;
}

void image_save(const char *filename) {
  FILE *f = fopen(filename, "wb");
  if (!f) {
    perror(filename);
    exit(1);
  }
  fwrite(image_data, 1, image_size, f);
  fclose(f);
}

/* ---------------- image layer ---------------- */

uint8_t image_read(uint8_t part, uint32_t offset, void *buffer, uint16_t length) {
  (void)part;
  if (offset + length > image_size)
    return 2;
  memcpy(buffer, image_data + offset, length);
  return 0;
}

uint8_t image_write(uint8_t part, uint32_t offset, const void *buffer,
                    uint16_t length, uint8_t flush) {
  (void)part;
  (void)flush;
  if (!(partition[0].imagehandle.flag & FA_WRITE))
    return 2;
  if (offset + length > image_size)
    return 2;
  memcpy(image_data + offset, buffer, length);
  return 0;
}

/* The harness only works on the root directory of a mounted image */
uint8_t image_chdir(path_t *path, cbmdirent_t *dent, buffer_t *buf) {
  (void)dent;
  (void)buf;
  path->dir.dxx.track  = 18;
  path->dir.dxx.sector = 1;
  return 0;
}

void image_unmount(uint8_t part) {
  (void)part;
}

/* ---------------- error channel ---------------- */

/* The harness only inspects the error number */
uint8_t  current_error;
uint8_t  error_buffer[CONFIG_ERROR_BUFFER_SIZE];

void set_error_ts(uint8_t err, uint8_t track, uint8_t sector) {
  (void)track;
  (void)sector;
  current_error = err;
}

void set_error(uint8_t err) {
  set_error_ts(err, 0, 0);
}

/* ---------------- LED ---------------- */

volatile uint8_t led_state;

void update_leds(void) {
}

/* ---------------- partition table ---------------- */

partition_t partition[CONFIG_MAX_PARTITIONS];
uint8_t     max_part = CONFIG_MAX_PARTITIONS;

/* ---------------- error channel buffer ---------------- */

/* Simplified error channel: set_ok_message just resets the error state */
uint8_t set_ok_message(buffer_t *buf) {
  (void)buf;
  current_error = ERROR_OK;
  return 0;
}
