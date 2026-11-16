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
#include "flags.h"
#include "ff.h"
#include "wrapops.h"
#include "d64ops.h"
#include "parser.h"
#include "stub.h"

/* ---------------- image access ---------------- */
/* Images are real host files accessed through the FatFs shim, the
   same way fatops.c accesses them on the firmware. Writes go straight
   to the file. */

static uint8_t image_open;

void image_load(const char *filename) {
  if (image_open)
    f_close(&partition[0].imagehandle);
  image_open = 0;

  /* host paths are used as given */
  fatshim_set_base("");
  if (f_open(&partition[0].fatfs, &partition[0].imagehandle,
             (const UCHAR *)filename, FA_READ | FA_WRITE)) {
    fprintf(stderr, "cannot open image %s\n", filename);
    exit(1);
  }
  image_open = 1;
}

void image_flush(void) {
  if (image_open && f_sync(&partition[0].imagehandle) != FR_OK) {
    fprintf(stderr, "image flush failed\n");
    exit(1);
  }
}

/* ---------------- image layer ---------------- */

/* image_read/image_write/image_chdir/image_unmount come from fatops.c */

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

/* ---------------- global flags ---------------- */

uint8_t globalflags;

/* set_changelist stub (normally diskchange.c): the harness has no swap
   lists, fatops.c only calls it with an empty file name */
void set_changelist(path_t *path, uint8_t *filename) {
  (void)path;
  (void)filename;
  globalflags &= (uint8_t)~AUTOSWAP_ACTIVE;
}

/* ---------------- command channel buffer ---------------- */
/* normally defined in doscmd.c, which the harness does not compile */
uint8_t command_buffer[CONFIG_COMMAND_BUFFER_SIZE+2];
uint8_t command_length;

/* date matching globals (normally in doscmd.c), the harness never
   matches on dates */
date_t date_match_start;
date_t date_match_end;

/* Simplified error channel: set_ok_message just resets the error state */
uint8_t set_ok_message(buffer_t *buf) {
  (void)buf;
  current_error = ERROR_OK;
  return 0;
}
