/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2026  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   d64ops.c: D64 operations

*/

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "dirent.h"
#include "errormsg.h"
#include "fatops.h"
#include "ff.h"
#include "parser.h"
#include "progmem.h"
#include "rtc.h"
#include "ustring.h"
#include "wrapops.h"
#include "d64ops.h"

#define D41_ERROR_OFFSET 174848
#define D71_ERROR_OFFSET 349696
#define D81_ERROR_OFFSET 819200

#define D41_BAM_TRACK           18
#define D41_BAM_SECTOR          0
#define D41_BAM_BYTES_PER_TRACK 4
#define D41_BAM_BITFIELD_BYTES  3

#define D81_BAM_TRACK           40
#define D81_BAM_SECTOR1         1
#define D81_BAM_SECTOR2         2
#define D81_BAM_OFFSET          10
#define D81_BAM_BYTES_PER_TRACK 6
#define D81_BAM_BITFIELD_BYTES  5

#define D71_BAM2_TRACK  53
#define D71_BAM2_SECTOR 0
#define D71_BAM2_BYTES_PER_TRACK 3
#define D71_BAM_COUNTER2OFFSET   0xdd

#define DNP_BAM_TRACK                     1
#define DNP_BAM_SECTOR                    2
#define DNP_BAM_LAST_TRACK_OFS            8
#define DNP_ROOTDIR_SECTOR               34 // only for formatting!
#define DNP_BAM_BYTES_PER_TRACK          32
#define DNP_DIRHEADER_DIR_TRACK           0
#define DNP_DIRHEADER_DIR_SECTOR          1
#define DNP_DIRHEADER_ROOTHDR_TRACK      32
#define DNP_DIRHEADER_ROOTHDR_SECTOR     33
#define DNP_DIRHEADER_PARENTHDR_TRACK    34
#define DNP_DIRHEADER_PARENTHDR_SECTOR   35
#define DNP_DIRHEADER_PARENTENTRY_TRACK  36
#define DNP_DIRHEADER_PARENTENTRY_SECTOR 37
#define DNP_DIRHEADER_PARENTENTRY_OFFSET 38
#define DNP_LABEL_OFFSET                  4
#define DNP_LABEL_AREA_SIZE             (28-4+1)
#define DNP_ID_OFFSET                    22

/* used for error info only */
#define MAX_SECTORS_PER_TRACK 40

typedef enum { BAM_BITFIELD, BAM_FREECOUNT } bamdata_t;

struct {
  uint8_t part;
  uint8_t track;
  uint8_t errors[MAX_SECTORS_PER_TRACK];
} errorcache;

static buffer_t *bam_buffer;  // recently-used buffer
static buffer_t *bam_buffer2; // secondary buffer
static uint8_t   bam_refcount;

/* ------------------------------------------------------------------------- */
/*  Forward declarations                                                     */
/* ------------------------------------------------------------------------- */

static uint8_t d64_opendir(dh_t *dh, path_t *path);

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_d71_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_d81_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_dnp_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);

/* ------------------------------------------------------------------------- */
/*  Utility functions                                                        */
/* ------------------------------------------------------------------------- */

static const PROGMEM struct param_s d41param = {
  18, 1, 35, 0x90, 0xa2, 10, 3, format_d41_image
};

static const PROGMEM struct param_s d71param = {
  18, 1, 70, 0x90, 0xa2, 6, 3, format_d71_image
};

static const PROGMEM struct param_s d81param = {
  40, 3, 80, 0x04, 0x16, 1, 1, format_d81_image
};

static const PROGMEM struct param_s dnpparam = {
  1, 1, 0, DNP_LABEL_OFFSET, DNP_ID_OFFSET, 1, 1, format_dnp_image
};

/**
 * get_param - get a format-dependent parameter
 * @part : partition number
 * @param: parameter to retrieve
 *
 * Returns a parameter that is format-dependent for the image type mounted
 * on the specified partition.
 */
static uint8_t get_param(uint8_t part, param_t param) {
  return *(((uint8_t *)&partition[part].d64data)+param);
}

/**
 * sector_lba - Transform a track/sector pair into a LBA sector number
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an LBA-style sector number for a given track/sector pair.
 */
/* This version used the least code of all tested variants. */
static uint16_t sector_lba(uint8_t part, uint8_t track, const uint8_t sector) {
  uint16_t offset = 0;

  track--; /* Track numbers are 1-based */

  switch (partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D41:
  case D64_TYPE_D71:
  default:
    if (track >= 35) {
      offset = 683;
      track -= 35;
    }
    if (track < 17)
      return track*21 + sector + offset;
    if (track < 24)
      return 17*21 + (track-17)*19 + sector + offset;
    if (track < 30)
      return 17*21 + 7*19 + (track-24)*18 + sector + offset;
    return 17*21 + 7*19 + 6*18 + (track-30)*17 + sector + offset;

  case D64_TYPE_D81:
    return track*40 + sector;

  case D64_TYPE_DNP:
    return track*256 + sector;
  }
}

/**
 * sector_offset - Transform a track/sector pair into a D64 offset
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an offset into a D64 file from a track and sector number.
 */
static uint32_t sector_offset(uint8_t part, uint8_t track, const uint8_t sector) {
  return 256L * sector_lba(part,track,sector);
}

/**
 * sectors_per_track - number of sectors on given track
 * @part : partition number
 * @track: Track number
 *
 * This function returns the number of sectors on the given track
 * of a 1541/71/81 disk. Invalid track numbers will return invalid results.
 */
static uint16_t sectors_per_track(uint8_t part, uint8_t track) {
  switch (partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D41:
  case D64_TYPE_D71:
  default:
    if (track > 35)
      track -= 35;
    if (track < 18)
      return 21;
    if (track < 25)
      return 19;
    if (track < 31)
      return 18;
    return 17;

  case D64_TYPE_D81:
    return 40;

  case D64_TYPE_DNP:
    return 256;
  }
}

/**
 * checked_read - read a specified sector after range-checking
 * @part  : partition number
 * @track : track number to be read
 * @sector: sector number to be read
 * @buf   : pointer to where the data should be read to
 * @len   : number of bytes to be read
 * @error : error number to be flagged if the range check fails
 *
 * This function checks if the track and sector are within the
 * limits for the image format and calls image_read to read
 * the data if they are. Returns the result of image_read or
 * 2 if the range check failed.
 */
static uint8_t checked_read(uint8_t part, uint8_t track, uint8_t sector, uint8_t *buf, uint16_t len, uint8_t error) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= sectors_per_track(part, track)) {
    set_error_ts(error,track,sector);
    return 2;
  }

  if (partition[part].imagetype & D64_HAS_ERRORINFO) {
    /* Check if the sector is marked as bad */
    if (errorcache.part != part || errorcache.track != track) {
      /* Read the error info for this track */
      memset(errorcache.errors, 1, sizeof(errorcache.errors));

      switch (partition[part].imagetype & D64_TYPE_MASK) {
      case D64_TYPE_D41:
        if (image_read(part, D41_ERROR_OFFSET + sector_lba(part,track,0),
                       errorcache.errors, sectors_per_track(part, track)) >= 2)
          return 2;
        break;

      case D64_TYPE_D71:
        if (image_read(part, D71_ERROR_OFFSET + sector_lba(part,track,0),
                       errorcache.errors, sectors_per_track(part, track)) >= 2)
          return 2;
        break;

      case D64_TYPE_D81:
        if (image_read(part, D81_ERROR_OFFSET + sector_lba(part,track,0),
                       errorcache.errors, sectors_per_track(part, track)) >= 2)
          return 2;
        break;

      default:
        /* Should not happen unless someone enables error info
           for additional image formats */
        return 2;

      }
      errorcache.part  = part;
      errorcache.track = track;
    }

    /* Calculate error message from the code */
    if (errorcache.errors[sector] >= 2 && errorcache.errors[sector] <= 11) {
      /* Most codes can be mapped directly */
      set_error_ts(errorcache.errors[sector]-2+20,track,sector);
      return 2;
    }
    if (errorcache.errors[sector] == 15) {
      /* Drive not ready */
      set_error(74);
      return 2;
    }
    /* 1 is OK, unknown values are accepted too */
  }

  return image_read(part, sector_offset(part,track,sector), buf, len);
}

/**
 * update_timestamp - update timestamp of a directory entry
 * @buffer: pointer to the directory entry
 *
 * This function updates the time stamp of a directory entry
 * at the address pointed to by @buffer with the current
 * date/time if available or the default timestamp if not.
 */
static void update_timestamp(uint8_t *buffer) {
#ifdef HAVE_RTC
  struct tm time;

  read_rtc(&time);
  buffer[DIR_OFS_YEAR]   = time.tm_year % 100;
  buffer[DIR_OFS_MONTH]  = time.tm_mon + 1;
  buffer[DIR_OFS_DAY]    = time.tm_mday;
  buffer[DIR_OFS_HOUR]   = time.tm_hour;
  buffer[DIR_OFS_MINUTE] = time.tm_min;
#else
  buffer[DIR_OFS_YEAR]   = 82;
  buffer[DIR_OFS_MONTH]  = 8;
  buffer[DIR_OFS_DAY]    = 31;
  buffer[DIR_OFS_HOUR]   = 0;
  buffer[DIR_OFS_MINUTE] = 0;
#endif
}

/**
 * write_entry - write a single directory entry
 * @part : partition number
 * @dh   : pointer to d64dh pointing to the entry
 * @buf  : pointer to the buffer where the data should be read from
 * @flush: if true, data is flushed to disk immediately
 *
 * This function writes a single directory entry specified by @dh
 * from the buffer @buf.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_write
 */
static uint8_t write_entry(uint8_t part, struct d64dh *dh, uint8_t *buf, uint8_t flush) {
  return image_write(part, sector_offset(part, dh->track, dh->sector) +
                           dh->entry * 32, buf, 32, flush);
}

/**
 * read_entry - read a single directory entry
 * @part: partition number
 * @dh  : pointer to d64dh pointing to the entry
 * @buf : pointer to the buffer where the data should be stored
 *
 * This function reads a single directory entry specified by dh
 * into the buffer buf which needs to be at least 32 bytes big.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_read (0 success, 1 partial read, 2 failed)
 */
static uint8_t read_entry(uint8_t part, struct d64dh *dh, uint8_t *buf) {
  return image_read(part, sector_offset(part, dh->track, dh->sector) +
                          dh->entry * 32, buf, 32);
}

/**
 * strnsubst - substitute one character with another in a buffer
 * @buffer : pointer to the buffer
 * @len    : length of buffer
 * @oldchar: character to be replaced
 * @newchar: character to be used as a replacement
 *
 * This functions changes all occurences of oldchar in the first len
 * byte of buffer with newchar. Although it is named str*, it doesn't
 * care about zero bytes in any way.
 */
static void strnsubst(uint8_t *buffer, uint8_t len, uint8_t oldchar, uint8_t newchar) {
  uint8_t i=len-1;

  do {
    if (buffer[i] == oldchar)
      buffer[i] = newchar;
  } while (i--);
}

/* fill a sector with 0 0xff 0* to generate a new empty directory sector */
/* needs a 256 byte work area in *data */
static uint8_t clear_dir_sector(uint8_t part, uint8_t t, uint8_t s, uint8_t *data) {
  memset(data, 0, 256);

  data[1] = 0xff;
  return image_write(part, sector_offset(part, t, s), data, 256, 0);
}


/* ------------------------------------------------------------------------- */
/*  BAM buffer handling                                                      */
/* ------------------------------------------------------------------------- */

/**
 * bam_buffer_flush - write BAM buffer to disk
 * @buf: pointer to the BAM buffer
 *
 * This function writes the contents of the BAM buffer to the disk image.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t bam_buffer_flush(buffer_t *buf) {
  uint8_t res;

  if (buf->mustflush && buf->pvt.bam.part < max_part) {
    res = image_write(buf->pvt.bam.part,
                      sector_offset(buf->pvt.bam.part,
                                    buf->pvt.bam.track,
                                    buf->pvt.bam.sector),
                      buf->data, 256, 1);
    buf->mustflush = 0;

    return res;
  } else
    return 0;
}

/**
 * d64_bam_commit - write BAM buffers to disk
 *
 * This function is the exported interface to force the BAM buffer
 * contents to disk. Returns 0 if successful, != 0 otherwise.
 */
uint8_t d64_bam_commit(void) {
  uint8_t res = 0;

  if (bam_buffer)
    res |= bam_buffer->cleanup(bam_buffer);

  if (bam_buffer2)
    res |= bam_buffer2->cleanup(bam_buffer2);

  return 0;
}

/**
 * bam_buffer_alloc - allocates a buffer for the BAM
 * @buf: pointer to the BAM buffer pointer
 *
 * This function tries to allocate a buffer for holding the BAM,
 * using the pointer pointed to by @buf. Returns 0 if successful,
 * != 0 otherwise.
 */
static uint8_t bam_buffer_alloc(buffer_t **buf) {
  *buf = alloc_system_buffer();
  if (!*buf)
    return 1;

  (*buf)->secondary    = BUFFER_SYS_BAM;
  (*buf)->pvt.bam.part = 255;
  (*buf)->cleanup      = bam_buffer_flush;
  stick_buffer(*buf);

  return 0;
}

/**
 * bam_buffer_swap - swap the two BAM buffers
 *
 * Thus function swaps the pointers to the two BAM buffers
 */
static void bam_buffer_swap(void) {
  buffer_t *tmp = bam_buffer;
  bam_buffer    = bam_buffer2;
  bam_buffer2   = tmp;
}

/**
 * bam_buffer_match - checks if a BAM buffer contains a specific sector
 * @buf   : BAM buffer to check
 * @part  : partition
 * @track : track
 * @sector: sector
 *
 * This function checks if the BAM buffer @buf currently holds the data
 * for the given @part, @track and @sector. Returns 1 if it does,
 * 0 otherwise.
 */
static uint8_t bam_buffer_match(buffer_t *buf, uint8_t part,
                                uint8_t track, uint8_t sector) {
  if (buf->pvt.bam.part   == part  &&
      buf->pvt.bam.track  == track &&
      buf->pvt.bam.sector == sector)
    return 1;
  else
    return 0;
}

/**
 * move_bam_window - read correct BAM sector into window.
 * @part  : partition
 * @track : track number
 * @type  : type of pointer requested
 * @ptr   : pointer to track information in BAM sector
 *
 * This function reads the correct BAM sector into memory for the requested
 * track, flushing an existing BAM sector to disk if needed.  It also
 * calculates the correct pointer into the BAM sector for the appropriate
 * track.  Since the BAM contains both sector counts and sector allocation
 * bitmaps, type is used to signal which reference is desired.
 * This function may swap the BAM buffer pointers, after it returns
 * bam_buffer is always the buffer with the requested sector.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t move_bam_window(uint8_t part, uint8_t track, bamdata_t type, uint8_t **ptr) {
  uint8_t res;
  uint8_t t,s, pos;

  switch(partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D41:
  default:
    t   = D41_BAM_TRACK;
    s   = D41_BAM_SECTOR;
    pos = D41_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ? 1:0);
    break;

  case D64_TYPE_D71:
    if (track > 35 && type == BAM_BITFIELD) {
      t   = D71_BAM2_TRACK;
      s   = D71_BAM2_SECTOR;
      pos = (track - 36) * D71_BAM2_BYTES_PER_TRACK;
    } else {
      t = D41_BAM_TRACK;
      s = D41_BAM_SECTOR;
      if (track > 35) {
        pos = (track - 36) + D71_BAM_COUNTER2OFFSET;
      } else {
        pos = D41_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ? 1:0);
      }
    }
    break;

  case D64_TYPE_D81:
    t   = D81_BAM_TRACK;
    s   = (track < 41 ? D81_BAM_SECTOR1 : D81_BAM_SECTOR2);
    if (track > 40)
      track -= 40;
    pos = D81_BAM_OFFSET + track * D81_BAM_BYTES_PER_TRACK + (type == BAM_BITFIELD ? 1:0);
    break;

  case D64_TYPE_DNP:
    t   = DNP_BAM_TRACK;
    s   = DNP_BAM_SECTOR + (track >> 3);
    pos = (track & 0x07) * 32;
    break;
  }

  if (!bam_buffer_match(bam_buffer, part, t, s)) {
    /* check if the second BAM buffer exists */
    if (bam_buffer2) {
      /* unconditionally swap to the other BAM buffer        */
      /* either it has the data or it was less-recently used */

      /* Note for future optimization:
       * If buf1 is invalid and buf2 valid but not matching this will swap and
       * use buf2 for the data instead of buf1 - using buf1
       * while keeping buf2 without swapping would be better.
       */
      bam_buffer_swap();

      /* check again, now in the other buffer */
      if (bam_buffer_match(bam_buffer, part, t, s))
        goto found;
    } else {
      /* allocate and swap to the second BAM buffer if the first one is valid */
      if (bam_buffer->pvt.bam.part != 255) {
        if (bam_buffer_alloc(&bam_buffer2)) {
          /* allocation failed, reset error and continue with just one buffer */
          set_error(ERROR_OK);
        } else {
          /* switch to the new buffer */
          bam_buffer_swap();
        }
      }
    }

    /* Need to read the BAM sector - flush only the target buffer */
    if (bam_buffer->cleanup(bam_buffer))
      return 1;

    res = image_read(part, sector_offset(part, t, s), bam_buffer->data, 256);
    if(res)
      return res;

    bam_buffer->pvt.bam.part   = part;
    bam_buffer->pvt.bam.track  = t;
    bam_buffer->pvt.bam.sector = s;
  }

 found:
  *ptr = bam_buffer->data + pos;
  return 0;
}

/**
 * is_free - checks if the given sector is marked as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function checks if the given sector is marked as free in
 * the BAM of drive "part". Returns 0 if allocated, >0 if free, <0 on error.
 */
static int8_t is_free(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t res;
  uint8_t *ptr = NULL;

  res = move_bam_window(part,track,BAM_BITFIELD,&ptr);
  if(res)
    return -1;

  if (partition[part].imagetype == D64_TYPE_DNP)
    return (ptr[sector>>3] & (0x80>>(sector&7))) != 0;
  else
    return (ptr[sector>>3] & (1<<(sector&7))) != 0;
}

/**
 * sectors_free - returns the number of free sectors on a given track
 * @part  : partition
 * @track : track number
 *
 * This function returns the number of free sectors on the given track
 * of partition part.
 */
static uint16_t sectors_free(uint8_t part, uint8_t track) {
  uint8_t *trackmap = NULL;

  if (track < 1 || track > get_param(part, LAST_TRACK))
    return 0;

  switch (partition[part].imagetype & D64_TYPE_MASK) {

  case D64_TYPE_DNP:
    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 0;

    uint16_t blocks = 0;
    for (uint8_t i=0;i < DNP_BAM_BYTES_PER_TRACK;i++) {
      // From http://everything2.com/title/counting%25201%2520bits
      uint8_t b = (trackmap[i] & 0x55) + (trackmap[i]>>1 & 0x55);
      b = (b & 0x33) + (b >> 2 & 0x33);
      b = (b & 0x0f) + (b >> 4 & 0x0f);
      blocks += b;
    }
    return blocks;

  case D64_TYPE_D71:
  case D64_TYPE_D81:
  case D64_TYPE_D41:
  default:
    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 0;
    return *trackmap;
  }
}

/**
 * allocate_sector - mark a sector as used
 * @part  : partitoin
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as used in the BAM of drive part.
 * If the sector was already marked as used nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t allocate_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res != 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    if (partition[part].imagetype == D64_TYPE_DNP) {
      /* For some reason DNP has its bitfield reversed */
      trackmap[sector>>3] &= (uint8_t)~(0x80>>(sector&7));

      /* DNP has no counter in its BAM */
      return 0;
    }

    trackmap[sector>>3] &= (uint8_t)~(1<<(sector&7));

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if (trackmap[0] > 0) {
      trackmap[0]--;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * free_sector - mark a sector as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as free in the BAM of partition
 * part. If the sector was already marked as free nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t free_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res == 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    if (partition[part].imagetype == D64_TYPE_DNP) {
      /* For some reason DNP has its bitfield reversed */
      trackmap[sector>>3] |= 0x80>>(sector&7);

      /* DNP has no counter in its BAM */
      return 0;
    }

    trackmap[sector>>3] |= 1<<(sector&7);

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if(trackmap[0] < sectors_per_track(part, track)) {
      trackmap[0]++;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * get_first_sector - calculate the first sector for a new file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the first sector to be allocated for a new
 * file. The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 *
 * This code will not skip track 53 of a D71 if there are any free
 * sectors on it - this behaviour is consistent with that of a 1571
 * with a revision 3.0 ROM.
 */
static uint8_t get_first_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  int8_t distance = 1;

  /* DNP uses a simple "first free" allocation scheme, starting at track 2.  */
  /* It is not known if this is the same algorithm as used in the original   */
  /* CMD drives, but track 1 seems to be semi-reserved for directory sectors.*/
  if (partition[part].imagetype == D64_TYPE_DNP) {
    *track = 2;
    while (sectors_free(part, *track) == 0) {
      (*track)++;

      if (*track == get_param(part, LAST_TRACK) ||
          *track == 0) {
        /* Wrap to track 1 */
        *track = 1;
      }

      if (*track == 2)
        /* If we're at track 2 again there are no free sectors anywhere */
        return 1;
    }
  } else {
    /* Look for a track with free sectors close to the directory */
    while (distance < get_param(part, LAST_TRACK)) {
      if (sectors_free(part, get_param(part, DIR_TRACK)-distance))
        break;

      /* Invert sign */
      distance = -distance;

      /* Increase distance every second try */
      if (distance > 0)
        distance++;
    }

    if (distance == get_param(part, LAST_TRACK)) {
      if (current_error == ERROR_OK)
        set_error(ERROR_DISK_FULL);
      return 1;
    }

    *track = get_param(part, DIR_TRACK)-distance;
  }

  /* Search for the first free sector on this track */
  for (*sector = 0;*sector < sectors_per_track(part, *track); *sector += 1)
    if (is_free(part, *track, *sector) > 0)
      return 0;

  /* If we're here the BAM is invalid or couldn't be read */
  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * get_next_sector - calculate the next sector for a file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the next sector to be allocated for a file
 * based on the current sector in the variables pointed to by track/sector.
 * The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 */
static uint8_t get_next_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  if (partition[part].imagetype == D64_TYPE_DNP) {
    uint8_t newtrack = *track;

    /* Find a track with free sectors */
    while (sectors_free(part, newtrack) == 0) {
      newtrack++;

      if (newtrack == get_param(part, LAST_TRACK) ||
          newtrack == 0) {
        /* Wrap to track 1 */
        newtrack = 1;
      }

      if (newtrack == *track)
        /* Returned to the start track -> no free sectors anywhere */
        return 1;
    }

    uint8_t newsector;

    if (newtrack == *track) {
      /* Same track: start at the previous sector */
      newsector = *sector;
    } else {
      /* New track: start at sector 0 */
      newsector = 0;
    }

    while (!is_free(part, newtrack, newsector))
      newsector++; // will automatically wrap from 255->0

    *track = newtrack;
    *sector = newsector;

    return 0;
  }

  uint8_t interleave,tries;

  if (*track == get_param(part, DIR_TRACK)) {
    if (sectors_free(part, get_param(part, DIR_TRACK)) == 0) {
      if (current_error == ERROR_OK)
        set_error(ERROR_DISK_FULL);
      return 1;
    }
    interleave = get_param(part, DIR_INTERLEAVE);
  } else
    interleave = get_param(part, FILE_INTERLEAVE);

  /* Look for a track with free sectors */
  tries = 0;
  while (tries < 3 && !sectors_free(part, *track)) {
    /* No more space on current track, try another */
    if (*track < get_param(part, DIR_TRACK))
      *track -= 1;
    else {
      *track += 1;
      /* Skip track 53 on D71 images */
      if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_D71 &&
          *track == D71_BAM2_TRACK)
        *track += 1;
    }

    if (*track < 1) {
      *track = get_param(part, DIR_TRACK) + 1;
      *sector = 0;
      tries++;
    }
    if (*track > get_param(part, LAST_TRACK)) {
      *track = get_param(part, DIR_TRACK) - 1;
      *sector = 0;
      tries++;
    }
  }

  if (tries == 3) {
    if (current_error == ERROR_OK)
      set_error(ERROR_DISK_FULL);
    return 1;
  }

  /* Look for a sector at interleave distance */
  *sector += interleave;
  if (*sector >= sectors_per_track(part, *track)) {
    *sector -= sectors_per_track(part, *track);
    if (*sector != 0)
      *sector -= 1;
  }

  /* Increase distance until an empty sector is found */
  tries = 99;
  while (is_free(part,*track,*sector) <= 0 && tries--) {
    *sector += 1;
    if (*sector >= sectors_per_track(part, *track))
      *sector = 0;
  }

  if (tries)
    return 0;

  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * nextdirentry - read the next dir entry to ops_scratch
 * @dh: directory handle
 *
 * This function reads the next directory entry from the disk
 * into ops_scratch. Returns 1 if an error occured, -1 if there
 * are no more directory entries and 0 if successful. This
 * function will return every entry, even deleted ones.
 */
static int8_t nextdirentry(dh_t *dh) {
  /* End of directory entries in this sector? */
  if (dh->dir.d64.entry == 8) {
    /* Read link pointer */
    if (checked_read(dh->part, dh->dir.d64.track, dh->dir.d64.sector, ops_scratch, 2, ERROR_ILLEGAL_TS_LINK))
      return 1;

    /* Final directory sector? */
    if (ops_scratch[0] == 0)
      return -1;

    dh->dir.d64.track  = ops_scratch[0];
    dh->dir.d64.sector = ops_scratch[1];
    dh->dir.d64.entry  = 0;
  }

  if (dh->dir.d64.track < 1 || dh->dir.d64.track > get_param(dh->part, LAST_TRACK) ||
      dh->dir.d64.sector >= sectors_per_track(dh->part, dh->dir.d64.track)) {
    set_error_ts(ERROR_ILLEGAL_TS_LINK,dh->dir.d64.track,dh->dir.d64.sector);
    return 1;
  }

  if (read_entry(dh->part, &dh->dir.d64, ops_scratch))
    return 1;

  dh->dir.d64.entry++;

  return 0;
}

/**
 * find_empty_entry - find an empty directory entry
 * @path: path of the directory
 * @dh: directory handle
 *
 * This funtion finds an empty entry in the directory, creating
 * a new directory sector if required.
 * Returns 0 if successful or 1 if not.
 */
static uint8_t find_empty_entry(path_t *path, dh_t *dh) {
  int8_t res;

  d64_opendir(dh, path);
  do {
    res = nextdirentry(dh);
    if (res > 0)
      return 1;
  } while (res == 0 && ops_scratch[DIR_OFS_FILE_TYPE] != 0);

  /* Allocate a new directory sector if no empty entries were found */
  if (res < 0) {
    uint8_t t,s;

    t = dh->dir.d64.track;
    s = dh->dir.d64.sector;

    if (get_next_sector(path->part, &dh->dir.d64.track, &dh->dir.d64.sector))
      return 1;

    /* Link the old sector to the new */
    ops_scratch[0] = dh->dir.d64.track;
    ops_scratch[1] = dh->dir.d64.sector;
    if (image_write(path->part, sector_offset(path->part,t,s), ops_scratch, 2, 0))
      return 1;

    if (allocate_sector(path->part, dh->dir.d64.track, dh->dir.d64.sector))
      return 1;

    /* DNP only: Increment the block count in the parent directory */
    if (partition[path->part].imagetype == D64_TYPE_DNP) {
      if (image_read(path->part,
                     DNP_DIRHEADER_PARENTENTRY_TRACK + sector_offset(path->part,
                                                                     path->dir.dxx.track,
                                                                     path->dir.dxx.sector),
                     ops_scratch, 3))
        return 1;

      if (ops_scratch[0] != 0) {
        /* Read block count of entry in parent directory */
        if (image_read(path->part,
                       sector_offset(path->part, ops_scratch[0], ops_scratch[1]) + ops_scratch[2] + DIR_OFS_SIZE_LOW - 2,
                       ops_scratch + 3, 2))
          return 1;

        uint16_t *blocks = (uint16_t *)(ops_scratch + 3);
        (*blocks)++;

        /* Write new block count */
        if (image_write(path->part,
                        sector_offset(path->part, ops_scratch[0], ops_scratch[1]) + ops_scratch[2] + DIR_OFS_SIZE_LOW - 2,
                        ops_scratch + 3, 2, 1))
          return 1;
      }
    }

    /* Clear the new directory sector */
    memset(ops_scratch, 0, 32);
    ops_scratch[1] = 0xff;
    for (uint8_t i=0;i<256/32;i++) {
      dh->dir.d64.entry = i;
      if (write_entry(path->part, &dh->dir.d64, ops_scratch, 0))
        return 1;

      ops_scratch[1] = 0;
    }

    /* Mark full sector as used */
    ops_scratch[1] = 0xff;
    dh->dir.d64.entry = 0;

  } else {
    /* nextdirentry has already incremented this variable, undo it */
    dh->dir.d64.entry--;
  }

  return 0;
}

/**
 * d64_read - refill-callback used for reading
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for reading.
 */
static uint8_t d64_read(buffer_t *buf) {
  /* Store the current sector, used for append */
  buf->pvt.d64.track  = buf->data[0];
  buf->pvt.d64.sector = buf->data[1];

  if (checked_read(buf->pvt.d64.part, buf->data[0], buf->data[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK)) {
    free_buffer(buf);
    return 1;
  }

  buf->position = 2;

  if (buf->data[0] == 0) {
    /* Final sector of the file */
    buf->lastused = buf->data[1];
    buf->sendeoi  = 1;
  } else {
    buf->lastused = 255;
    buf->sendeoi  = 0;
  }

  return 0;
}

/**
 * d64_seek - seek-callback
 * @buf     : target buffer
 * @position: offset to seek to
 * @index   : offset within the record to seek to
 *
 * This is the function used as the seek callback. Since seeking
 * isn't supported for D64 files it just sets an error message
 * and returns 1.
 */
static uint8_t d64_seek(buffer_t *buf, uint32_t position, uint8_t index) {
  (void)buf;
  (void)position;
  (void)index;

  set_error(ERROR_SYNTAX_UNABLE);
  return 1;
}

/**
 * d64_write - refill-callback used for writing
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for writing.
 */
static uint8_t d64_write(buffer_t *buf) {
  uint8_t t, s, savederror;

  savederror = 0;
  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;

  buf->pvt.d64.blocks++;

  /* Mark as last sector in case something below fails */
  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  /* Find another free sector */
  if (get_next_sector(buf->pvt.d64.part, &t, &s)) {
    t = 0;
    savederror = current_error;
    goto storedata;
  }

  buf->data[0] = t;
  buf->data[1] = s;

  /* Allocate it */
  if (allocate_sector(buf->pvt.d64.part, t, s)) {
    free_buffer(buf);
    return 1;
  }

 storedata:
  /* Store data in the already-reserved sector */
  if (image_write(buf->pvt.d64.part,
                  sector_offset(buf->pvt.d64.part,
                                buf->pvt.d64.track,
                                buf->pvt.d64.sector),
                  buf->data, 256, 1)) {
    free_buffer(buf);
    return 1;
  }

  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
  buf->position  = 2;
  buf->lastused  = 1;
  buf->mustflush = 0;
  mark_buffer_clean(buf);

  if (savederror) {
    set_error(savederror);
    free_buffer(buf);
    return 1;
  } else
    return 0;
}

static uint8_t d64_write_cleanup(buffer_t *buf) {
  uint8_t t,s;

  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;
  buf->pvt.d64.blocks++;

  /* Track=0 means that there was an error somewhere earlier */
  if (t == 0)
    return 1;

  /* Store data */
  if (image_write(buf->pvt.d64.part, sector_offset(buf->pvt.d64.part,t,s), buf->data, 256, 1))
    return 1;

  /* Update directory entry */
  if (read_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch))
    return 1;

  ops_scratch[DIR_OFS_FILE_TYPE] |= FLAG_SPLAT;
  ops_scratch[DIR_OFS_SIZE_LOW]   = buf->pvt.d64.blocks & 0xff;
  ops_scratch[DIR_OFS_SIZE_HI]    = buf->pvt.d64.blocks >> 8;
  update_timestamp(ops_scratch);

  if (write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1))
    return 1;

  buf->cleanup = callback_dummy;
  free_buffer(buf);

  return 0;
}


/* ------------------------------------------------------------------------- */
/*  fileops-API                                                              */
/* ------------------------------------------------------------------------- */

uint8_t d64_mount(path_t *path, uint8_t *name) {
  uint8_t imagetype;
  uint8_t part = path->part;
  uint32_t fsize = partition[part].imagehandle.fsize;

  switch (fsize) {
  case 174848:
    imagetype = D64_TYPE_D41;
    memcpy_P(&partition[part].d64data, &d41param, sizeof(struct param_s));
    break;

  case 175531:
    imagetype = D64_TYPE_D41 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d41param, sizeof(struct param_s));
    break;

  case 349696:
    imagetype = D64_TYPE_D71;
    memcpy_P(&partition[part].d64data, &d71param, sizeof(struct param_s));
    break;

  case 351062:
    imagetype = D64_TYPE_D71 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d71param, sizeof(struct param_s));
    break;

  case 819200:
    imagetype = D64_TYPE_D81;
    memcpy_P(&partition[part].d64data, &d81param, sizeof(struct param_s));
    break;

  case 822400:
    imagetype = D64_TYPE_D81 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d81param, sizeof(struct param_s));
    break;

  default:
    if ((fsize % (256*256L)) != 0) {
      set_error(ERROR_IMAGE_INVALID);
      return 1;
    }

    /* sanity check: ignore 40-track D64 images */
    if (fsize == 196608) {
      uint8_t *ptr = ustrrchr(name, '.');

      if (ptr[2] == '6' && ptr[3] == '4') {
        set_error(ERROR_IMAGE_INVALID);
        return 1;
      }
    }

    imagetype = D64_TYPE_DNP;
    memcpy_P(&partition[part].d64data, &dnpparam, sizeof(struct param_s));
    partition[part].d64data.last_track = fsize / (256*256L);
  }

  /* allocate the first BAM buffer if required */
  if (bam_buffer == NULL) {
    if (bam_buffer_alloc(&bam_buffer))
      return 1;
  }

  partition[part].imagetype = imagetype;
  path->dir.dxx.track  = get_param(part, DIR_TRACK);
  path->dir.dxx.sector = get_param(part, DIR_START_SECTOR);

  bam_refcount++;

  if (imagetype & D64_HAS_ERRORINFO)
    /* Invalidate error cache */
    errorcache.part = 255;

  return 0;
}

static uint8_t d64_opendir(dh_t *dh, path_t *path) {
  dh->part = path->part;
  dh->dir.d64.track  = path->dir.dxx.track;
  dh->dir.d64.sector = path->dir.dxx.sector;
  dh->dir.d64.entry  = 0;

  if (partition[path->part].imagetype == D64_TYPE_DNP) {
    /* Read the real first directory sector from the header sector */
    uint8_t tmp[2];
    if (image_read(path->part,
                   sector_offset(path->part, dh->dir.d64.track, dh->dir.d64.sector),
                   tmp, 2))
      return 1;

    dh->dir.d64.track  = tmp[0];
    dh->dir.d64.sector = tmp[1];
  }
  return 0;
}

static int8_t d64_readdir(dh_t *dh, cbmdirent_t *dent) {
  int8_t res;

  do {
    res = nextdirentry(dh);
    if (res)
      return res;

    if (ops_scratch[DIR_OFS_FILE_TYPE] != 0)
      break;
  } while (1);

  memset(dent, 0, sizeof(cbmdirent_t));

  dent->opstype = OPSTYPE_DXX;
  dent->typeflags = (ops_scratch[DIR_OFS_FILE_TYPE] ^ FLAG_SPLAT)
    & ~FLAG_HIDDEN;

  if ((dent->typeflags & TYPE_MASK) > TYPE_DIR)
    /* Change invalid types to DEL */
    /* FIXME: Should exclude DIR on non-DNP */
    dent->typeflags &= (uint8_t)~TYPE_MASK;

  dent->pvt.dxx.dh = dh->dir.d64;
  dent->pvt.dxx.dh.entry -= 1; /* undo increment in nextdirentry */
  dent->blocksize = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI];
  dent->remainder = 0xff;
  memcpy(dent->name, ops_scratch + DIR_OFS_FILE_NAME, CBM_NAME_LENGTH);
  strnsubst(dent->name, 16, 0xa0, 0);

  /* sanitize values in case they were not stored */
  dent->date.minute=  ops_scratch[DIR_OFS_MINUTE] % 60;
  dent->date.hour  =  ops_scratch[DIR_OFS_HOUR] % 24;
  dent->date.day   = (ops_scratch[DIR_OFS_DAY]   - 1) % 31 + 1;
  dent->date.month = (ops_scratch[DIR_OFS_MONTH] - 1) % 12 + 1;
  dent->date.year  =  ops_scratch[DIR_OFS_YEAR] % 100;
  /* adjust year into 1980..2079 range */
  if (dent->date.year < 80)
    dent->date.year += 100;

  return 0;
}

/* Reads and converts a string from the dir header sector (BAM for D41/D71) to the buffer */
/* Used by d64_get(disk|dir)label and d64_getid */
static uint8_t read_string_from_dirheader(path_t *path, uint8_t *buffer, param_t what, uint8_t size) {
  uint8_t sector;

  if (partition[path->part].imagetype == D64_TYPE_DNP)
    sector = path->dir.dxx.sector;
  else
    sector = 0;

  if (image_read(path->part,
                 sector_offset(path->part, path->dir.dxx.track, sector)
                 + get_param(path->part, what),
                 buffer, size))
    return 1;

  strnsubst(buffer, size, 0xa0, 0x20);
  return 0;
}

static uint8_t d64_getdirlabel(path_t *path, uint8_t *label) {
  return read_string_from_dirheader(path, label, LABEL_OFFSET, 16);
}

static uint8_t d64_getdisklabel(uint8_t part, uint8_t *label) {
  if (partition[part].imagetype == D64_TYPE_DNP) {
    /* Read directly from root dir header */
    if (image_read(part, 256 + DNP_LABEL_OFFSET, label, 16))
      return 1;
  } else {
    /* Use getdirlabel instead */
    path_t curpath;

    curpath.part = part;
    curpath.dir = partition[part].current_dir;
    if (d64_getdirlabel(&curpath, label))
      return 1;
  }

  /* Zero-terminate label */
  uint8_t *ptr = label+16;
  *ptr-- = 0;
  while (ptr != label && *ptr == ' ')
    *ptr-- = 0;

  return 0;
}

static uint8_t d64_getid(path_t *path, uint8_t *id) {
  return read_string_from_dirheader(path, id, ID_OFFSET, 5);
}

static uint16_t d64_freeblocks(uint8_t part) {
  uint16_t blocks = 0;
  uint8_t i;

  for (i = 1; i != 0 && i <= get_param(part, LAST_TRACK); i++) {
    /* Skip directory track */
    switch (partition[part].imagetype & D64_TYPE_MASK) {
    case D64_TYPE_D81:
      if (i == D81_BAM_TRACK)
        continue;
      break;

    case D64_TYPE_DNP:
      /* DNP reserves a partial track, handled below */
      break;

    case D64_TYPE_D41:
    case D64_TYPE_D71:
    default:
      if (i == D41_BAM_TRACK || i == D71_BAM2_TRACK)
        continue; // continue the for loop
      break;      // break out of the switch

    }

    if ((partition[part].imagetype & D64_TYPE_MASK)
        == D64_TYPE_DNP && i == 1) {
      /* DNP: ignore sectors 0-63 on track 1 */
      for (uint16_t j = 64; j < 256; j++) {
        if (is_free(part, 1, j) > 0)
          blocks++;
      }

    } else {
      blocks += sectors_free(part,i);
    }
  }

  return blocks;
}

static void d64_open_read(path_t *path, cbmdirent_t *dent, buffer_t *buf) {
  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

  buf->data[0] = ops_scratch[DIR_OFS_TRACK];
  buf->data[1] = ops_scratch[DIR_OFS_SECTOR];

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

/**
 * create_entry - prepare the directory entry of a new file
 * @path: path of the file
 * @dent: name of the file
 * @type: file type byte of the entry
 * @dh  : receives the handle of the new directory entry
 * @t   : receives the track of the first data sector
 * @s   : receives the sector of the first data sector
 *
 * Finds an empty directory entry, builds the new entry in ops_scratch
 * and allocates the first data sector of the file. The entry is not
 * written to the image.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t create_entry(path_t *path, cbmdirent_t *dent, uint8_t type,
                            dh_t *dh, uint8_t *t, uint8_t *s) {
  uint8_t *name = dent->name;
  uint8_t *ptr  = ops_scratch + DIR_OFS_FILE_NAME;

  if (find_empty_entry(path, dh))
    return 1;

  memset(ops_scratch + 2, 0, sizeof(ops_scratch) - 2);  /* Don't overwrite the link pointer! */
  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  while (*name) *ptr++ = *name++;
  ops_scratch[DIR_OFS_FILE_TYPE] = type;

  /* Find a free sector and allocate it */
  if (get_first_sector(path->part, t, s))
    return 1;

  ops_scratch[DIR_OFS_TRACK]  = *t;
  ops_scratch[DIR_OFS_SECTOR] = *s;

  return allocate_sector(path->part, *t, *s);
}

static void d64_open_write(path_t *path, cbmdirent_t *dent, uint8_t type, buffer_t *buf, uint8_t append) {
  dh_t dh;

  /* Check for read-only image file */
  if (!(partition[path->part].imagehandle.flag & FA_WRITE)) {
    set_error(ERROR_WRITE_PROTECT);
    return;
  }

  if (append) {
    /* Append case: Open the file and read the last sector */
    d64_open_read(path, dent, buf);
    while (!current_error && buf->data[0])
      buf->refill(buf);

    if (current_error)
      return;

    /* Modify the buffer for writing */
    buf->pvt.d64.dh     = dent->pvt.dxx.dh;
    buf->pvt.d64.blocks = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI]-1;
    buf->read       = 0;
    buf->position   = buf->lastused+1;
    if (buf->position == 0)
      buf->mustflush = 1;
    else
      buf->mustflush = 0;
    buf->refill     = d64_write;
    buf->cleanup    = d64_write_cleanup;
    buf->seek       = d64_seek;
    mark_write_buffer(buf);

    update_timestamp(ops_scratch);
    write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1);

    return;
  }

  /* Non-append case: create the directory entry in ops_scratch */
  uint8_t t,s;

  if (create_entry(path, dent, type, &dh, &t, &s))
    return;

  /* Write the directory entry */
  update_timestamp(ops_scratch);
  if (write_entry(path->part, &dh.dir.d64, ops_scratch, 1))
    return;

  /* Prepare the data buffer */
  mark_write_buffer(buf);
  buf->position       = 2;
  buf->lastused       = 2;
  buf->cleanup        = d64_write_cleanup;
  buf->refill         = d64_write;
  buf->seek           = d64_seek;
  buf->data[2]        = 13; /* Verified on VICE */
  buf->pvt.d64.dh     = dh.dir.d64;
  buf->pvt.d64.part   = path->part;
  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
}

/* Upper bound for following a file's sector chain in d64_delete, larger
   than the number of sectors in any supported image */
#define FILE_CHAIN_LIMIT    65536ul

#ifdef CONFIG_D64_REL
/* ------------------------------------------------------------------------- */
/*  REL file handling                                                        */
/* ------------------------------------------------------------------------- */
/* CONFIG_D64_REL controls whether REL files on disk images are supported;
   without it d64_open_rel reports an error and the side sectors of a
   deleted file are left to be recycled by the image's format operation. */

/* Offsets in a side sector block */
#define SS_OFS_NEXT_TRACK   0
#define SS_OFS_NEXT_SECTOR  1
#define SS_OFS_NUMBER       2
#define SS_OFS_RECORD_SIZE  3
#define SS_OFS_FIRST_SS     4   /* t/s of side sector 0, repeated for SS1-SS5 */
#define SS_OFS_POINTER      16  /* first data sector pointer */
#define SS_INDICES          120 /* data sector pointers per side sector */
#define SS_SIDE_SECTORS     6   /* side sectors per group */
#define SS_SUPER_GROUPS     126 /* group entries in a super side sector */

/* Upper bound for following the side sector chain, larger than the
   number of side sector blocks in any supported image */
#define SS_CHAIN_LIMIT      768

/* The super side sector stores the head of each group's chain at this
   offset instead of a record size; bytes 0/1 point to the first group. */
#define SS_OFS_SUPER_POINTER 3

#define REL_SUPER_MARKER    254 /* side sector number of a super side sector */

/* Read a full sector, flagging an illegal t/s link on range errors */
static uint8_t rel_read(uint8_t part, uint8_t track, uint8_t sector, uint8_t *data) {
  return checked_read(part, track, sector, data, 256, ERROR_ILLEGAL_TS_LINK);
}

/* Write a full sector */
static uint8_t rel_write(uint8_t part, uint8_t track, uint8_t sector, uint8_t *data) {
  return image_write(part, sector_offset(part, track, sector), data, 256, 1);
}

/* Write the buffer's data sector to the image if it holds changes */
static uint8_t rel_commit(buffer_t *buf) {
  if (buf->dirty) {
    if (rel_write(buf->pvt.d64.part, buf->pvt.d64.track, buf->pvt.d64.sector,
                  buf->data))
      return 1;
    mark_buffer_clean(buf);
  }
  return 0;
}

/* Discard uncommitted changes to the buffer's data sector from
   position @pos on by reading that part back from the image */
static uint8_t rel_restore(buffer_t *buf, uint8_t pos) {
  uint8_t part = buf->pvt.d64.part;

  return image_read(part, sector_offset(part, buf->pvt.d64.track,
                                        buf->pvt.d64.sector) + pos,
                    buf->data + pos, 256 - pos);
}

/* Buffer position of a file offset within its data sector */
static uint8_t rel_pos(uint32_t offset) {
  return 2 + offset % 254;
}

/* File offset of the first byte of a record */
static uint32_t rel_offset(buffer_t *buf, uint16_t record) {
  return (uint32_t)record * buf->recordlen;
}

/**
 * rel_read_sidesector - read a side sector block into the buffer data area
 * @buf: buffer of an open REL file
 * @dsec: data sector index to locate
 * @sst : receives the track of the side sector block (may be NULL)
 * @sss : receives the sector of the side sector block (may be NULL)
 *
 * This function reads the side sector block that holds the track/sector
 * pointer of data sector number @dsec into buf->data, following the
 * side sector chain (or the super side sector if present).
 * Returns a pointer to the track/sector entry of data sector @dsec in
 * buf->data if successful, NULL otherwise.
 */
static uint8_t *rel_read_sidesector(buffer_t *buf, uint16_t dsec,
                                    uint8_t *sst, uint8_t *sss) {
  uint8_t part   = buf->pvt.d64.part;
  uint8_t track  = buf->pvt.d64.sstrack;
  uint8_t sector = buf->pvt.d64.sssector;
  uint16_t block = dsec / SS_INDICES;
  uint8_t  index = dsec % SS_INDICES;

  if (buf->pvt.d64.relflags & REL_FLAG_SUPER) {
    uint8_t group = block / SS_SIDE_SECTORS;

    if (rel_read(part, track, sector, buf->data))
      return NULL;

    if (buf->data[SS_OFS_NUMBER] != REL_SUPER_MARKER) {
      set_error(ERROR_IMAGE_INVALID);
      return NULL;
    }

    track  = buf->data[SS_OFS_SUPER_POINTER + 2 * group];
    sector = buf->data[SS_OFS_SUPER_POINTER + 1 + 2 * group];
    block %= SS_SIDE_SECTORS;
  }

  /* Follow the side sector chain; the walk takes at most @block steps,
     so a cyclic chain cannot hang it */
  while (1) {
    if (rel_read(part, track, sector, buf->data))
      return NULL;

    if (block == 0) {
      if (sst != NULL)
        *sst = track;
      if (sss != NULL)
        *sss = sector;
      return buf->data + SS_OFS_POINTER + 2 * index;
    }

    block--;
    track  = buf->data[SS_OFS_NEXT_TRACK];
    sector = buf->data[SS_OFS_NEXT_SECTOR];
    if (track == 0) {
      set_error_ts(ERROR_ILLEGAL_TS_LINK, track, sector);
      return NULL;
    }
  }
}

/**
 * rel_load_data_sector - load a REL file data sector into the buffer
 * @buf: buffer of an open REL file
 * @dsec: data sector index
 *
 * Locates data sector @dsec using the side sector chain and reads it
 * into buf->data. Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_load_data_sector(buffer_t *buf, uint16_t dsec) {
  uint8_t track, sector;

  uint8_t *entry = rel_read_sidesector(buf, dsec, NULL, NULL);

  if (entry == NULL)
    return 1;

  track  = entry[0];
  sector = entry[1];

  if (track == 0) {
    set_error(ERROR_RECORD_MISSING);
    return 1;
  }

  buf->pvt.d64.track  = track;
  buf->pvt.d64.sector = sector;

  return rel_read(buf->pvt.d64.part, track, sector, buf->data);
}

/* Load the data sector holding the current file position */
static uint8_t rel_load_fptr(buffer_t *buf) {
  return rel_load_data_sector(buf, buf->fptr / 254);
}

/**
 * rel_write_empty - write an empty data sector of a REL file
 * @buf : buffer of the REL file, its data area is overwritten
 * @dsec: index of the data sector within the file
 * @t   : track of the data sector
 * @s   : sector of the data sector
 *
 * Fills the data area with the empty record pattern - bytes at record
 * boundaries become 0xff and the rest 0x00, matching the pattern a 1541
 * writes into freshly allocated data sectors - marks the sector as the
 * last one of the file, using only its complete records, and writes it.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_write_empty(buffer_t *buf, uint16_t dsec, uint8_t t, uint8_t s) {
  uint8_t length = buf->recordlen;
  uint8_t pos = ((uint32_t)dsec * 254) % length;
  uint8_t i = 2;

  do {
    buf->data[i] = pos ? 0x00 : 0xff;
    if (++pos == length)
      pos = 0;
  } while (++i != 0);

  buf->data[SS_OFS_NEXT_TRACK]  = 0;
  buf->data[SS_OFS_NEXT_SECTOR] = 255 - pos;
  return rel_write(buf->pvt.d64.part, t, s, buf->data);
}

/**
 * rel_init_sidesector - set up a new side sector block in the buffer
 * @buf   : buffer whose data area is overwritten
 * @number: number of the block within its group
 * @st    : track of the side sector block
 * @ss    : sector of the side sector block
 * @t     : track of the first data sector it points to
 * @s     : sector of the first data sector it points to
 *
 * Clears the data area and fills in the header and the first data
 * sector pointer. The block's own entry in the group table is set,
 * entries of other blocks in the group are left to the caller.
 */
static void rel_init_sidesector(buffer_t *buf, uint8_t number, uint8_t st,
                                uint8_t ss, uint8_t t, uint8_t s) {
  memset(buf->data, 0, 256);
  buf->data[SS_OFS_NEXT_SECTOR] = SS_OFS_POINTER + 1;
  buf->data[SS_OFS_NUMBER]      = number;
  buf->data[SS_OFS_RECORD_SIZE] = buf->recordlen;
  buf->data[SS_OFS_FIRST_SS + 2 * number]     = st;
  buf->data[SS_OFS_FIRST_SS + 2 * number + 1] = ss;
  buf->data[SS_OFS_POINTER]     = t;
  buf->data[SS_OFS_POINTER + 1] = s;
}

/**
 * rel_update_dir - store the block count in the directory entry
 * @buf      : buffer of an open REL file
 * @timestamp: update the entry timestamp as well
 *
 * Reads the directory entry of the file and updates its size field
 * with the current number of allocated blocks.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_update_dir(buffer_t *buf, uint8_t timestamp) {
  if (read_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch))
    return 1;

  ops_scratch[DIR_OFS_SIZE_LOW] = buf->pvt.d64.blocks & 0xff;
  ops_scratch[DIR_OFS_SIZE_HI]  = buf->pvt.d64.blocks >> 8;
  if (timestamp)
    update_timestamp(ops_scratch);

  return write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1);
}

/**
 * rel_patch_ts - store a track/sector pair in a sector of the image
 * @buf   : buffer of the REL file, its data area is overwritten
 * @track : track of the sector to change
 * @sector: sector of the sector to change
 * @ofs   : offset of the pair within the sector
 * @t     : track to store
 * @s     : sector to store
 *
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_patch_ts(buffer_t *buf, uint8_t track, uint8_t sector,
                            uint8_t ofs, uint8_t t, uint8_t s) {
  uint8_t part = buf->pvt.d64.part;

  if (rel_read(part, track, sector, buf->data))
    return 1;

  buf->data[ofs]     = t;
  buf->data[ofs + 1] = s;
  return rel_write(part, track, sector, buf->data);
}

/**
 * rel_alloc - allocate a sector for a REL file
 * @buf  : buffer of the REL file
 * @track: track to start the search at, receives the allocated track
 * @sector: sector to start the search at, receives the allocated sector
 *
 * Allocates the next free sector after @track/@sector and counts it in
 * the block count of the file.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_alloc(buffer_t *buf, uint8_t *track, uint8_t *sector) {
  uint8_t part = buf->pvt.d64.part;

  if (get_next_sector(part, track, sector) ||
      allocate_sector(part, *track, *sector))
    return 1;

  buf->pvt.d64.blocks++;
  return 0;
}

/**
 * rel_add_data_sector - append a data sector to a REL file
 * @buf: buffer of an open REL file
 *
 * Allocates a new data sector, fills it with the empty record pattern
 * (0xff as the first byte of every record, 0x00 elsewhere) and adds it
 * to the side sector structures. Every 120 data sectors a new side
 * sector block is created; all side sector blocks of a group carry the
 * same number in their byte 2 and share the pointer table at bytes
 * 4-15, and a new group is registered in the super side sector of
 * images that use one. Extends the file to a multiple of the record
 * length and updates the directory entry size.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_add_data_sector(buffer_t *buf) {
  uint8_t part = buf->pvt.d64.part;
  uint8_t length = buf->recordlen;
  /* With at most 65535 records of up to 254 bytes, the number of data
     sectors fits into 16 bits */
  uint16_t n      = ((uint32_t)buf->pvt.d64.record_max * length + 253) / 254;
  uint16_t ssnum  = n / SS_INDICES;           /* global side sector index */
  uint8_t  ingroup = ssnum % SS_SIDE_SECTORS; /* index within its group */
  uint8_t  group  = ssnum / SS_SIDE_SECTORS;
  uint32_t newmax = (uint32_t)(n + 1) * 254 / length;
  uint8_t t, s, prevt = 0, prevs = 0, tailt = 0, tails = 0;

  /* The record count is tracked in 16 bits, the super side sector holds
     at most SS_SUPER_GROUPS group entries and the images without a super
     side sector support at most six side sector blocks */
  if (newmax > 65535 ||
      ssnum >= ((buf->pvt.d64.relflags & REL_FLAG_SUPER) ?
                SS_SUPER_GROUPS * SS_SIDE_SECTORS : SS_SIDE_SECTORS)) {
    set_error(ERROR_FILE_TOO_LARGE);
    return 1;
  }

  /* Find the position for the new data sector */
  if (n != 0) {
    /* data sector n-1 is the last one allocated, so its side sector
       block is the tail of the chain */
    uint8_t *entry = rel_read_sidesector(buf, n - 1, &tailt, &tails);

    if (entry == NULL)
      return 1;

    prevt = entry[0];
    prevs = entry[1];
    t = prevt;
    s = prevs;
  } else {
    /* A file without data sectors, start the search at its side sector */
    t = buf->pvt.d64.sstrack;
    s = buf->pvt.d64.sssector;
  }

  if (rel_alloc(buf, &t, &s))
    return 1;

  if (n % SS_INDICES == 0 && n != 0) {
    /* A new side sector block is needed */
    uint8_t st, ss, i;
    uint8_t table[SS_SIDE_SECTORS * 2];

    st = t;
    ss = s;
    if (rel_alloc(buf, &st, &ss))
      return 1;

    /* Link the tail of the chain to the new block; this continues
       into the next group */
    if (rel_patch_ts(buf, tailt, tails, SS_OFS_NEXT_TRACK, st, ss))
      return 1;

    if (ingroup != 0) {
      /* Keep the pointer table of the group's first side sector, the
         new block shares it */
      if (rel_read_sidesector(buf, n - ingroup * SS_INDICES,
                              NULL, NULL) == NULL)
        return 1;

      memcpy(table, buf->data + SS_OFS_FIRST_SS, sizeof(table));
    }

    /* Initialize the new side sector; the first block of a group is
       its own head, later ones copy the group table of the first */
    rel_init_sidesector(buf, ingroup, st, ss, t, s);
    memcpy(buf->data + SS_OFS_FIRST_SS, table, 2 * ingroup);
    if (rel_write(part, st, ss, buf->data))
      return 1;

    if (ingroup == 0) {
      if (buf->pvt.d64.relflags & REL_FLAG_SUPER) {
        /* Register the new group in the super side sector */
        if (rel_patch_ts(buf, buf->pvt.d64.sstrack, buf->pvt.d64.sssector,
                         SS_OFS_SUPER_POINTER + 2 * group, st, ss))
          return 1;
      }
    } else {
      /* All previously existing side sectors of the group share the
         pointer table, add the new block to their tables too */
      for (i = 0; i < ingroup; i++)
        if (rel_patch_ts(buf, table[2 * i], table[2 * i + 1],
                         SS_OFS_FIRST_SS + 2 * ingroup, st, ss))
          return 1;
    }
  } else {
    /* Append the new data sector to an existing side sector */
    uint8_t sst, sss;
    uint8_t *entry = rel_read_sidesector(buf, n, &sst, &sss);

    if (entry == NULL)
      return 1;

    entry[0] = t;
    entry[1] = s;
    buf->data[SS_OFS_NEXT_SECTOR] = entry + 1 - buf->data;
    if (rel_write(part, sst, sss, buf->data))
      return 1;
  }

  /* Link the previous data sector to the new one, the first data
     sector is linked from the directory entry */
  if (n != 0) {
    uint8_t link[2];
    link[0] = t;
    link[1] = s;
    if (image_write(part, sector_offset(part, prevt, prevs), link, 2, 0))
      return 1;
  } else {
    if (read_entry(part, &buf->pvt.d64.dh, ops_scratch))
      return 1;
    ops_scratch[DIR_OFS_TRACK]  = t;
    ops_scratch[DIR_OFS_SECTOR] = s;
    if (write_entry(part, &buf->pvt.d64.dh, ops_scratch, 0))
      return 1;
  }

  /* Fill the new data sector with the empty record pattern */
  if (rel_write_empty(buf, n, t, s))
    return 1;

  buf->pvt.d64.record_max = newmax;

  return rel_update_dir(buf, 0);
}

/**
 * rel_grow - extend a REL file to hold a given record
 * @buf   : buffer of an open REL file
 * @target: record number to be available, 0-based
 *
 * Adds data sectors to the file until record @target exists.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_grow(buffer_t *buf, uint16_t target) {
  while (buf->pvt.d64.record_max <= target)
    if (rel_add_data_sector(buf))
      return 1;

  return 0;
}

/**
 * rel_fill_remainder - zero-fill the rest of the current record
 * @buf: buffer of an open REL file positioned inside a record
 *
 * Writes zeroes over the part of the current record that hasn't been
 * filled by the host, committing every touched sector to disk.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_fill_remainder(buffer_t *buf) {
  uint32_t end = rel_offset(buf, buf->pvt.d64.record + 1);
  /* a record holds at most 254 bytes */
  uint8_t left = (buf->fptr < end) ? (uint8_t)(end - buf->fptr) : 0;
  uint8_t first = 1;

  while (left) {
    uint8_t rest = rel_pos(buf->fptr) - 2;
    uint8_t len  = 254 - rest;

    if (len > left)
      len = left;

    /* buf->data holds the sector containing the position the transfer
       stopped at, unless the position sits exactly at a sector
       boundary - commit the pending sector and load the sector holding
       the fill range otherwise */
    if (!(first && rest != 0)) {
      if (rel_commit(buf))
        return 1;
      if (rel_load_fptr(buf))
        return 1;
    }
    first = 0;

    memset(buf->data + 2 + rest, 0, len);
    mark_buffer_dirty(buf);
    buf->fptr += len;
    left -= len;
  }

  /* Commit the sector holding the end of the record */
  if (rel_commit(buf))
    return 1;

  buf->mustflush = 0;
  return 0;
}

/**
 * rel_trim - find the amount of valid data in a record
 * @buf  : buffer holding the record data
 * @start: first buffer position of the remaining record part
 * @end  : last buffer position of the record
 *
 * CBM DOS determines the length of a record by scanning from its end
 * towards the start for the first non-zero byte. This function sets
 * lastused accordingly. Reading a record always terminates with EOI.
 * If the whole remaining part is zero-filled the bus layer still needs
 * a single byte to send, so lastused points at the first byte of the
 * record part, which is transmitted with EOI.
 */
static void rel_trim(buffer_t *buf, uint8_t start, uint8_t end) {
  uint8_t i = end;

  while (i > start && buf->data[i] == 0)
    i--;

  buf->lastused = buf->data[i] ? i : start;
  buf->sendeoi  = 1;
}

/**
 * rel_view - set up the read view for the current record
 * @buf: buffer with the record's start sector loaded and fptr set to
 *       the first byte to deliver
 *
 * Sets position, relstart, lastused and sendeoi for the part of the
 * record that starts at buf->fptr, scanning the record tail for
 * zero-filled bytes like CBM DOS does. Records extending into the next
 * data sector are scanned there as well, so a record whose tail is
 * completely zero terminates with EOI in this view. The current sector
 * must not hold uncommitted writes when the record spans two sectors.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t rel_view(buffer_t *buf) {
  /* fptr lies within the record, so at most 254 bytes remain */
  uint8_t rest = rel_offset(buf, buf->pvt.d64.record + 1) - buf->fptr;
  uint8_t tail;

  buf->position = rel_pos(buf->fptr);
  tail = 256 - buf->position;
  buf->pvt.d64.relstart = buf->position;

  if (rest <= tail) {
    rel_trim(buf, buf->position, buf->position + rest - 1);
    return 0;
  }

  /* The record continues in the next data sector; check whether its
     remaining part is zero-filled, in which case it ends in the
     current sector. */
  if (rel_load_data_sector(buf, buf->fptr / 254 + 1))
    return 1;

  {
    uint8_t continuation = rest - tail;
    uint8_t first; /* first byte of the continuation, zero means unused */

    while (continuation > 1 && buf->data[2 + continuation - 1] == 0)
      continuation--;
    first = buf->data[2];

    /* Restore the current sector before the trim scan */
    if (rel_load_fptr(buf))
      return 1;

    if (continuation == 1 && first == 0) {
      rel_trim(buf, buf->position, 255);
      return 0;
    }
  }

  buf->lastused = 255;
  buf->sendeoi  = 0;
  return 0;
}

/**
 * rel_present_eof - prepare the buffer to deliver an end-of-file marker
 * @buf: buffer of an open REL file
 *
 * A read past the end of the file delivers a single carriage return
 * with EOI. Error 50 is reported separately when the marker is
 * actually transferred, matching the 1541 which only reports it after
 * a read that ran past the end of the file.
 */
static void rel_present_eof(buffer_t *buf) {
  buf->data[2]     = 13;
  buf->lastused    = 2;
  buf->position    = 2;
  buf->sendeoi     = 1;
  buf->mustflush   = 0;
  buf->pvt.d64.relstart = 2;
}

/**
 * rel_present_eof_grow - present the end-of-file marker, allow growth
 * @buf: buffer of an open REL file
 *
 * Like rel_present_eof, but on a writable channel the next write
 * creates the current record.
 */
static void rel_present_eof_grow(buffer_t *buf) {
  rel_present_eof(buf);
  if (buf->write) {
    buf->pvt.d64.relflags |= REL_FLAG_GROW;
    buf->mustflush = 1;
  }
}

/**
 * rel_write_view - set up the buffer to receive record data at fptr
 * @buf: buffer of an open REL file, the sector holding fptr is loaded
 */
static void rel_write_view(buffer_t *buf) {
  buf->position = rel_pos(buf->fptr);
  buf->pvt.d64.relstart = buf->position;
  buf->lastused  = buf->position - 1;
  buf->sendeoi   = 0;
  buf->mustflush = 0;
}

/**
 * d64_rel_seek - seek-callback for REL files
 * @buf     : target buffer
 * @position: record number multiplied by the record length
 * @index   : position within the record, 0-based
 *
 * Positions the buffer at the given record. Seeking to a record that
 * doesn't exist yet sets error 50; on a writable channel the record
 * will be created by the next write (growth is deferred so that
 * seeking alone doesn't extend the file).
 * Returns 0 if successful, 1 if an error occured.
 */
static uint8_t d64_rel_seek(buffer_t *buf, uint32_t position, uint8_t index) {
  uint16_t record = position / buf->recordlen;

  /* Commit a pending record write, the 1541 does the same */
  if (buf->dirty && rel_fill_remainder(buf))
    return 1;

  buf->fptr = position + index;
  buf->pvt.d64.relflags &= (uint8_t)~(REL_FLAG_GROW | REL_FLAG_OVERFLOW);
  buf->mustflush = 0;

  buf->pvt.d64.record = record;

  if (record >= buf->pvt.d64.record_max) {
    /* Growth is deferred to the next write access */
    rel_present_eof_grow(buf);
    set_error(ERROR_RECORD_MISSING);
    return 0;
  }

  if (rel_load_fptr(buf))
    return 1;

  buf->mustflush = 0;

  /* A successful positioning command clears the error channel, like
     the 1541 does */
  set_error(ERROR_OK);

  /* Set up the read view of the record */
  return rel_view(buf);
}

/**
 * d64_rel_sync - refill-callback for REL files
 * @buf: target buffer
 *
 * This callback completes the current record and advances to the next
 * one. It is called by the bus layer when a record write is terminated
 * by EOI, when the sector holding the record is full or when a read has
 * consumed the current record. Writing past the end of the file
 * allocates and pattern-fills the required sectors.
 * Returns 0 if successful, 1 if an error occured.
 */
static uint8_t d64_rel_sync(buffer_t *buf) {
  uint8_t writing;
  uint32_t rec_end;

  if (buf->pvt.d64.relflags & REL_FLAG_OVERFLOW) {
    /* A previous write overflowed this record, everything the host
       writes until the transfer ends is junk. Discard it by restoring
       the sector from the image. The bus layer clears mustflush on the
       EOI that ends the transfer - when that refill arrives, resume
       normal operation at the record after the written one, with the
       growth still deferred to the next write. */
    if (rel_restore(buf, 0))
      return 1;
    mark_buffer_clean(buf);

    if (buf->mustflush) {
      /* the transfer is still running */
      rel_write_view(buf);
      return 0;
    }

    buf->pvt.d64.relflags &= (uint8_t)~REL_FLAG_OVERFLOW;
    /* The sector holds no position of the following record */
    buf->position = 0;
    goto next_record;
  }

  if ((buf->pvt.d64.relflags & REL_FLAG_GROW) &&
      buf->position == buf->pvt.d64.relstart) {
    /* Create the record that the host is about to write. Refills for
       read accesses always transfer at least one byte before entering
       this callback, so position == relstart means a write is pending. */
    buf->pvt.d64.relflags &= (uint8_t)~REL_FLAG_GROW;

    if (rel_grow(buf, buf->pvt.d64.record))
      return 1;

    if (rel_load_fptr(buf))
      return 1;

    rel_write_view(buf);
    mark_buffer_clean(buf);
    return 0;
  }

  /* Account for the bytes transferred since the last callback */
  buf->fptr += (buf->position == 0) ? (uint8_t)(256 - buf->pvt.d64.relstart)
                                    : (uint8_t)(buf->position - buf->pvt.d64.relstart);
  writing = buf->dirty;
  rec_end = rel_offset(buf, buf->pvt.d64.record + 1);

  if (buf->fptr > rec_end) {
    if (writing) {
      /* The host wrote past the end of the record. Restore the
         overwritten bytes of the following record from the image (they
         haven't been committed yet) and report error 51, like a 1541. */
      if (rel_restore(buf, rel_pos(rec_end)))
        return 1;
      set_error(ERROR_RECORD_OVERFLOW);
      /* If the transfer continues (mustflush set by the sector wrap),
         everything the host writes from now on is junk and dropped
         until the next seek or the end of the transfer */
      if (buf->mustflush)
        buf->pvt.d64.relflags |= REL_FLAG_OVERFLOW;
    }
    buf->fptr = rec_end;
  }

  if (buf->pvt.d64.record >= buf->pvt.d64.record_max) {
    /* Reading a record past the end of the file presents the
       end-of-file marker; error 50 is raised when it is transferred.
       The next write still creates the record because the growth
       request and mustflush are kept alive. */
    rel_present_eof(buf);
    set_error(ERROR_RECORD_MISSING);
    buf->mustflush = 1;
    buf->fptr = rel_offset(buf, buf->pvt.d64.record);
    return 0;
  }

  if (buf->fptr < rec_end && buf->position == 0 &&
      (writing ? buf->mustflush : !buf->sendeoi)) {
    /* The transfer wrapped at the sector boundary with bytes left in
       the record: a continuing write is flagged by mustflush, a
       continuing read by an unset EOI. The record continues in the
       next data sector. */
    if (rel_commit(buf))
      return 1;

    if (rel_load_fptr(buf))
      return 1;

    buf->position = 2;
    buf->pvt.d64.relstart = 2;
    buf->mustflush = 0;
    buf->lastused = 255;
    buf->sendeoi  = 0;

    if (!writing && rec_end - buf->fptr < 254)
      rel_trim(buf, 2, 2 + (uint8_t)(rec_end - buf->fptr) - 1);
    return 0;
  }

  /* The current record is complete (fully transferred, consumed by a
     read, or the transfer was terminated by EOI / the record data
     ended before the sector boundary) */
  if (writing) {
    if (rel_fill_remainder(buf))
      return 1;
  }

  buf->pvt.d64.record++;

 next_record:
  buf->fptr = rel_offset(buf, buf->pvt.d64.record);

  if (buf->pvt.d64.record >= buf->pvt.d64.record_max) {
    /* The next record does not exist: present the end-of-file marker
       without raising error 50 yet, it is only reported when the
       marker is transferred. On a writable channel keep the growth
       request alive so that the next write creates the record - a
       1541 does not allocate anything when the last record of a file
       is rewritten. */
    rel_present_eof_grow(buf);
    return 0;
  }

  /* Position at the next record, skipping the reload if the correct
     sector is already present */
  if (buf->position == 0 || buf->position != rel_pos(buf->fptr)) {
    if (rel_load_fptr(buf))
      return 1;
  }

  buf->mustflush = 0;

  /* Always set up a read view so that reading the record directly
     after a write delivers it correctly. */
  return rel_view(buf);
}

/**
 * d64_rel_cleanup - cleanup-callback for REL files
 * @buf: target buffer
 *
 * Completes a record left unfinished by the host, updates the size
 * field of the directory entry and frees the buffer.
 * Returns 0 if successful, 1 if an error occured.
 */
static uint8_t d64_rel_cleanup(buffer_t *buf) {
  if (buf->write) {
    if (buf->dirty && rel_fill_remainder(buf))
      return 1;

    if (rel_update_dir(buf, 1))
      return 1;
  }

  buf->cleanup = callback_dummy;
  free_buffer(buf);

  return 0;
}

/**
 * rel_calc_record_max - count the records in an existing REL file
 * @buf: buffer of a REL file, buf->data holds the first side sector block
 *
 * Walks the side sector chain to find the last data sector and derives
 * the number of complete records from the number of used bytes stored
 * in its link field, the same way a 1541 drive does it.
 * Returns the number of records.
 */
static uint16_t rel_calc_record_max(buffer_t *buf) {
  uint8_t part = buf->pvt.d64.part;
  uint8_t track, sector;
  uint16_t ssindex = 0;
  uint16_t entries;
  uint32_t lastsec, total;

  if (buf->pvt.d64.relflags & REL_FLAG_SUPER) {
    /* The chain starts with the first entry of the super side sector */
    track  = buf->data[SS_OFS_SUPER_POINTER];
    sector = buf->data[SS_OFS_SUPER_POINTER + 1];
    if (rel_read(part, track, sector, buf->data))
      return 0;
  }

  while (buf->data[SS_OFS_NEXT_TRACK] != 0) {
    track  = buf->data[SS_OFS_NEXT_TRACK];
    sector = buf->data[SS_OFS_NEXT_SECTOR];
    if (rel_read(part, track, sector, buf->data))
      return 0;
    if (++ssindex > SS_CHAIN_LIMIT) {
      /* cyclic or corrupt chain */
      set_error(ERROR_IMAGE_INVALID);
      return 0;
    }
  }

  entries = (buf->data[SS_OFS_NEXT_SECTOR] + 1 - SS_OFS_POINTER) / 2;
  if (entries == 0)
    return 0;

  /* Read the last data sector to determine its number of used bytes */
  lastsec = (uint32_t)ssindex * SS_INDICES + entries - 1;
  if (lastsec > 0xffff) {
    set_error(ERROR_IMAGE_INVALID);
    return 0;
  }
  if (rel_load_data_sector(buf, (uint16_t)lastsec))
    return 0;

  total = lastsec * 254;
  if (buf->data[SS_OFS_NEXT_SECTOR] >= 2)
    total += buf->data[SS_OFS_NEXT_SECTOR] - 1;

  total /= buf->recordlen;
  return (total > 0xffff) ? 0xffff : (uint16_t)total;
}

/**
 * d64_open_rel - open or create a relative file on a disk image
 * @path  : path of the file
 * @dent  : name of the file
 * @buf   : buffer to be used
 * @length: record length
 * @mode  : select between new or existing file
 *
 * This function opens a REL file in a D64/D71/D81/DNP image and prepares
 * it for record-wise access. If the mode parameter is 0, a new file is
 * created. New files are allocated one data sector and one side sector
 * block, existing files are located through the side sector chain stored
 * in the directory entry.
 */
static void d64_open_rel(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t length, uint8_t mode) {
  uint8_t part = path->part;
  uint8_t writable = partition[part].imagehandle.flag & FA_WRITE;
  uint8_t mismatch = 0;
  dh_t dh;
  uint8_t t, s, sts, sss;

  /* Existing files can be opened on read-only images, only creating
     requires a writable image */
  if (!mode && !writable) {
    set_error(ERROR_WRITE_PROTECT);
    return;
  }

  buf->recordlen     = length;
  buf->pvt.d64.part  = part;

  if (!mode) {
    /* Create a new REL file and allocate its first data sector */
    if (create_entry(path, dent, TYPE_REL | FLAG_SPLAT /* closed */, &dh, &t, &s))
      return;

    ops_scratch[DIR_OFS_RECORD_SIZE] = length;
    buf->pvt.d64.blocks = 1;

    /* Allocate the first side sector */
    sts = t;
    sss = s;
    if (rel_alloc(buf, &sts, &sss))
      return;

    ops_scratch[DIR_OFS_SIDESECTOR_TRACK]  = sts;
    ops_scratch[DIR_OFS_SIDESECTOR_SECTOR] = sss;

    /* D81 and DNP images use a super side sector, D41/D71 do not */
    if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_D81 ||
        (partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
      uint8_t spt, sps;

      buf->pvt.d64.relflags |= REL_FLAG_SUPER;
      spt = sts;
      sps = sss;
      if (rel_alloc(buf, &spt, &sps))
        return;

      memset(buf->data, 0, 256);
      buf->data[SS_OFS_NEXT_TRACK]  = sts;
      buf->data[SS_OFS_NEXT_SECTOR] = sss;
      buf->data[SS_OFS_NUMBER]      = REL_SUPER_MARKER;
      buf->data[SS_OFS_SUPER_POINTER]     = sts;
      buf->data[SS_OFS_SUPER_POINTER + 1] = sss;
      if (rel_write(part, spt, sps, buf->data))
        return;

      ops_scratch[DIR_OFS_SIDESECTOR_TRACK]  = spt;
      ops_scratch[DIR_OFS_SIDESECTOR_SECTOR] = sps;
    }

    /* Initialize the first side sector */
    rel_init_sidesector(buf, 0, sts, sss, t, s);
    if (rel_write(part, sts, sss, buf->data))
      return;

    /* Write the directory entry */
    update_timestamp(ops_scratch);
    if (write_entry(part, &dh.dir.d64, ops_scratch, 1))
      return;

    /* Fill the first data sector with the empty record pattern */
    if (rel_write_empty(buf, 0, t, s))
      return;

    buf->pvt.d64.dh        = dh.dir.d64;
    buf->pvt.d64.sstrack   = ops_scratch[DIR_OFS_SIDESECTOR_TRACK];
    buf->pvt.d64.sssector  = ops_scratch[DIR_OFS_SIDESECTOR_SECTOR];
    buf->pvt.d64.record_max = 254 / length;
  } else {
    /* Open an existing REL file */
    if (read_entry(part, &dent->pvt.dxx.dh, ops_scratch))
      return;

    mismatch = (length != 0 && length != ops_scratch[DIR_OFS_RECORD_SIZE]);

    length = ops_scratch[DIR_OFS_RECORD_SIZE];
    if (length == 0) {
      set_error(ERROR_IMAGE_INVALID);
      return;
    }

    sts = ops_scratch[DIR_OFS_SIDESECTOR_TRACK];
    sss = ops_scratch[DIR_OFS_SIDESECTOR_SECTOR];
    if (sts == 0) {
      set_error(ERROR_IMAGE_INVALID);
      return;
    }

    buf->pvt.d64.dh       = dent->pvt.dxx.dh;
    buf->pvt.d64.sstrack  = sts;
    buf->pvt.d64.sssector = sss;
    buf->pvt.d64.blocks   = ops_scratch[DIR_OFS_SIZE_LOW]
                          + 256 * ops_scratch[DIR_OFS_SIZE_HI];

    /* Detect a super side sector and count the records in the file */
    if (rel_read(part, sts, sss, buf->data))
      return;

    if (buf->data[SS_OFS_NUMBER] == REL_SUPER_MARKER)
      buf->pvt.d64.relflags |= REL_FLAG_SUPER;

    /* The error channel may still hold an error from a previous
       command, only errors raised by the walk below abort the open */
    buf->recordlen = length;
    set_error(ERROR_OK);
    buf->pvt.d64.record_max = rel_calc_record_max(buf);
    if (current_error != ERROR_OK)
      return;
  }

  /* Position at the first record */
  buf->read      = 1;
  buf->refill    = d64_rel_sync;
  buf->seek      = d64_rel_seek;
  buf->cleanup   = d64_rel_cleanup;
  buf->fptr      = 0;
  buf->position  = 2;
  buf->pvt.d64.record   = 0;
  buf->pvt.d64.relstart = 2;

  /* On a read-only image the file is opened for reading only, the
     write flag would make the cleanup rewrite the directory entry */
  if (writable)
    mark_write_buffer(buf);
  else
    stick_buffer(buf);

  if (buf->pvt.d64.record_max == 0) {
    /* Present an empty file without raising an error on open, the
       first write creates record 1 */
    rel_present_eof_grow(buf);
  } else {
    if (rel_load_fptr(buf) || rel_view(buf))
      return;
  }

  if (mismatch)
    /* Record length mismatch, the 1541 reports error 50 here too */
    set_error(ERROR_RECORD_MISSING);
}
#else
/* REL files on disk images are not supported in this build */
static void d64_open_rel(path_t *path, cbmdirent_t *dent, buffer_t *buf,
                         uint8_t length, uint8_t mode) {
  (void)path;
  (void)dent;
  (void)buf;
  (void)length;
  (void)mode;

  set_error(ERROR_SYNTAX_UNABLE);
}
#endif

static uint8_t d64_delete(path_t *path, cbmdirent_t *dent) {
  uint8_t linkbuf[2];
  uint32_t steps;
  uint8_t part = path->part;
#ifdef CONFIG_D64_REL
  uint8_t ssbuf[SS_OFS_SUPER_POINTER + 2];
  uint8_t sstrack, sssector, isrel;
#endif

  /* Read the directory entry of the file */
  if (read_entry(part, &dent->pvt.dxx.dh, ops_scratch))
    return 255;

#ifdef CONFIG_D64_REL
  isrel   = (ops_scratch[DIR_OFS_FILE_TYPE] & TYPE_MASK) == TYPE_REL;
  sstrack = ops_scratch[DIR_OFS_SIDESECTOR_TRACK];
  sssector = ops_scratch[DIR_OFS_SIDESECTOR_SECTOR];
#endif

  /* Free the sector chain in the BAM */
  linkbuf[0] = ops_scratch[DIR_OFS_TRACK];
  linkbuf[1] = ops_scratch[DIR_OFS_SECTOR];

  steps = 0;
  do {
    free_sector(part, linkbuf[0], linkbuf[1]);

    if (checked_read(part, linkbuf[0], linkbuf[1], linkbuf, 2, ERROR_ILLEGAL_TS_LINK))
      return 255;

    if (++steps > FILE_CHAIN_LIMIT) {
      /* cyclic or corrupt chain */
      set_error(ERROR_IMAGE_INVALID);
      return 255;
    }
  } while (linkbuf[0]);

#ifdef CONFIG_D64_REL
  /* REL files also reference side sectors, and on images with a super
     side sector the super side sector block itself. Read into ssbuf,
     ops_scratch still holds the directory entry for the update below. */
  if (isrel && sstrack != 0) {
    if (checked_read(part, sstrack, sssector, ssbuf, sizeof(ssbuf),
                     ERROR_ILLEGAL_TS_LINK))
      return 255;

    if (ssbuf[SS_OFS_NUMBER] == REL_SUPER_MARKER) {
      /* free the super side sector block and start the walk at the
         first group, its chain continues across all groups */
      free_sector(part, sstrack, sssector);
      linkbuf[0] = ssbuf[SS_OFS_SUPER_POINTER];
      linkbuf[1] = ssbuf[SS_OFS_SUPER_POINTER + 1];
    } else {
      linkbuf[0] = sstrack;
      linkbuf[1] = sssector;
    }

    steps = 0;
    do {
      free_sector(part, linkbuf[0], linkbuf[1]);

      if (checked_read(part, linkbuf[0], linkbuf[1], linkbuf, 2, ERROR_ILLEGAL_TS_LINK))
        return 255;

      if (++steps > SS_CHAIN_LIMIT) {
        /* cyclic or corrupt chain */
        set_error(ERROR_IMAGE_INVALID);
        return 255;
      }
    } while (linkbuf[0]);
  }
#endif

  /* Clear directory entry */
  ops_scratch[DIR_OFS_FILE_TYPE] = 0;
  if (write_entry(part, &dent->pvt.dxx.dh, ops_scratch, 1))
    return 255;

  return 1;
}

static void d64_read_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  checked_read(part, track, sector, buf->data, 256, ERROR_ILLEGAL_TS_COMMAND);
}

static void d64_write_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= sectors_per_track(part, track)) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND,track,sector);
  } else
    image_write(part, sector_offset(part,track,sector), buf->data, 256, 1);
}

static void d64_rename(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  uint8_t *ptr;

  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*newname) *ptr++ = *newname++;

  write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1);
}


/**
 * d64_raw_directory - open directory track as file
 * @buf: target buffer
 *
 * This function opens the directory track as a file on the
 * buffer passed in buf. Used when $ is opened on a secondary
 * address > 0.
 */
void d64_raw_directory(path_t *path, buffer_t *buf) {
  /* Copy&Waste from d64_open_read */
  buf->data[0] = path->dir.dxx.track;
  if (partition[path->part].imagetype == D64_TYPE_DNP)
    buf->data[1] = path->dir.dxx.sector;
  else
    buf->data[1] = 0;

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

/**
 * d64_chdir - chdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be changed into, NULL to move one directory
 *           up (left arrow) - unmounts the image at the root
 *
 * Changes the directory in the path object for DNP files,
 * returns an error for everything else.
 * Returns 0 if successful, 1 otherwise.
 */
static uint8_t d64_chdir(path_t *path, cbmdirent_t *dirname) {
  if (partition[path->part].imagetype != D64_TYPE_DNP)
    return image_chdir(path,dirname);

  if (dirname == NULL) {
    /* Move up a directory, unmount if at the root */
    uint8_t parent[2];

    if (image_read(path->part,
                   sector_offset(path->part, path->dir.dxx.track, path->dir.dxx.sector) + DNP_DIRHEADER_PARENTHDR_TRACK,
                   parent, 2))
      return 1;

    if (parent[0] == 0)
      /* Already at the root directory */
      return image_unmount(path->part);

    path->dir.dxx.track  = parent[0];
    path->dir.dxx.sector = parent[1];
    return 0;
  }

  if (dirname->name[0] == 0) {
    /* Empty string: root directory */
    path->dir.dxx.track  = 1;
    path->dir.dxx.sector = 1;
    return 0;
  }

  if (read_entry(path->part, &dirname->pvt.dxx.dh, ops_scratch))
    return 1;

  path->dir.dxx.track  = ops_scratch[DIR_OFS_TRACK];
  path->dir.dxx.sector = ops_scratch[DIR_OFS_SECTOR];

  return 0;
}

/**
 * d64_mkdir - mkdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be created
 *
 * Creates a subdirectory for DNP files, returns an error
 * for everything else.
 */
static void d64_mkdir(path_t *path, uint8_t *dirname) {
  dh_t dh;
  buffer_t *buf;
  uint8_t h_track,h_sector,d_track,d_sector;
  uint8_t *ptr, *name;

  if (partition[path->part].imagetype != D64_TYPE_DNP) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  buf = alloc_buffer();
  if (buf == NULL)
    return;

  /* Find an empty directory entry */
  if (find_empty_entry(path, &dh))
    return;

  /* Find a free track/sector for the dir header */
  h_track  = path->dir.dxx.track;
  h_sector = path->dir.dxx.sector;

  if (get_next_sector(path->part, &h_track, &h_sector)) {
    set_error(ERROR_DISK_FULL);
    return;
  }

  /* Allocate it now so the next call to get_next doesn't return it again */
  if (allocate_sector(path->part, h_track, h_sector))
    return;

  /* Find a free track/sector for the first directory sector */
  d_track  = h_track;
  d_sector = h_sector;

  if (get_next_sector(path->part, &d_track, &d_sector)) {
    set_error(ERROR_DISK_FULL);
    free_sector(path->part, h_track, h_sector);
    return;
  }

  if (allocate_sector(path->part, d_track, d_sector))
    return;

  /* Build a directory header */
  memset(buf->data, 0, 256);
  memset(buf->data + DNP_LABEL_OFFSET, 0xa0, DNP_LABEL_AREA_SIZE);

  /* Unfortunately this generates much smaller code on AVR */
  ptr = buf->data;
  *ptr++ = d_track;
  *ptr++ = d_sector;
  *ptr++ = 'H';

  /* Note: The CMD FD manual says these bytes point to the root header, */
  /*       but all observed DNP files actually store the t/s of the     */
  /*       sector itself here.                                          */
  ptr = buf->data + DNP_DIRHEADER_ROOTHDR_TRACK;
  *ptr++ = h_track;
  *ptr++ = h_sector;

  *ptr++ = path->dir.dxx.track;
  *ptr++ = path->dir.dxx.sector;

  *ptr++ = dh.dir.d64.track;
  *ptr++ = dh.dir.d64.sector;
  *ptr++ = dh.dir.d64.entry * 32 + 2;

  /* Fill name and ID */
  ptr = buf->data + DNP_LABEL_OFFSET;
  name = dirname;
  while (*name) *ptr++ = *name++;

  ptr = buf->data + DNP_ID_OFFSET;
  *ptr++ = dirname[0];
  *ptr++ = dirname[1];
   ptr++;
  *ptr++ = '1';
  *ptr++ = 'H';

  /* Write directory header sector */
  if (image_write(path->part,
                  sector_offset(path->part, h_track, h_sector),
                  buf->data, 256, 0))
    return;

  /* Create empty directory sector */
  memset(buf->data, 0, 256);
  buf->data[1] = 0xff;
  if (image_write(path->part,
                  sector_offset(path->part, d_track, d_sector),
                  buf->data, 256, 0))
    return;

  /* Create the directory entry */
  /* FIXME: Isn't similiar code duplicated in open_write and rename? */
  memset(ops_scratch + 2, 0, sizeof(ops_scratch) - 2);
  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);

  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*dirname) *ptr++ = *dirname++;

  ops_scratch[DIR_OFS_FILE_TYPE] = TYPE_DIR | FLAG_SPLAT;
  ops_scratch[DIR_OFS_TRACK]     = h_track;
  ops_scratch[DIR_OFS_SECTOR]    = h_sector;
  ops_scratch[DIR_OFS_SIZE_LOW]  = 2;
  update_timestamp(ops_scratch);

  image_write(path->part, sector_offset(path->part, dh.dir.d64.track, dh.dir.d64.sector)
                          + dh.dir.d64.entry * 32 + 2, ops_scratch + 2, 30, 1);
}

/**
 * d64_invalidate - invalidate internal state
 *
 * This function invalidates all cached state when
 * a card change is detected.
 */
void d64_invalidate(void) {
  free_buffer(bam_buffer);
  bam_buffer   = NULL;
  free_buffer(bam_buffer2);
  bam_buffer2  = NULL;
  bam_refcount = 0;
}

/**
 * d64_unmount - unmount disk image
 * @part: partition number
 *
 * This function in called on image unmount and handles
 * refcounting for the BAM buffers.
 */
void d64_unmount(uint8_t part) {
  /* invalidate BAM buffers that point to the current partition */
  if (bam_buffer) {
    bam_buffer->cleanup(bam_buffer);
    if (bam_buffer->pvt.bam.part == part)
      bam_buffer->pvt.bam.part = 255;
  }

  if (bam_buffer2) {
    bam_buffer2->cleanup(bam_buffer2);
    if (bam_buffer2->pvt.bam.part == part)
      bam_buffer2->pvt.bam.part = 255;
  }

  /* decrease BAM buffer refcounter - it can never be zero while a Dxx is mounted*/
  if (--bam_refcount) {
    free_buffer(bam_buffer);
    free_buffer(bam_buffer2);
    bam_buffer  = NULL;
    bam_buffer2 = NULL;
  }
}


/* ------------------------------------------------------------------------- */
/*  Formatting disk images                                                   */
/* ------------------------------------------------------------------------- */

/* create a 1581/DNP BAM signature */
static void format_add_bam_signature(uint8_t doschar, uint8_t *idbuf) {
  uint8_t *ptr = bam_buffer->data + 2;

  *ptr++ = doschar;
  *ptr++ = doschar ^ 0xff;
  *ptr++ = idbuf[0];
  *ptr++ = idbuf[1];
  *ptr++ = 0xc0;
}

/* copy name and id into some buffer */
static void format_copy_label(uint8_t part, uint8_t *data, uint8_t *name, uint8_t *idbuf) {
  uint8_t *ptr;
  uint8_t count;

  ptr = data + get_param(part, LABEL_OFFSET);
  memset(ptr, 0xa0, 25); /* 25=name+id+dos+spaces+padding */

  count = 16;
  while (*name && count--)
    *ptr++ = *name++;

  memcpy(data + get_param(part, ID_OFFSET), idbuf, 5);
}

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* allocate BAM and first directory sector */
  for (uint8_t s=0; s<2; s++)
    allocate_sector(part, D41_BAM_TRACK, s);

  /* 18/0 is now available via bam_buffer */
  uint8_t *ptr = bam_buffer->data;
  *ptr++ = 18;
  *ptr++ = 1;
  *ptr++ = 0x41;

  /* copy disk label and ID */
  idbuf[3] = '2';
  idbuf[4] = 'A';
  format_copy_label(part, bam_buffer->data, name, idbuf);
  /* two additional 0xa0 characters on 5.25" disks */
  bam_buffer->data[0xa9] = 0xa0;
  bam_buffer->data[0xaa] = 0xa0;

  clear_dir_sector(part, D41_BAM_TRACK, 1, buf->data);
}

static void format_d71_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* almost everything is the same as D41 */
  format_d41_image(part, buf, name, idbuf);

  /* add double-sided marker in 18/0 (still in bam_buffer) */
  bam_buffer->data[3] = 0x80;

  /* allocate all of track 53 */
  for (uint8_t s=0; s<19; s++)
    allocate_sector(part, D71_BAM2_TRACK, s);
}

static void format_d81_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* allocate 40/0 to 40/3 */
  for (uint8_t s=0; s<4; s++)
    allocate_sector(part, D81_BAM_TRACK, s);

  /* bam_buffer buffer now holds 40/1 */
  bam_buffer->data[0] = 40;
  bam_buffer->data[1] = 2;
  format_add_bam_signature('D', idbuf);
  // mustflush was already set by allocate_sector

  /* switch bam_buffer to 40/2 */
  (void)sectors_free(part, 41);
  bam_buffer->data[0] = 0;
  bam_buffer->data[1] = 0xff;
  format_add_bam_signature('D', idbuf);
  bam_buffer->mustflush = 1;

  /* build contents of 40/0 */
  uint8_t *ptr = buf->data;
  *ptr++ = 40;
  *ptr++ = 3;
  *ptr++ = 'D';

  /* copy disk label and ID to buffer */
  idbuf[3] = '3';
  idbuf[4] = 'D';
  format_copy_label(part, buf->data, name, idbuf);

  /* write 40/0 */
  if (image_write(part, /*sector_offset(part, 40, 0)*/ (40-1)*40*256L,
                  buf->data, 256, 0))
    return;

  clear_dir_sector(part, D81_BAM_TRACK, 3, buf->data);
}

static void format_dnp_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* Note: Formatting is only accepted while in the root dir, so the
           d64param struct doesn't need to be changed */
  for (uint8_t s=0; s<35; s++)
    allocate_sector(part, DNP_BAM_TRACK, s);

  /* add BAM signature - first BAM sector is in bam_buffer because of allocate_sector */
  format_add_bam_signature('H', idbuf);
  bam_buffer->data[DNP_BAM_LAST_TRACK_OFS] = get_param(part, LAST_TRACK);

  /* build root dirheader */
  uint8_t *ptr = buf->data;
  memset(ptr, 0, 256);
  *ptr++ = 1;
  *ptr++ = 34;
  *ptr++ = 'H';
  idbuf[3] = '1';
  idbuf[4] = 'H';
  format_copy_label(part, buf->data, name, idbuf);
  buf->data[DNP_DIRHEADER_ROOTHDR_TRACK]  = 1;
  buf->data[DNP_DIRHEADER_ROOTHDR_SECTOR] = 1;

  /* write 1/1 */
  if (image_write(part, /*sector_offset(part, 1, 1)*/ 256,
                  buf->data, 256, 0))
    return;

  clear_dir_sector(part, 1, DNP_ROOTDIR_SECTOR, buf->data);
}

static void d64_format(uint8_t part, uint8_t *name, uint8_t *id) {
  buffer_t *buf;
  uint8_t  idbuf[5];
  uint16_t t;
  uint16_t  s;

  /* allow format on DNP only when in the root directory */
  if (partition[part].imagetype == D64_TYPE_DNP &&
      (partition[part].current_dir.dxx.track  != 1 ||
       partition[part].current_dir.dxx.sector != 1)) {
    /* just ignore, CMD HD doesn't return an error either */
    return;
  }

  /* grab a buffer as work area */
  buf = alloc_buffer();
  if (buf == NULL)
    return;

  mark_write_buffer(buf);
  unstick_buffer(buf);
  mark_buffer_dirty(buf);
  memset(buf->data, 0, 256);

  /* Flush BAM buffers and mark their contents as invalid */
  d64_bam_commit();
  bam_buffer->pvt.bam.part = 0xff;
  if (bam_buffer2)
    bam_buffer2->pvt.bam.part = 0xff;

  if (id != NULL) {
    /* Clear the data area of the disk image */
    for (t=1; t<=get_param(part, LAST_TRACK); t++) {
      for (s=0; s<sectors_per_track(part, t); s++) {
        if (image_write(part, sector_offset(part, t, s),
                        buf->data, 256, 0))
          return;
      }
    }

    /* Copy the new ID into the buffer */
    idbuf[0] = id[0];
    idbuf[1] = id[1];
  } else {
    /* Read the old ID into the buffer */
    path_t path;
    path.part = part;
    path.dir.dxx.track  = get_param(part, DIR_TRACK);
    path.dir.dxx.sector = 1; // only relevant for DNP
    if (d64_getid(&path, idbuf))
      return;

    /* clear the entire directory track */
    /* This is not accurate, but I do not care. */
    t = get_param(part, DIR_TRACK);
    for (s=0; s < sectors_per_track(part, t); s++) {
      if (image_write(part, sector_offset(part, t, s),
                      buf->data, 256, 0))
        return;
    }
  }
  idbuf[2] = 0xa0;

  /* Mark all sectors as free */
  for (t=1; t<=get_param(part, LAST_TRACK); t++) {
    for (s=0; s<sectors_per_track(part, t); s++)
      free_sector(part,t,s);
  }

  /* call imagetype-specific format function */
  partition[part].d64data.format_function(part, buf, name, idbuf);

  /* FIXME: Clear the error info block */
}


/* ------------------------------------------------------------------------- */
/*  ops struct                                                               */
/* ------------------------------------------------------------------------- */

const PROGMEM fileops_t d64ops = {
  d64_open_read,
  d64_open_write,
  d64_open_rel,
  d64_delete,
  d64_getdisklabel,
  d64_getdirlabel,
  d64_getid,
  d64_freeblocks,
  d64_read_sector,
  d64_write_sector,
  d64_format,
  d64_opendir,
  d64_readdir,
  d64_mkdir,
  d64_chdir,
  d64_rename
};
