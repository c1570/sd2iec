/* This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   arch-eeprom.c: EEPROM emulation in flash (rppico version)

*/

/*
 * The EEPROM contents are kept in RAM. Writes mark the copy dirty,
 * it is written to flash by eeprom_flush (via eeprom_safety, used
 * after writing the configuration) or by eeprom_idle when the bus
 * has been idle for a while after the last change.
 *
 * Two banks at the end of flash alternate, each holds the data
 * followed by a footer page with a sequence number and a CRC. The
 * footer is programmed last, so an interrupted update leaves the
 * previous bank as the newest valid one.
 *
 * The whole firmware runs from RAM, so interrupts can stay enabled
 * while the flash is erased and programmed.
 */

#include <string.h>
#include "config.h"
#include "pico.h"
#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "crc.h"
#include "timer.h"
#include "arch-eeprom.h"

#define FOOTER_MAGIC 0x45453253 // "S2EE"

/* bank layout: data, then one sector holding the footer page */
#define BANK_SIZE   (EEPROM_SIZE + FLASH_SECTOR_SIZE)
#define BANK_OFFSET(bank) (PICO_FLASH_SIZE_BYTES - (2 - (bank)) * BANK_SIZE)

_Static_assert(EEPROM_SIZE % FLASH_SECTOR_SIZE == 0,
               "EEPROM_SIZE must be a multiple of the flash sector size");

typedef union {
  struct {
    uint32_t magic;
    uint32_t sequence;
    uint16_t crc;
  };
  uint8_t page[FLASH_PAGE_SIZE];
} footer_t;

/* delay between the last change and writing to flash when idle */
#define FLUSH_DELAY (HZ)

static uint8_t eeprom_data[EEPROM_SIZE];
static footer_t footer;
static unsigned int current_bank;
static bool loaded;
static bool dirty;
static tick_t last_change;

static const void *flash_ptr(uint32_t offset) {
  /* uncached alias, avoids stale data after programming */
  return (const void *)(XIP_NOCACHE_NOALLOC_BASE + offset);
}

/* returns the sequence number of a bank if it is valid */
static bool bank_valid(unsigned int bank, uint32_t *sequence) {
  const footer_t *f = flash_ptr(BANK_OFFSET(bank) + EEPROM_SIZE);

  if (f->magic != FOOTER_MAGIC)
    return false;

  if (crc_xmodem_block(0, flash_ptr(BANK_OFFSET(bank)), EEPROM_SIZE) != f->crc)
    return false;

  *sequence = f->sequence;
  return true;
}

/* load the newest valid bank into RAM */
static void eeprom_load(void) {
  uint32_t seq0 = 0, seq1 = 0;
  bool valid0 = bank_valid(0, &seq0);
  bool valid1 = bank_valid(1, &seq1);

  loaded = true;
  footer.sequence = 0;

  if (valid0 && (!valid1 || (int32_t)(seq0 - seq1) > 0))
    current_bank = 0;
  else if (valid1)
    current_bank = 1;
  else {
    /* nothing stored yet: behave like an erased EEPROM */
    memset(eeprom_data, 0xff, sizeof(eeprom_data));
    current_bank = 1;
    return;
  }

  memcpy(eeprom_data, flash_ptr(BANK_OFFSET(current_bank)), EEPROM_SIZE);
  footer.sequence = current_bank ? seq1 : seq0;
}

/* returns a pointer into the RAM copy, NULL if out of range */
static uint8_t *eeprom_ptr(void *addr, unsigned int length) {
  uintptr_t offset = (uintptr_t)addr;

  if (!loaded)
    eeprom_load();

  if (offset + length > EEPROM_SIZE)
    return NULL;

  return eeprom_data + offset;
}

void eeprom_flush(void) {
  unsigned int bank;

  if (!dirty)
    return;

  bank = !current_bank;

  footer.magic    = FOOTER_MAGIC;
  footer.sequence++;
  footer.crc      = crc_xmodem_block(0, eeprom_data, EEPROM_SIZE);

  flash_range_erase(BANK_OFFSET(bank), BANK_SIZE);
  flash_range_program(BANK_OFFSET(bank), eeprom_data, EEPROM_SIZE);
  flash_range_program(BANK_OFFSET(bank) + EEPROM_SIZE, footer.page, FLASH_PAGE_SIZE);

  current_bank = bank;
  dirty = false;
}

void eeprom_idle(void) {
  if (dirty && time_after(getticks(), last_change + FLUSH_DELAY))
    eeprom_flush();
}

uint8_t eeprom_read_byte(void *addr) {
  uint8_t val = 0xff;

  eeprom_read_block(&val, addr, 1);
  return val;
}

uint16_t eeprom_read_word(void *addr) {
  uint16_t val = 0xffff;

  eeprom_read_block(&val, addr, 2);
  return val;
}

void eeprom_read_block(void *destptr, void *addr, unsigned int length) {
  uint8_t *ptr = eeprom_ptr(addr, length);

  if (ptr)
    memcpy(destptr, ptr, length);
}

void eeprom_write_byte(void *addr, uint8_t value) {
  eeprom_write_block(&value, addr, 1);
}

void eeprom_write_word(void *addr, uint16_t value) {
  eeprom_write_block(&value, addr, 2);
}

void eeprom_write_block(void *srcptr, void *addr, unsigned int length) {
  uint8_t *ptr = eeprom_ptr(addr, length);

  if (!ptr || !memcmp(ptr, srcptr, length))
    return;

  memcpy(ptr, srcptr, length);
  dirty = true;
  last_change = getticks();
}
