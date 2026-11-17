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


   arch-eeprom.h: EEPROM access functions (rppico version)

*/

#ifndef ARCH_EEPROM_H
#define ARCH_EEPROM_H

/* EEPROM structure hack, section starts at 0 via scripts/rppico/eeprom.ld */
#define EEMEM __attribute__((section(".eeprom")))

/* The EEPROM is emulated in RAM and written to flash later, */
/* eeprom_safety() makes sure the flash copy is up to date.  */
void eeprom_flush(void);
#define eeprom_safety() eeprom_flush()

/* Write the flash copy if the RAM copy has been unchanged for a while */
void eeprom_idle(void);

uint8_t  eeprom_read_byte(void *addr);
uint16_t eeprom_read_word(void *addr);
void     eeprom_read_block(void *destptr, void *addr, unsigned int length);
void     eeprom_write_byte(void *addr, uint8_t value);
void     eeprom_write_word(void *addr, uint16_t value);
void     eeprom_write_block(void *srcptr, void *addr, unsigned int length);

#endif
