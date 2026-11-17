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


   crc.h: CRC functions (rppico version)

*/

#ifndef CRC_H
#define CRC_H

#include <stdint.h>

uint8_t crc7update(uint8_t crc, uint8_t data);
uint16_t crc_xmodem_update(uint16_t crc, uint8_t data);
uint16_t crc_xmodem_block(uint16_t crc, const uint8_t *data, uint32_t length);
uint16_t crc16_update(uint16_t crc, uint8_t data);

#endif
