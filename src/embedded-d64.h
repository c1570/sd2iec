/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2026  Ingo Korb <ingo@akana.de>

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


   embedded-d64.h: Built-in read-only D64 image for debugging
*/

#ifndef EMBEDDED_D64_H
#define EMBEDDED_D64_H

#include <stdint.h>
#include "config.h"

#ifdef CONFIG_EMBEDDED_D64

/* Non-zero while the built-in image is the active disk */
extern uint8_t embedded_d64_active;

/* Read from the built-in image, same return values as image_read */
uint8_t embedded_d64_read(uint32_t offset, void *buffer, uint16_t bytes);

/* Mount the built-in image as partition 0 */
void embedded_d64_mount(uint8_t preserve_path);

#endif

#endif
