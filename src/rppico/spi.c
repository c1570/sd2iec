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


   spi.c: Low-level SPI routines (rppico version)

*/

/*
 * Direct register access to the ARM PL022 SPI block. hardware/spi.h
 * is not used here because its function names clash, see rp-glue.h.
 */

#include "config.h"
#include "hardware/structs/spi.h"
#include "rp-glue.h"
#include "spi.h"

#define SPI_HW ((spi_hw_t *)(SD_SPI_NUM ? SPI1_BASE : SPI0_BASE))

static unsigned int speed_to_baudrate(spi_speed_t speed) {
  if (speed == SPI_SPEED_FAST)
    return SPI_FREQ_FAST;
  else
    return SPI_FREQ_SLOW;
}

void spi_init(spi_speed_t speed) {
  rp_spi_setup(speed_to_baudrate(speed));
}

void spi_tx_byte(uint8_t data) {
  /* Wait until TX fifo can accept data */
  while (!(SPI_HW->sr & SPI_SSPSR_TNF_BITS)) ;

  /* Send byte */
  SPI_HW->dr = data;
}

/* Wait until SPI is idle and clear the RX fifo */
static void spi_flush(void) {
  while (SPI_HW->sr & SPI_SSPSR_BSY_BITS) ;

  while (SPI_HW->sr & SPI_SSPSR_RNE_BITS)
    (void) SPI_HW->dr;
}

uint8_t spi_rx_byte(void) {
  spi_flush();

  /* Transmit a single dummy byte */
  SPI_HW->dr = 0xff;

  /* Wait until answer has been received */
  while (!(SPI_HW->sr & SPI_SSPSR_RNE_BITS)) ;

  return SPI_HW->dr;
}

void spi_tx_block(const void *ptr, unsigned int length) {
  const uint8_t *data = (const uint8_t *)ptr;

  while (length--) {
    /* Wait until TX fifo can accept data */
    while (!(SPI_HW->sr & SPI_SSPSR_TNF_BITS)) ;

    SPI_HW->dr = *data++;
  }
}

void spi_rx_block(void *ptr, unsigned int length) {
  uint8_t *data = (uint8_t *)ptr;
  unsigned int txlen = length;

  spi_flush();

  while (length > 0) {
    /* Keep the TX fifo ahead, but never more than the RX fifo can hold */
    if (txlen > 0 && length - txlen < 8 &&
        (SPI_HW->sr & SPI_SSPSR_TNF_BITS)) {
      SPI_HW->dr = 0xff;
      txlen--;
    }

    if (SPI_HW->sr & SPI_SSPSR_RNE_BITS) {
      *data++ = SPI_HW->dr;
      length--;
    }
  }
}

void spi_set_speed(spi_speed_t speed) {
  /* Wait until TX fifo is empty */
  while (!(SPI_HW->sr & SPI_SSPSR_TFE_BITS)) ;

  rp_spi_set_baudrate(speed_to_baudrate(speed));
}

void spi_select_device(spi_device_t dev) {
  /* Wait until the last byte has left the TX shift register */
  while (SPI_HW->sr & SPI_SSPSR_BSY_BITS) ;

  if (dev == SPIDEV_CARD0 || dev == SPIDEV_ALLCARDS)
    sdcard_set_ss(0);
  else
    sdcard_set_ss(1);
}
