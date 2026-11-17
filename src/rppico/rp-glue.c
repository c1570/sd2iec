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


   rp-glue.c: Pico SDK setup calls that clash with sd2iec names

*/

#include "config.h"

/* see arch-config.h */
#undef spi_init
#undef uart_init

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "rp-glue.h"

void rp_spi_setup(unsigned int baudrate) {
  spi_init(SPI_INSTANCE(SD_SPI_NUM), baudrate);
  spi_set_format(SPI_INSTANCE(SD_SPI_NUM), 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}

void rp_spi_set_baudrate(unsigned int baudrate) {
  spi_set_baudrate(SPI_INSTANCE(SD_SPI_NUM), baudrate);
}

#ifdef CONFIG_UART_DEBUG
void rp_uart_setup(unsigned int baudrate) {
  uart_init(UART_INSTANCE(DEBUG_UART_NUM), baudrate);
  uart_set_format(UART_INSTANCE(DEBUG_UART_NUM), 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_INSTANCE(DEBUG_UART_NUM), true);
}
#endif
