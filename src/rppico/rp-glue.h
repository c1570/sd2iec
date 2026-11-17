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


   rp-glue.h: Pico SDK setup calls that clash with sd2iec names

*/

#ifndef RP_GLUE_H
#define RP_GLUE_H

/*
 * The SDK headers hardware/spi.h and hardware/uart.h declare
 * functions with the same names as sd2iec (spi_init, uart_init,
 * uart_putc, ...), so they are only included in rp-glue.c. The
 * linker symbols of spi_init and uart_init are renamed for sd2iec
 * in arch-config.h.
 */

/* Reset and configure the SD card SPI for 8 bit, mode 0 */
void rp_spi_setup(unsigned int baudrate);

/* Change the SD card SPI clock */
void rp_spi_set_baudrate(unsigned int baudrate);

/* Reset and configure the debug UART for 8n1 with FIFOs */
void rp_uart_setup(unsigned int baudrate);

#endif
