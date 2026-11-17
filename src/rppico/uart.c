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


   uart.c: UART access routines (rppico version)

*/

/*
 * hardware/uart.h is not used here because its function names clash,
 * see rp-glue.h.
 */

#include "config.h"
#include "hardware/irq.h"
#include "hardware/structs/uart.h"
#include "rp-glue.h"
#include "uart.h"

#if DEBUG_UART_NUM == 0
#  define UART_HW      uart0_hw
#  define UART_IRQ     UART0_IRQ
#  define UART_HANDLER isr_uart0
#elif DEBUG_UART_NUM == 1
#  define UART_HW      uart1_hw
#  define UART_IRQ     UART1_IRQ
#  define UART_HANDLER isr_uart1
#else
#  error DEBUG_UART_NUM is not set or has an invalid value!
#endif

static char txbuf[1 << CONFIG_UART_TX_BUF_SHIFT];
static volatile unsigned int read_idx,write_idx;

void UART_HANDLER(void) {
  /* refill the TX fifo */
  while (read_idx != write_idx && !(UART_HW->fr & UART_UARTFR_TXFF_BITS)) {
    UART_HW->dr = (unsigned char)txbuf[read_idx];
    read_idx = (read_idx+1) & (sizeof(txbuf)-1);
  }

  if (read_idx == write_idx) {
    /* buffer empty - turn off TX interrupt */
    UART_HW->imsc &= ~UART_UARTIMSC_TXIM_BITS;
    UART_HW->icr = UART_UARTICR_TXIC_BITS;
  }
}

void uart_putc(char c) {
  unsigned int tmp = (write_idx+1) & (sizeof(txbuf)-1) ;

  if (read_idx == write_idx && !(UART_HW->fr & UART_UARTFR_TXFF_BITS)) {
    /* buffer empty, TX fifo not full -> send immediately */
    UART_HW->dr = (unsigned char)c;
  } else {
#ifdef CONFIG_DEADLOCK_ME_HARDER
    while (tmp == read_idx) ;
#endif
    UART_HW->imsc &= ~UART_UARTIMSC_TXIM_BITS; // turn off UART interrupt
    txbuf[write_idx] = c;
    write_idx = tmp;
    UART_HW->imsc |= UART_UARTIMSC_TXIM_BITS;
  }
}

/* Polling version only */
unsigned char uart_getc(void) {
  /* wait for character */
  while (UART_HW->fr & UART_UARTFR_RXFE_BITS) ;

  return UART_HW->dr;
}

void uart_init(void) {
  rp_uart_setup(CONFIG_UART_BAUDRATE);

  /* the TX interrupt is enabled when the buffer is used */
  irq_set_enabled(UART_IRQ, true);

  /* connect to I/O pins */
  uart_pins_connect();
}

/* --- generic code below --- */
void uart_puthex(uint8_t num) {
  uint8_t tmp;
  tmp = (num & 0xf0) >> 4;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);

  tmp = num & 0x0f;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);
}

void uart_trace(void *ptr, uint16_t start, uint16_t len) {
  uint16_t i;
  uint8_t j;
  uint8_t ch;
  uint8_t *data = ptr;

  data+=start;
  for(i=0;i<len;i+=16) {

    uart_puthex(start>>8);
    uart_puthex(start&0xff);
    uart_putc('|');
    uart_putc(' ');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data + j);
        uart_puthex(ch);
      } else {
        uart_putc(' ');
        uart_putc(' ');
      }
      uart_putc(' ');
    }
    uart_putc('|');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data++);
        if(ch<32 || ch>0x7e)
          ch='.';
        uart_putc(ch);
      } else {
        uart_putc(' ');
      }
    }
    uart_putc('|');
    uart_putcrlf();
    start+=16;
  }
}

void uart_flush(void) {
  while (read_idx != write_idx) ;
}

void uart_puts(const char *text) {
  while (*text) {
    uart_putc(*text++);
  }
}

void uart_puts_P(const char *text) {
  uart_puts(text);
}

void uart_putcrlf(void) {
  uart_putc(13);
  uart_putc(10);
}
