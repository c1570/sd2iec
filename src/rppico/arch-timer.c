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


   arch-timer.c: Architecture-specific timer functions (rppico version)

*/

#include "config.h"
#include "hardware/exception.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/timer.h"
#include "hardware/timer.h"
#include "timer.h"

static uint32_t timeout_end;

void timer_init(void) {
  /* SysTick on the processor clock, HZ interrupts per second */
  exception_set_priority(SYSTICK_EXCEPTION, PICO_LOWEST_IRQ_PRIORITY);
  systick_hw->rvr = CONFIG_MCU_FREQ / HZ - 1;
  systick_hw->cvr = 0;
  systick_hw->csr = M33_SYST_CSR_CLKSOURCE_BITS |
                    M33_SYST_CSR_TICKINT_BITS   |
                    M33_SYST_CSR_ENABLE_BITS;
}

void delay_us(unsigned int time) {
  busy_wait_us_32(time);
}

void delay_ms(unsigned int time) {
  busy_wait_ms(time);
}

/**
 * start_timeout - start a timeout
 * @usecs: number of microseconds before timeout
 *
 * This function sets up a timer so it times out after the specified
 * number of microseconds.
 */
void start_timeout(unsigned int usecs) {
  timeout_end = timer_hw->timerawl + usecs;
}

/**
 * has_timed_out - returns true if timeout was reached
 *
 * This function returns true if the timer started by start_timeout
 * has reached its timeout value.
 */
unsigned int has_timed_out(void) {
  return (int32_t)(timer_hw->timerawl - timeout_end) >= 0;
}
