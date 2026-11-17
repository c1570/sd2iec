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


   llfl-common.c: Common subroutines for low-level fastloader code (rppico version)

*/

/*
 * Line changes must happen at exact times relative to a captured
 * bus edge. Without timer match outputs on the IEC pins, this code
 * busy-waits on the CPU cycle counter with interrupts disabled. Changes
 * requested with NO_WAIT are kept as pending events and applied by
 * the next llfl call that waits, or by llfl_teardown. All loaders
 * follow a NO_WAIT change with a WAIT change at the same time, so
 * both lines switch within a few cycles of each other.
 *
 * Times are in 100ns units, as expected by the shared loader code.
 */

#include "config.h"
#include "pico.h"
#include "atomic.h"
#include "hardware/structs/m33.h"
#include "fastloader.h"
#include "iec-bus.h"
#include "llfl-common.h"

#define CYCLES_PER_TICK (CONFIG_MCU_FREQ / 10000000)

_Static_assert(CONFIG_MCU_FREQ % 10000000 == 0,
               "CONFIG_MCU_FREQ must be a multiple of 10MHz");

/* Lines that can be changed at a specified time */
enum { LINE_CLOCK, LINE_DATA, LINE_SRQ, LINE_COUNT };

static const uint32_t line_masks[LINE_COUNT] = {
  IEC_OBIT_CLOCK, IEC_OBIT_DATA, IEC_OBIT_SRQ
};

typedef struct {
  uint32_t     when;  // cycle counter value
  unsigned int state;
} llfl_event_t;

/* Reference time in 100ns units, may be changed by the loaders */
uint32_t llfl_reference_time;

/* The exact cycle count of the last captured reference time */
static uint32_t reference_cycles;
static uint32_t reference_ticks;

static llfl_event_t events[LINE_COUNT];
static unsigned int pending_lines;
static unsigned int saved_irq_state;

static inline __attribute__((always_inline)) uint32_t llfl_cycles(void) {
  return m33_hw->dwt_cyccnt;
}

/* true if cycle count a is at or after b */
static inline __attribute__((always_inline)) int reached(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) >= 0;
}

/* capture the current time as reference */
static inline __attribute__((always_inline)) void capture_reference(void) {
  reference_cycles    = llfl_cycles();
  reference_ticks     = reference_cycles / CYCLES_PER_TICK;
  llfl_reference_time = reference_ticks;
}

/* convert a time relative to llfl_reference_time to cycles */
static inline __attribute__((always_inline)) uint32_t target_cycles(uint32_t time) {
  uint32_t base;

  if (llfl_reference_time == reference_ticks)
    base = reference_cycles;
  else
    /* set by the caller, modulo arithmetic handles counter wrap */
    base = llfl_reference_time * CYCLES_PER_TICK;

  return base + time * CYCLES_PER_TICK;
}

/* apply all pending events that are due at the given time */
static void __not_in_flash_func(apply_pending)(uint32_t now) {
  for (unsigned int line = 0; line < LINE_COUNT; line++) {
    if ((pending_lines & (1 << line)) && reached(now, events[line].when)) {
      iec_set_lines(line_masks[line], events[line].state);
      pending_lines &= ~(1 << line);
    }
  }
}

/* wait until the cycle counter reaches target, servicing pending events */
static void __not_in_flash_func(wait_until)(uint32_t target) {
  uint32_t now;

  while (!reached(now = llfl_cycles(), target)) {
    if (pending_lines)
      apply_pending(now);
  }
}

/* ---------- utility functions ---------- */

void llfl_setup(void) {
  /* enable the cycle counter */
  m33_hw->demcr    |= M33_DEMCR_TRCENA_BITS;
  m33_hw->dwt_ctrl |= M33_DWT_CTRL_CYCCNTENA_BITS;

  /* interrupts would delay line changes */
  saved_irq_state = __get_primask();
  __iCliRetVal();
  pending_lines   = 0;
}

void llfl_teardown(void) {
  /* apply line changes that are still outstanding */
  while (pending_lines)
    apply_pending(llfl_cycles());

  __iRestore(&saved_irq_state);
}

/**
 * wait_line - wait until an IEC line has the specified level
 * @bit      : IEC_BIT_* of the line
 * @state    : line level to wait for (0 low, 1 high)
 * @atnabort : abort when ATN is low
 *
 * This function waits until the line has the specified level and
 * captures the time when it reached it as llfl_reference_time.
 */
static void __not_in_flash_func(wait_line)(uint32_t bit, unsigned int state,
                                           llfl_atnabort_t atnabort) {
  while (!IEC_IN_COND_INV(IEC_INPUT & bit) != !state) {
    if (atnabort && !IEC_ATN)
      break;
    if (pending_lines)
      apply_pending(llfl_cycles());
  }
  capture_reference();
}

/**
 * llfl_wait_atn - wait until ATN has the specified level and capture time
 * @state: line level to wait for (0 low, 1 high)
 *
 * This function waits until the ATN line has the specified level and captures
 * the time when its level changed.
 */
void llfl_wait_atn(unsigned int state) {
  wait_line(IEC_BIT_ATN, state, NO_ATNABORT);
}

/* llfl_wait_clock - see llfl_wait_atn, aborts on ATN low if atnabort is true */
void llfl_wait_clock(unsigned int state, llfl_atnabort_t atnabort) {
  wait_line(IEC_BIT_CLOCK, state, atnabort);
}

/* llfl_wait_data - see llfl_wait_clock */
void llfl_wait_data(unsigned int state, llfl_atnabort_t atnabort) {
  wait_line(IEC_BIT_DATA, state, atnabort);
}

/**
 * set_line_at - sets an IEC line at a specified time offset
 * @line : LINE_* index of the line
 * @time : change time in 100ns after llfl_reference_time
 * @state: new line state (0 low, 1 high)
 * @wait : wait until change happened if WAIT, else schedule it
 */
static void __not_in_flash_func(set_line_at)(unsigned int line, uint32_t time,
                                             unsigned int state, llfl_wait_t wait) {
  uint32_t target = target_cycles(time);

  /* check if requested time is possible */
  if (!reached(target, llfl_cycles()))
    set_test_led(1);

  if (wait == NO_WAIT) {
    events[line].when  = target;
    events[line].state = state;
    pending_lines |= 1 << line;
    return;
  }

  /* collect this line and pending changes for the same time, */
  /* so they are applied right after the wait                 */
  uint32_t release_mask = 0, pull_mask = 0;

  pending_lines &= ~(1 << line);
  if (state)
    release_mask = line_masks[line];
  else
    pull_mask = line_masks[line];

  for (unsigned int i = 0; i < LINE_COUNT; i++) {
    if ((pending_lines & (1 << i)) && events[i].when == target) {
      if (events[i].state)
        release_mask |= line_masks[i];
      else
        pull_mask |= line_masks[i];
      pending_lines &= ~(1 << i);
    }
  }

  wait_until(target);
  iec_set_lines(release_mask, 1);
  iec_set_lines(pull_mask, 0);
}

/**
 * llfl_set_clock_at - sets clock line at a specified time offset
 * @time : change time in 100ns after llfl_reference_time
 * @state: new line state (0 low, 1 high)
 * @wait : wait until change happened if 1
 *
 * This function sets the clock line to a specified state at a defined time
 * after the llfl_reference_time set by a previous wait_* function.
 */
void llfl_set_clock_at(uint32_t time, unsigned int state, llfl_wait_t wait) {
  set_line_at(LINE_CLOCK, time, state, wait);
}

/* llfl_set_data_at - see llfl_set_clock_at */
void llfl_set_data_at(uint32_t time, unsigned int state, llfl_wait_t wait) {
  set_line_at(LINE_DATA, time, state, wait);
}

/* llfl_set_srq_at - see llfl_set_clock_at (nice for debugging) */
void llfl_set_srq_at(uint32_t time, unsigned int state, llfl_wait_t wait) {
  set_line_at(LINE_SRQ, time, state, wait);
}

/**
 * llfl_read_bus_at - reads the IEC bus at a certain time
 * @time: read time in 100ns after llfl_reference_time
 *
 * This function returns the current IEC bus state at a certain time
 * after the llfl_reference_time set by a previous wait_* function.
 */
uint32_t __not_in_flash_func(llfl_read_bus_at)(uint32_t time) {
  uint32_t target = target_cycles(time);

  /* check if requested time is possible */
  if (!reached(target, llfl_cycles()))
    set_test_led(1);

  wait_until(target);

  return iec_bus_read();
}

/**
 * llfl_now - returns current time
 *
 * This function returns the current time in 100ns units
 */
uint32_t llfl_now(void) {
  return llfl_cycles() / CYCLES_PER_TICK;
}

/**
 * llfl_generic_load_2bit - generic 2-bit fastloader transmit
 * @def : pointer to fastloader definition struct
 * @byte: data byte
 *
 * This function implements generic 2-bit fastloader
 * transmission based on a generic_2bit_t struct.
 */
void llfl_generic_load_2bit(const generic_2bit_t *def, uint8_t byte) {
  unsigned int i;

  byte ^= def->eorvalue;

  for (i=0;i<4;i++) {
    llfl_set_clock_at(def->pairtimes[i], byte & (1 << def->clockbits[i]), NO_WAIT);
    llfl_set_data_at (def->pairtimes[i], byte & (1 << def->databits[i]),  WAIT);
  }
}

/**
 * llfl_generic_save_2bit - generic 2-bit fastsaver receive
 * @def: pointer to fastloader definition struct
 *
 * This function implements genereic 2-bit fastsaver reception
 * based on a generic_2bit_t struct.
 */
uint8_t llfl_generic_save_2bit(const generic_2bit_t *def) {
  unsigned int i;
  uint8_t result = 0;

  for (i=0;i<4;i++) {
    uint32_t bus = llfl_read_bus_at(def->pairtimes[i]);

    result |= (!!(bus & IEC_BIT_CLOCK)) << def->clockbits[i];
    result |= (!!(bus & IEC_BIT_DATA))  << def->databits[i];
  }

  return result ^ def->eorvalue;
}
