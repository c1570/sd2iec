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


   system.c: System-specific initialisation (rppico version)

*/

#include "config.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "arch-eeprom.h"
#include "system.h"

/* Early system initialisation */
void system_init_early(void) {
  __asm__ volatile ("cpsid i" ::: "memory");

  /* The SDK runtime has set up the clocks, make sure the */
  /* system clock matches the configured frequency.       */
  /* Peripheral clocks follow, so this must come first.   */
  set_sys_clock_hz(CONFIG_MCU_FREQ, true);
}

/* Late initialisation */
void system_init_late(void) {
  /* Enable GPIO interrupt */
  irq_set_enabled(IO_IRQ_BANK0, true);
}

/* Called while the bus is idle: store EEPROM changes or sleep */
void system_sleep(void) {
  eeprom_idle();
  __asm__ volatile ("wfi");
}

/* Reset MCU */
void system_reset(void) {
  __asm__ volatile ("cpsid i" ::: "memory");
  watchdog_reboot(0, 0, 0);
  while (1) ;
}

/* Disable interrupts */
void disable_interrupts(void) {
  __asm__ volatile ("cpsid i" ::: "memory");
}

/* Enable interrupts */
void enable_interrupts(void) {
  __asm__ volatile ("cpsie i" ::: "memory");
}

/*** GPIO interrupt demux ***/

/* Declare handler functions */
SD_CHANGE_HANDLER;
IEC_ATN_HANDLER;
IEC_CLOCK_HANDLER;

/* acknowledge and return the enabled pending events of a pin */
static uint32_t pin_events(unsigned int pin) {
  uint32_t events = gpio_get_irq_event_mask(pin);

  if (events)
    gpio_acknowledge_irq(pin, events);
  return events;
}

void isr_io_bank0(void) {
  if (pin_events(IEC_PIN_ATN))
    iec_atn_handler();

#ifdef CONFIG_LOADER_DREAMLOAD
  if (pin_events(IEC_PIN_CLOCK))
    iec_clock_handler();
#endif

  if (pin_events(SD_PIN_DETECT))
    sdcard_change_handler();
}

void isr_hardfault(void) {
  set_test_led(1);
  while (1) ;
}
