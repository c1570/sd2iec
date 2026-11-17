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


   arch-config.h: The main architecture-specific config header (rppico version)

*/

#ifndef ARCH_CONFIG_H
#define ARCH_CONFIG_H

#include <stdint.h>
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

/* AVR compatibility macro */
#define BV(x) (1u<<(x))

/* The Pico SDK has global functions with the same names, */
/* rp-glue.c and i2c.c undefine these for the SDK versions */
#define spi_init sd2iec_spi_init
#ifdef CONFIG_UART_DEBUG
#  define uart_init sd2iec_uart_init
#endif
#ifdef HAVE_I2C
#  define i2c_init sd2iec_i2c_init
#endif

/* ----- Common definitions for all rppico hardware variants ------ */

/* Return value of buttons_read() */
typedef unsigned int rawbutton_t;

/* Interrupt handler for system tick */
#define SYSTEM_TICK_HANDLER void isr_systick(void)

/* The GPIO interrupt is demuxed in system.c, function names are fixed */
#define SD_CHANGE_HANDLER  void sdcard_change_handler(void)
#define IEC_ATN_HANDLER    void iec_atn_handler(void)
#define IEC_CLOCK_HANDLER  void iec_clock_handler(void)

/* SPI clock rates: 400kHz for card initialisation, 25MHz afterwards */
#define SPI_FREQ_SLOW 400000
#define SPI_FREQ_FAST 25000000

static inline void iec_interrupts_init(void) {
  // Nothing, the GPIO interrupt is enabled in system_init_late
}

/* All of RAM is equal, no special placement for the P00 name cache */
#define P00CACHE_ATTRIB

/* Size of the emulated EEPROM, kept in flash */
#define EEPROM_SIZE 8192

/* EEPROMFS: offset and size must be multiples of 4 */
/* to actually enable it, CONFIG_HAVE_EEPROMFS must be set in config */
#  define EEPROMFS_OFFSET     512
#  define EEPROMFS_SIZE       7680
#  define EEPROMFS_ENTRIES    16
#  define EEPROMFS_SECTORSIZE 64


/* ---------- Pins common to all variants ---------- */
/* Pin numbers are GPIO numbers. GPIO29 is used by variant 2, so it */
/* needs a board that brings it out (not a standard Pico 2).        */

#define DEBUG_UART_NUM 0
#define UART_TX_PIN  0
#define UART_RX_PIN  1

#define BUTTON_PIN_NEXT 10
#define BUTTON_PIN_PREV 11

#define LED_PIN_DIRTY 12
#define LED_PIN_BUSY  25

/* optional remote display, hardware I2C1 */
#define DISPLAY_I2C_NUM 1
#define I2C_PIN_SDA 14
#define I2C_PIN_SCL 15
#define I2C_CLOCK   100000

/* device address jumpers to GND */
#define ADDR_PIN_A 20
#define ADDR_PIN_B 21

#define HAVE_SD
#define SD_SUPPLY_VOLTAGE (1L<<21)
#define SD_SPI_NUM    0
#define SD_PIN_MISO   16
#define SD_PIN_CS     17
#define SD_PIN_SCK    18
#define SD_PIN_MOSI   19
/* card detect switch to GND, closed when a card is inserted */
#define SD_PIN_DETECT 22


#if CONFIG_HARDWARE_VARIANT == 1
/* ---------- Hardware configuration: direct connection ---------- */
/* The IEC lines are connected to the pins directly, which emulate */
/* open-collector outputs. No SRQ.                                 */

#  define IEC_PIN_ATN   2
#  define IEC_PIN_CLOCK 3
#  define IEC_PIN_DATA  4
#  define IEC_PIN_RESET 5

#elif CONFIG_HARDWARE_VARIANT == 2
/* ---------- Hardware configuration: buffered ---------- */
/* Inputs through a 74LVC04, outputs through a 7406, so all lines */
/* are inverted. ATN is input only.                               */

#  define IEC_INPUTS_INVERTED
#  define IEC_PIN_ATN   2
#  define IEC_PIN_CLOCK 3
#  define IEC_PIN_DATA  4
#  define IEC_PIN_RESET 5
#  define IEC_PIN_SRQ   29

#  define IEC_SEPARATE_OUT
#  define IEC_OUTPUTS_INVERTED
#  define IEC_OPIN_CLOCK 6
#  define IEC_OPIN_DATA  7
#  define IEC_OPIN_SRQ   28

#else
#  error "CONFIG_HARDWARE_VARIANT is unset or set to an unknown value."
#endif


/* ---------------------------------------------------------------------------- */
/*  Generic helpers based on the pin definitions above                          */
/* ---------------------------------------------------------------------------- */

static inline void sio_set_level(uint32_t mask, unsigned int state) {
  if (state)
    sio_hw->gpio_set = mask;
  else
    sio_hw->gpio_clr = mask;
}

/* --- device address --- */
static inline void device_hw_address_init(void) {
  gpio_init(ADDR_PIN_A);
  gpio_init(ADDR_PIN_B);
  gpio_pull_up(ADDR_PIN_A);
  gpio_pull_up(ADDR_PIN_B);
}

static inline uint8_t device_hw_address(void) {
  return 8 + !gpio_get(ADDR_PIN_A) + 2*!gpio_get(ADDR_PIN_B);
}

/* --- LEDs --- */
static inline void leds_init(void) {
  gpio_init(LED_PIN_BUSY);
  gpio_init(LED_PIN_DIRTY);
  gpio_set_dir(LED_PIN_BUSY,  GPIO_OUT);
  gpio_set_dir(LED_PIN_DIRTY, GPIO_OUT);
}

static inline __attribute__((always_inline)) void set_busy_led(uint8_t state) {
  sio_set_level(BV(LED_PIN_BUSY), state);
}

static inline __attribute__((always_inline)) void set_dirty_led(uint8_t state) {
  sio_set_level(BV(LED_PIN_DIRTY), state);
}

/* no separate test LED, used to signal missed fastloader timing */
static inline __attribute__((always_inline)) void set_test_led(uint8_t state) {
  sio_set_level(BV(LED_PIN_DIRTY), state);
}

static inline void toggle_dirty_led(void) {
  sio_hw->gpio_togl = BV(LED_PIN_DIRTY);
}

/* --- buttons --- */
#define BUTTON_NEXT BV(BUTTON_PIN_NEXT)
#define BUTTON_PREV BV(BUTTON_PIN_PREV)

static inline rawbutton_t buttons_read(void) {
  return sio_hw->gpio_in & (BUTTON_NEXT | BUTTON_PREV);
}

static inline void buttons_init(void) {
  gpio_init(BUTTON_PIN_NEXT);
  gpio_init(BUTTON_PIN_PREV);
  gpio_pull_up(BUTTON_PIN_NEXT);
  gpio_pull_up(BUTTON_PIN_PREV);
}

/* --- SD card --- */
static inline void sdcard_interface_init(void) {
  /* chip select: output, high */
  gpio_init(SD_PIN_CS);
  gpio_put(SD_PIN_CS, 1);
  gpio_set_dir(SD_PIN_CS, GPIO_OUT);

  /* SPI pins, pull-up on MISO for cards that are slow to drive it */
  gpio_set_function(SD_PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(SD_PIN_SCK,  GPIO_FUNC_SPI);
  gpio_set_function(SD_PIN_MOSI, GPIO_FUNC_SPI);
  gpio_pull_up(SD_PIN_MISO);

  /* card detect, interrupt on both edges */
  gpio_init(SD_PIN_DETECT);
  gpio_pull_up(SD_PIN_DETECT);
  gpio_set_irq_enabled(SD_PIN_DETECT, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);

  /* Note: The GPIO interrupt is enabled in system_init_late */
}

static inline void sdcard_set_ss(int state) {
  sio_set_level(BV(SD_PIN_CS), state);
}

static inline uint8_t sdcard_detect(void) {
  return !gpio_get(SD_PIN_DETECT);
}

/* microSD cards have no write protect switch */
static inline uint8_t sdcard_wp(void) {
  return 0;
}

/* --- UART --- */
static inline void uart_pins_connect(void) {
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

/* --- IEC bus --- */
typedef uint32_t iec_bus_t;

#define IEC_INPUT        (sio_hw->gpio_in)
#define IEC_BIT_ATN      BV(IEC_PIN_ATN)
#define IEC_BIT_CLOCK    BV(IEC_PIN_CLOCK)
#define IEC_BIT_DATA     BV(IEC_PIN_DATA)
#ifdef IEC_PIN_SRQ
#  define IEC_BIT_SRQ    BV(IEC_PIN_SRQ)
#else
#  define IEC_BIT_SRQ    0
#endif
/* RESET is only an input for now, sd2iec does not react to it */
#ifdef IEC_PIN_RESET
#  define IEC_BIT_RESET  BV(IEC_PIN_RESET)
#else
#  define IEC_BIT_RESET  0
#endif

#ifdef IEC_INPUTS_INVERTED
#  define IEC_IN_COND_INV(x) (!(x))
#else
#  define IEC_IN_COND_INV(x) (x)
#endif

#define IEC_ATN   IEC_IN_COND_INV(IEC_INPUT & IEC_BIT_ATN)
#define IEC_CLOCK IEC_IN_COND_INV(IEC_INPUT & IEC_BIT_CLOCK)
#define IEC_DATA  IEC_IN_COND_INV(IEC_INPUT & IEC_BIT_DATA)
#define IEC_SRQ   IEC_IN_COND_INV(IEC_INPUT & IEC_BIT_SRQ)

/* output bits, 0 for lines that cannot be driven */
#ifdef IEC_SEPARATE_OUT
#  ifdef IEC_OPIN_ATN
#    define IEC_OBIT_ATN BV(IEC_OPIN_ATN)
#  else
#    define IEC_OBIT_ATN 0
#  endif
#  define IEC_OBIT_CLOCK BV(IEC_OPIN_CLOCK)
#  define IEC_OBIT_DATA  BV(IEC_OPIN_DATA)
#  ifdef IEC_OPIN_SRQ
#    define IEC_OBIT_SRQ BV(IEC_OPIN_SRQ)
#  else
#    define IEC_OBIT_SRQ 0
#  endif
#else
#  define IEC_OBIT_ATN   IEC_BIT_ATN
#  define IEC_OBIT_CLOCK IEC_BIT_CLOCK
#  define IEC_OBIT_DATA  IEC_BIT_DATA
#  define IEC_OBIT_SRQ   IEC_BIT_SRQ
#endif

#define IEC_INPUT_MASK  (IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | \
                         IEC_BIT_SRQ | IEC_BIT_RESET)
#define IEC_OUTPUT_MASK (IEC_OBIT_ATN | IEC_OBIT_CLOCK | IEC_OBIT_DATA | \
                         IEC_OBIT_SRQ)

/**
 * iec_set_lines - set IEC output lines
 * @mask : IEC_OBIT_* bits of the lines to change
 * @state: new line state (0 low, 1 released)
 *
 * With separate outputs the pins drive an external open-collector
 * buffer, otherwise the bus pins themselves emulate open drain by
 * switching their output enable with the output latch kept at 0.
 */
static inline __attribute__((always_inline)) void iec_set_lines(uint32_t mask, unsigned int state) {
#if !defined(IEC_SEPARATE_OUT)
  if (state)
    sio_hw->gpio_oe_clr = mask;
  else
    sio_hw->gpio_oe_set = mask;
#elif defined(IEC_OUTPUTS_INVERTED)
  sio_set_level(mask, !state);
#else
  sio_set_level(mask, state);
#endif
}

static inline __attribute__((always_inline)) void set_atn(unsigned int state) {
  iec_set_lines(IEC_OBIT_ATN, state);
}

static inline __attribute__((always_inline)) void set_clock(unsigned int state) {
  iec_set_lines(IEC_OBIT_CLOCK, state);
}

static inline __attribute__((always_inline)) void set_data(unsigned int state) {
  iec_set_lines(IEC_OBIT_DATA, state);
}

static inline __attribute__((always_inline)) void set_srq(unsigned int state) {
  iec_set_lines(IEC_OBIT_SRQ, state);
}

/* Enable/disable ATN interrupt, triggered on both edges */
static inline void set_atn_irq(uint8_t state) {
  gpio_set_irq_enabled(IEC_PIN_ATN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, state);
}

/* Enable/disable CLOCK interrupt, triggered on both edges */
static inline void set_clock_irq(uint8_t state) {
  gpio_set_irq_enabled(IEC_PIN_CLOCK, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, state);
}
#define HAVE_CLOCK_IRQ

/* Connect the IEC pins, the output state must be set before */
static inline void iec_pins_connect(void) {
  for (unsigned int pin = 0; pin < 32; pin++) {
    if (IEC_INPUT_MASK & BV(pin)) {
      /* gpio_set_function keeps the output latch, unlike gpio_init */
      gpio_set_function(pin, GPIO_FUNC_SIO);
      gpio_disable_pulls(pin);
    }
  }

#ifndef IEC_SEPARATE_OUT
  /* open drain emulation: pulling low only needs the output enable */
  sio_hw->gpio_clr = IEC_OUTPUT_MASK;
#else
  sio_hw->gpio_oe_set = IEC_OUTPUT_MASK;
  for (unsigned int pin = 0; pin < 32; pin++)
    if (IEC_OUTPUT_MASK & BV(pin))
      gpio_set_function(pin, GPIO_FUNC_SIO);
#endif
}

/* no parallel port */
static inline void parallel_init(void) {}

/* Display interrupt request line, not connected */
static inline void display_intrq_init(void) {}

static inline unsigned int display_intrq_active(void) {
  return 0;
}

#endif
