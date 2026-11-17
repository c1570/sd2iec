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


   i2c.c: I2C access through the hardware I2C unit (rppico version)

*/

/*
 * Implements the register access functions of i2c.h with the SDK
 * I2C driver. Addresses are passed in 8 bit form (R/W bit 0) like
 * for the AVR softi2c, the SDK expects 7 bit addresses.
 */

#include <string.h>
#include "config.h"
#include "i2c.h"

/* see arch-config.h */
#undef i2c_init

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#define I2C_INST I2C_INSTANCE(DISPLAY_I2C_NUM)

/* generous timeout for a transfer of len bytes, ~90us per byte at 100kHz */
static unsigned int timeout_us(unsigned int len) {
  return 2000 + 200 * len;
}

void sd2iec_i2c_init(void) {
  i2c_init(I2C_INST, I2C_CLOCK);
  gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_PIN_SDA);
  gpio_pull_up(I2C_PIN_SCL);
}

static uint8_t write_bytes(uint8_t address, const uint8_t *data, unsigned int len, bool nostop) {
  int res = i2c_write_timeout_us(I2C_INST, address >> 1, data, len, nostop, timeout_us(len));

  return res != (int)len;
}

/* Returns 1 if the transfer failed */
uint8_t i2c_write_register(uint8_t address, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };

  return write_bytes(address, buf, 2, false);
}

/* Returns 1 if the transfer failed */
uint8_t i2c_write_registers(uint8_t address, uint8_t startreg, uint8_t count, const void *data) {
  uint8_t buf[256];

  buf[0] = startreg;
  memcpy(buf + 1, data, count);
  return write_bytes(address, buf, count + 1, false);
}

/* Returns 1 if the transfer failed */
uint8_t i2c_read_registers(uint8_t address, uint8_t startreg, uint8_t count, void *data) {
  int res;

  if (write_bytes(address, &startreg, 1, true))
    return 1;

  res = i2c_read_timeout_us(I2C_INST, address >> 1, data, count, false, timeout_us(count));
  return res != count;
}

/* Returns -1 if the transfer failed, register contents otherwise */
int16_t i2c_read_register(uint8_t address, uint8_t reg) {
  uint8_t val;

  if (i2c_read_registers(address, reg, 1, &val))
    return -1;
  return val;
}
