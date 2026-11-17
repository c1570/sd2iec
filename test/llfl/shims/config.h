/* Host shim for config.h: IEC bus access goes to the simulation in llfltest.c */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define CONFIG_MCU_FREQ 150000000

#define BV(x) (1u<<(x))

typedef uint32_t iec_bus_t;

#define IEC_BIT_ATN    BV(2)
#define IEC_BIT_CLOCK  BV(3)
#define IEC_BIT_DATA   BV(4)
#define IEC_BIT_SRQ    BV(5)
#define IEC_OBIT_CLOCK BV(7)
#define IEC_OBIT_DATA  BV(8)
#define IEC_OBIT_SRQ   BV(9)

uint32_t sim_gpio_in(void);
void iec_set_lines(uint32_t mask, unsigned int state);
void set_test_led(uint8_t state);

#define IEC_INPUT          sim_gpio_in()
#define IEC_IN_COND_INV(x) (x)
#define IEC_ATN            (IEC_INPUT & IEC_BIT_ATN)
#define IEC_CLOCK          (IEC_INPUT & IEC_BIT_CLOCK)
#define IEC_DATA           (IEC_INPUT & IEC_BIT_DATA)

static inline iec_bus_t iec_bus_read(void) {
  return IEC_INPUT & (IEC_BIT_ATN | IEC_BIT_CLOCK | IEC_BIT_DATA | IEC_BIT_SRQ);
}

#endif
