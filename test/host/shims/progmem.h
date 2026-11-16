/* Host shim for avr/pgmspace.h */
#ifndef PROGMEM_H
#define PROGMEM_H

#include <stdint.h>
#include <string.h>

#define PROGMEM
#define PGM_P const char *
#define pgm_read_byte(x)   (*(const uint8_t *)(x))
#define pgm_read_word(x)   (*(const uint16_t *)(x))
#define pgm_read_dword(x)  (*(const uint32_t *)(x))
#define strcpy_P           strcpy
#define strncpy_P          strncpy
#define memcmp_P           memcmp
#define memcpy_P           memcpy

#endif
