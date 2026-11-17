/*
 * Augments the Pico SDK linker script: objects marked EEMEM get
 * addresses starting at 0 without occupying memory in the image,
 * matching the offset-based addressing of the eeprom_* functions.
 */
SECTIONS
{
  .eeprom 0 (INFO) : { KEEP(*(.eeprom)) }
}
INSERT AFTER .bss;
