MEMORY
{
  /* FLASH and RAM are mandatory memory regions */
  FLASH0  : ORIGIN = 0x08000000, LENGTH = 1024K
  FLASH : ORIGIN = 0x08100000, LENGTH = 1024K
  RAM    : ORIGIN = 0x30020000, LENGTH = 128K /* SRAM2 */

  /* AXISRAM */
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K

  /* SRAM */
  SRAM1 : ORIGIN = 0x30000000, LENGTH = 128K
  /* SRAM2: See RAM */
  SRAM3 : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4 : ORIGIN = 0x38000000, LENGTH = 64K

  /* Backup SRAM */
  BSRAM : ORIGIN = 0x38800000, LENGTH = 4K
}

/* The location of the stack can be overridden using the
   `_stack_start` symbol.  Place the stack at the end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */
