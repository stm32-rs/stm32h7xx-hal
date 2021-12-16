MEMORY
{
  /* This file is intended for parts in the STM32H743/743v/753/753v families (RM0433), */
  /* with the exception of the STM32H742/742v parts which have a different RAM layout. */
  /* - FLASH and RAM are mandatory memory sections.                                    */
  /* - The sum of all non-FLASH sections must add to 1060K total device RAM.           */
  /* - The FLASH section size must match your device, see table below.                 */

  /* FLASH */
  /* Flash is divided in two independent banks (except 750xB). */
  /* Select the appropriate FLASH size for your device.        */
  /* - STM32H750xB                          128K (only FLASH1) */
  /* - STM32H750xB                            1M (512K + 512K) */
  /* - STM32H743xI/753xI                      2M (  1M +   1M) */
  FLASH1  : ORIGIN = 0x08000000, LENGTH = 1M
  FLASH2  : ORIGIN = 0x08100000, LENGTH = 1M
  
  /* Data TCM  */
  /* - Two contiguous 64KB RAMs.                                     */
  /* - Used for interrupt handlers, stacks and general RAM.          */
  /* - Zero wait-states.                                             */
  /* - The DTCM is taken as the origin of the base ram. (See below.) */
  /*   This is also where the interrupt table and such will live,    */
  /*   which is required for deterministic performance.              */
  DTCM    : ORIGIN = 0x20000000, LENGTH = 128K

  /* Instruction TCM */
  /* - Used for latency-critical interrupt handlers etc. */
  /* - Zero wait-states.                                 */
  ITCM    : ORIGIN = 0x00000000, LENGTH = 64K
  
  /* AXI SRAM */
  /* - AXISRAM is in D1 and accessible by all system masters except BDMA. */
  /* - Suitable for application data not stored in DTCM.                  */
  /* - Zero wait-states.                                                  */
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K
  
  /* AHB SRAM */
  /* - SRAM1-3 are in D2 and accessible by all system masters except BDMA.   */
  /*   Suitable for use as DMA buffers.                                      */
  /* - SRAM4 is in D3 and additionally accessible by the BDMA. Used for BDMA */
  /*   buffers, for storing application data in lower-power modes.           */
  /* - Zero wait-states.                                                     */
  SRAM1   : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2   : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3   : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4   : ORIGIN = 0x38000000, LENGTH = 64K

  /* Backup SRAM */
  BSRAM   : ORIGIN = 0x38800000, LENGTH = 4K
}

/*
/* Assign the memory regions defined above for use. */
/*

/* Provide the mandatory FLASH and RAM definitions for cortex-m-rt's linker script. */
REGION_ALIAS(FLASH, FLASH1);
REGION_ALIAS(RAM,   DTCM);

/* The location of the stack can be overridden using the `_stack_start` symbol. */
/* - Set the stack location at the end of RAM, using all remaining space.       */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the  */
/* `_stext` symbol. By default it will place after .vector_table. */
/* _stext = ORIGIN(FLASH) + 0x40c; */

/* Define sections for placing symbols into the extra memory regions above.   */
/* This makes them accessible from code.                                      */
/* - ITCM, DTCM and AXISRAM connect to a 64-bit wide bus -> align to 8 bytes. */
/* - All other memories     connect to a 32-bit wide bus -> align to 4 bytes. */
SECTIONS {
  .flash2 (NOLOAD) : ALIGN(4) {
    *(.flash2 .flash2.*);
    . = ALIGN(4);
    } > FLASH2
    
  .itcm (NOLOAD) : ALIGN(8) {
    *(.itcm .itcm.*);
    . = ALIGN(8);
    } > ITCM
    
  .axisram (NOLOAD) : ALIGN(8) {
    *(.axisram .axisram.*);
    . = ALIGN(8);
    } > AXISRAM
    
  .sram1 (NOLOAD) : ALIGN(8) {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
    } > SRAM1  
    
  .sram2 (NOLOAD) : ALIGN(8) {
    *(.sram2 .sram2.*);
    . = ALIGN(4);
    } > SRAM2
    
  .sram3 (NOLOAD) : ALIGN(4) {
    *(.sram3 .sram3.*);
    . = ALIGN(4);
    } > SRAM3
    
  .sram4 (NOLOAD) : ALIGN(4) {
    *(.sram4 .sram4.*);
    . = ALIGN(4);
    } > SRAM4
    
  .bsram (NOLOAD) : ALIGN(4) {
    *(.bsram .bsram.*);
    . = ALIGN(4);
    } > BSRAM  
    
} INSERT AFTER .bss;
