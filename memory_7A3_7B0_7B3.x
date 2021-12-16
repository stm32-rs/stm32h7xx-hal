MEMORY
{
  /* This file is intended for parts in the STM32H7B3 family. (RM0455)       */
  /* - FLASH and RAM are mandatory memory sections.                          */
  /* - The sum of all non-FLASH sections must add to 1380K total device RAM. */
  /* - The FLASH section size must match your device, see table below.       */

  /* FLASH */
  /* Flash is divided in two independent banks (except 7B0xB). */
  /* Select the appropriate FLASH size for your device.        */
  /* - STM32H7B0xB                          128K (only FLASH1) */
  /* - STM32H7A3xG/7B3xG                      1M (512K + 512K) */
  /* - STM32H7A3xI/7B3xI                      2M (  1M +   1M) */
  FLASH1   : ORIGIN = 0x08000000, LENGTH = 1M
  FLASH2   : ORIGIN = 0x08100000, LENGTH = 1M
  
  /* Data TCM  */
  /* - Two contiguous 64KB RAMs.                                     */
  /* - Used for interrupt handlers, stacks and general RAM.          */
  /* - Zero wait-states.                                             */
  /* - The DTCM is taken as the origin of the base ram. (See below.) */
  /*   This is also where the interrupt table and such will live,    */
  /*   which is required for deterministic performance.              */
  DTCM     : ORIGIN = 0x20000000, LENGTH = 128K

  /* Instruction TCM */
  /* - Used for latency-critical interrupt handlers etc. */
  /* - Zero wait-states.                                 */
  ITCM     : ORIGIN = 0x00000000, LENGTH = 64K
  
  /* AXI SRAM */
  /* - AXISRAM1-3 are mapped in the CPU domain (CD).            */
  /* - Accessible by all system masters except BDMA1 and BDMA2. */
  /* - Suitable for application data not stored in DTCM.        */
  /* - Zero wait-states.                                        */
  AXISRAM1 : ORIGIN = 0x24000000, LENGTH = 256K
  AXISRAM2 : ORIGIN = 0x24040000, LENGTH = 384K
  AXISRAM3 : ORIGIN = 0x240A0000, LENGTH = 384K
  
  /* AHB SRAM */
  /* - AHBSRAM1-2 are mapped in the CPU domain (CD).  */
  /* - Accessible by all system masters except BDMA2. */
  /* - Suitable for use as DMA buffers.               */
  /* - Zero wait-states.                              */
  AHBSRAM1 : ORIGIN = 0x30000000, LENGTH = 64K
  AHBSRAM2 : ORIGIN = 0x30010000, LENGTH = 64K
  
  /* SRD SRAM */
  /* - Mapped in the Smart Run Domain (SRD).         */
  /* - Accessible by most system masters through SRD */
  /*   domain AHB matrix.                            */
  SRDSRAM  : ORIGIN = 0x38000000, LENGTH = 32K

  /* Backup SRAM */
  /* Used to store data during low-power sleeps. */
  BSRAM    : ORIGIN = 0x38800000, LENGTH = 4K  
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
    
  .axisram1 (NOLOAD) : ALIGN(8) {
    *(.axisram1 .axisram1.*);
    . = ALIGN(8);
    } > AXISRAM1
    
  .axisram2 (NOLOAD) : ALIGN(8) {
    *(.axisram2 .axisram2.*);
    . = ALIGN(8);
    } > AXISRAM2
    
  .axisram3 (NOLOAD) : ALIGN(8) {
    *(.axisram3 .axisram3.*);
    . = ALIGN(8);
    } > AXISRAM3
    
  .ahbsram1 (NOLOAD) : ALIGN(4) {
    *(.ahbsram1 .ahbsram1.*);
    . = ALIGN(4);
    } > AHBSRAM1
    
  .ahbsram2 (NOLOAD) : ALIGN(4) {
    *(.ahbsram2 .ahbsram2.*);
    . = ALIGN(4);
    } > AHBSRAM2   
    
  .srdsram (NOLOAD) : ALIGN(4) {
    *(.srdsram .srdsram.*);
    . = ALIGN(4);
    } > SRDSRAM
    
  .bsram (NOLOAD) : ALIGN(4) {
    *(.bsram .bsram.*);
    . = ALIGN(4);
    } > BSRAM  
    
} INSERT AFTER .bss;
