MEMORY
{
  /* This file is intended for parts in the STM32H747 family. (RM0399)       */
  /* - These are dual-core parts, having both a CM7 (primary) and CM4 core.  */
  /* - FLASH and RAM are mandatory memory sections.                          */
  /* - The sum of all non-FLASH sections must add to 1060k total device RAM. */
  /* - The FLASH section size must match your device, see table below.       */

  /* FLASH */
  /* Flash is divided in two independent banks.          */
  /* Select the appropriate FLASH sizes for your device. */
  /* - STM32H745xG/747xG                1M (512K + 512K) */
  /* - STM32H745xI/747xI/755xI/757xI    2M (  1M +   1M) */
  FLASH1  : ORIGIN = 0x08000000, LENGTH = 1M
  FLASH2  : ORIGIN = 0x08100000, LENGTH = 1M 
  
  /* Data TCM  */
  /* - Two contiguous 64KB RAMs.                                     */
  /* - Used for interrupt handlers, stacks and general RAM.          */
  /* - Only accessible to the Cortex-M7 core.                        */
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
  /* - Accessible by all system masters except BDMA.     */
  /* - Suitable for application data not stored in DTCM. */
  /* - Zero wait-states.                                 */
  AXISRAM : ORIGIN = 0x24000000, LENGTH = 512K
  
  /* AHB SRAM */
  /* - SRAM1-3 are in D2 and accessible by all system masters except BDMA.   */
  /*   Suitable for use as DMA buffers.                                      */
  /* - SRAM4 is in D3 and additionally accessible by the BDMA. Used for BDMA */
  /*   buffers, for storing application data in lower-power modes, or for    */
  /*   sharing data between the Cortex-M4 and Cortex-M7 cores.               */
  /* - Zero wait-states.                                                     */
  SRAM1   : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2   : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3   : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4   : ORIGIN = 0x38000000, LENGTH = 64K

  /* Backup SRAM */
  /* Used to store data during low-power sleeps. */
  BSRAM   : ORIGIN = 0x38800000, LENGTH = 4K  
}

/*
/* Assign the memory regions defined above for use. */
/*

/* Provide the mandatory FLASH and RAM definitions for cortex-m-rt's linker script. */
/* - These definitions concern only the CM7 core (CPU1).                            */
/* - The CM4 core (CPU2) is set-up separately in the various other sections, below. */
REGION_ALIAS(FLASH, FLASH1);
REGION_ALIAS(RAM,   DTCM);

/* The location of the stack can be overridden using the `_stack_start` symbol.           */
/* - Set the CM7 core (CPU1) stack location at the end of RAM, using all remaining space. */
/* - Set the CM4 core (CPU2) stack location in SRAM2, using the whole memory bank.        */
_stack_start      = ORIGIN(RAM)   + LENGTH(RAM);
_cpu2_stack_start = ORIGIN(SRAM2) + LENGTH(SRAM2);

/* The location of the .text section can be overridden using the  */
/* `_stext` symbol. By default it will place after .vector_table. */
/* _stext = ORIGIN(FLASH) + 0x40c; */

/* Define sections for placing symbols into the extra memory regions above.   */
/* This makes them accessible from code.                                      */
/* - ITCM, DTCM and AXISRAM connect to a 64-bit wide bus -> align to 8 bytes. */
/* - All other memories     connect to a 32-bit wide bus -> align to 4 bytes. */
/* - The SRAM1 and SRAM2 sections are commonly used as the stack and heap for */
/*   the CM4 core in dual core versions and may thus have to be omitted from  */
/*   this list.                                                               */
SECTIONS {
  .flash2 : ALIGN(4) {
    LONG(_cpu2_stack_start);
    KEEP(*(.flash2.reset_vector));
    KEEP(*(.flash2.vector_table));
    *(.flash2 .flash2.*);
    . = ALIGN(4);
    } > FLASH2

  /*.itcm (NOLOAD) : ALIGN(8) { */
  /*  *(.itcm .itcm.*);         */
  /*  . = ALIGN(8);             */
  /*  } > ITCM                  */
    
  .itcm : ALIGN(4) {
    *(.itcm .itcm.*);
    . = ALIGN(4);
    } > ITCM AT > FLASH
    
  .axisram (NOLOAD) : ALIGN(8) {
    *(.axisram .axisram.*);
    . = ALIGN(8);
    } > AXISRAM
    
  .sram1 (NOLOAD) : ALIGN(4) {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
    } > SRAM1
    
  .sram2 (NOLOAD) : ALIGN(4) {
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
    
};
