/*
******************************************************************************
** Linker script for STM32H723VGTx (达妙 DM-MC02 / CtrBoard-H7)
** 1MB FLASH @ 0x08000000, 128KB DTCM @ 0x20000000
**
** 默认把 .data/.bss/heap/stack 全放在 DTCM（0x20000000）：
**   - DTCM 是 Cortex-M7 上最快的 RAM，且与 CPU 同频；
**   - 本工程完全不用 DMA（USB 用内部 FIFO、FDCAN 用报文 RAM），
**     因此 DTCM 不可被 DMA 访问这个限制不影响我们。
** 如果以后要加 DMA（ADC/SPI/串口 DMA），把对应缓冲区放到 RAM_D1
** （0x24000000，320KB）并在代码里用 __attribute__((section(".RAM_D1")))。
******************************************************************************
*/

ENTRY(Reset_Handler)

_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);   /* 栈顶，DTCM 最高地址 */

_Min_Heap_Size  = 0x200;    /* 供 malloc 使用（本工程走 FreeRTOS heap_4） */
_Min_Stack_Size = 0x1000;    /* 主栈，仅调度器启动前和中断使用 */

MEMORY
{
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM_D1  (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
  RAM_D2  (xrw) : ORIGIN = 0x30000000, LENGTH = 32K
  RAM_D3  (xrw) : ORIGIN = 0x38000000, LENGTH = 16K
  ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 896K
}

SECTIONS
{
  /* 中断向量表：必须放在 FLASH 最前面，由 BOOT 配置决定映射到 0x00000000 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCMRAM

  /* FreeRTOS 的堆：heap_4 用一块静态数组，这里只是给个名字方便 map 分析 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  /* 显式命名的内存段，方便以后放 DMA 缓冲 */
  .RAM_D1 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.RAM_D1)
    *(.RAM_D1*)
    . = ALIGN(4);
  } >RAM_D1

  .RAM_D2 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.RAM_D2)
    *(.RAM_D2*)
    . = ALIGN(4);
  } >RAM_D2

  .RAM_D3 (NOLOAD) :
  {
    . = ALIGN(4);
    *(.RAM_D3)
    *(.RAM_D3*)
    . = ALIGN(4);
  } >RAM_D3

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
