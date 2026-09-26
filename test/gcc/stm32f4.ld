/* Minimal GCC linker script, used only for cross-checks and Renode tests.
   FLASH/RAM sizes are passed with --defsym. */
ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 0x1000;
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = __flash_size
  RAM  (xrw) : ORIGIN = 0x20000000, LENGTH = __ram_size
}
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH
  .text : { *(.text) *(.text*) *(.rodata) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .; } >FLASH
  .ARM.exidx : { *(.ARM.exidx*) } >FLASH
  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data) *(.data*) *(.ramfunc) *(.ramfunc*) . = ALIGN(4); _edata = .; } >RAM AT> FLASH
  .bss : { . = ALIGN(4); _sbss = .; __bss_start__ = _sbss; *(.bss) *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; __bss_end__ = _ebss; } >RAM
  ._user_heap_stack : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . = . + _Min_Stack_Size; . = ALIGN(8); } >RAM
  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
