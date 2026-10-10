/* Production layout matching cmsis/linker/*.icf and hal/flash_store.c.
 * The entire last 128 KiB flash sector is reserved for persistent settings.
 * IAR: 4 KiB stack, no heap. .ramfunc is copied with initialized data.
 */
ENTRY(Reset_Handler)
_Min_Stack_Size = 0x1000;
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = __flash_size - 128K
  NVS   (r)  : ORIGIN = 0x08000000 + __flash_size - 128K, LENGTH = 128K
  RAM  (xrw) : ORIGIN = 0x20000000, LENGTH = __ram_size
}
_estack = ORIGIN(RAM) + LENGTH(RAM);
__StackLimit = _estack - _Min_Stack_Size;
__nvs_start = ORIGIN(NVS);
__nvs_end = ORIGIN(NVS) + LENGTH(NVS);
SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text : {
    *(.text .text.*) *(.rodata .rodata.*)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4); _etext = .;
  } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : {
    __exidx_start = .; *(.ARM.exidx* .gnu.linkonce.armexidx.*) __exidx_end = .;
  } > FLASH
  .preinit_array : {
    PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array : {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array : {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH
  _sidata = LOADADDR(.data);
  .data : {
    . = ALIGN(4); _sdata = .;
    *(.data .data.*)
    . = ALIGN(4); __ramfunc_start = .; *(.ramfunc .ramfunc.*) __ramfunc_end = .;
    . = ALIGN(4); _edata = .;
  } > RAM AT> FLASH
  .bss (NOLOAD) : {
    . = ALIGN(4); _sbss = .; __bss_start__ = .;
    *(.bss .bss.*) *(COMMON)
    . = ALIGN(4); _ebss = .; __bss_end__ = .;
  } > RAM
  .noinit (NOLOAD) : { . = ALIGN(8); *(.noinit .noinit.*) . = ALIGN(8); } > RAM
  /* No dynamic allocation: _sbrk fails rather than consuming the stack. */
  __HeapBase = .;
  __HeapLimit = .;
  PROVIDE(end = .); PROVIDE(_end = .);
  .stack __StackLimit (NOLOAD) : { . += _Min_Stack_Size; } > RAM
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
ASSERT(__HeapLimit <= __StackLimit, "RAM overflow: static data collides with 4 KiB stack")
ASSERT(ADDR(.isr_vector) == 0x08000000, "Vector table must start at flash base")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= __nvs_start, "Image overlaps persistence sector")
