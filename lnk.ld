ENTRY(main)

MEMORY{
   ISR (rx): 	ORIGIN = 0x00000000, LENGTH = 0x00000100
   FLASH (rwx): ORIGIN = 0x00000100, LENGTH = 0x00002000
   RAM (rwx):   ORIGIN = 0x20000000, LENGTH = 0x00001000
}

SECTIONS {
    .isrVectors : {
        . = ALIGN(4);
        KEEP(*(.isrVectors))
        . = ALIGN(4);
    } > ISR

    .text : {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
    } > FLASH

    _data_flash = .;

    .data : AT ( _data_flash ) {
        . = ALIGN(4);
        _data_begin = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _data_end = .;
    } > RAM

    .bss : {
        . = ALIGN(4);
        _bss_begin = .;
        _bss_start__ = _bss_begin;
        *(.bss)
        *(.bss*)
	*(COMMON)
        . = ALIGN(4);
        _bss_end = .;
        __bss_end__ = _bss_end;
    } > RAM

    _stack_size = 0x100;
    _stack_end = ORIGIN(RAM)+LENGTH(RAM);
    _stack_begin = _stack_end - _stack_size;
    . = _stack_begin;

    ._stack : {
        . = . + _stack_size;
    } > RAM
}

