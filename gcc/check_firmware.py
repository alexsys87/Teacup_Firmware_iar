#!/usr/bin/env python3
"""Check ELF/HEX/BIN flash layout without third-party Python packages."""
import argparse
from pathlib import Path
import struct

MEMORY = {
    "F401C": (256 * 1024, 64 * 1024),
    "F401E": (512 * 1024, 96 * 1024),
    "F411E": (512 * 1024, 128 * 1024),
}
FLASH_BASE = 0x08000000
RAM_BASE = 0x20000000


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_hex(path):
    memory, upper, eof = {}, 0, False
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        require(not eof and line.startswith(":"), "Invalid HEX record/record after EOF")
        record = bytes.fromhex(line[1:])
        require(len(record) >= 5 and len(record) == record[0] + 5,
                "Invalid HEX record length")
        require(sum(record) % 256 == 0, "HEX checksum mismatch")
        kind, address, data = record[3], int.from_bytes(record[1:3], "big"), record[4:-1]
        if kind == 0:
            for offset, byte in enumerate(data):
                absolute = upper + address + offset
                require(absolute not in memory, "Overlapping HEX records")
                memory[absolute] = byte
        elif kind == 1:
            require(not data and address == 0, "Invalid HEX EOF")
            eof = True
        elif kind in (2, 4):
            require(len(data) == 2 and address == 0, "Invalid HEX address record")
            upper = int.from_bytes(data, "big") << (4 if kind == 2 else 16)
        elif kind in (3, 5):
            require(len(data) == 4, "Invalid HEX entry record")
        else:
            raise ValueError(f"Unsupported HEX record type {kind}")
    require(eof and memory, "Empty/incomplete HEX")
    return memory


def check(args):
    flash_size, ram_size = MEMORY[args.chip]
    nvs_start = FLASH_BASE + flash_size - 128 * 1024
    nvs_end = FLASH_BASE + flash_size
    image = args.elf.read_bytes()
    require(image[:7] == b"\x7fELF\x01\x01\x01", "Expected little-endian ELF32")
    header = struct.unpack_from("<HHIIIIIHHHHHH", image, 16)
    require(header[1] == 40, "Expected ARM ELF")
    entry, phoff, phsize, phcount = header[3], header[4], header[8], header[9]
    require(phsize == 32, "Unexpected program header size")
    segments = []
    for index in range(phcount):
        kind, offset, vaddr, paddr, filesz, memsz, flags, alignment = struct.unpack_from(
            "<IIIIIIII", image, phoff + index * phsize)
        if kind != 1 or not memsz:
            continue
        require(filesz <= memsz and offset + filesz <= len(image), "Invalid ELF segment")
        require((FLASH_BASE <= vaddr and vaddr + memsz <= FLASH_BASE + flash_size)
                or (RAM_BASE <= vaddr and vaddr + memsz <= RAM_BASE + ram_size),
                f"ELF segment outside device memory: {vaddr:#x}")
        if not filesz:
            continue
        require(FLASH_BASE <= paddr and paddr + filesz <= FLASH_BASE + flash_size,
                "ELF load image outside flash")
        require(paddr + filesz <= nvs_start or paddr >= nvs_end,
                "ELF load segment overlaps settings sector")
        segments.append((offset, paddr, filesz))
    # objcopy emits allocated sections, not alignment padding between them
    # in a PT_LOAD segment. Derive LMAs from the section's containing segment.
    memory = {}
    shoff, shsize, shcount = header[5], header[10], header[11]
    require(shsize == 40, "Unexpected section header size")
    sections = []
    for index in range(shcount):
        name, kind, flags, vaddr, offset, size, link, info, align, entsize = struct.unpack_from(
            "<IIIIIIIIII", image, shoff + index * shsize)
        sections.append((name, kind, flags, vaddr, offset, size, link, info, align, entsize))
        if not flags & 2 or kind == 8 or not size:  # SHF_ALLOC, SHT_NOBITS
            continue
        owners = [(start, paddr) for start, paddr, length in segments
                  if start <= offset and offset + size <= start + length]
        require(len(owners) == 1, "Allocated section has no unique load segment")
        start, paddr = owners[0]
        for i, byte in enumerate(image[offset:offset + size]):
            absolute = paddr + offset - start + i
            require(absolute not in memory, "Overlapping ELF load sections")
            memory[absolute] = byte
    require(memory and min(memory) == FLASH_BASE, "Missing vector table at flash base")
    hex_memory = read_hex(args.hex)
    require(memory == hex_memory, "HEX differs from ELF load image")
    vector = bytes(memory[FLASH_BASE + i] for i in range(8))
    stack, reset = struct.unpack("<II", vector)
    require(stack == RAM_BASE + ram_size, "Initial stack pointer does not match chip")
    require(reset & 1 and reset == entry, "Reset vector/ELF entry mismatch or missing Thumb bit")
    require((reset & ~1) in memory, "Reset vector points outside load image")
    symbols = {}
    for section in sections:
        name, kind, flags, vaddr, offset, size, link, info, align, entsize = section
        if kind != 2:  # SHT_SYMTAB
            continue
        require(entsize == 16 and link < shcount, "Invalid ELF symbol table")
        strings = sections[link]
        names = image[strings[4]:strings[4] + strings[5]]
        for position in range(offset, offset + size, entsize):
            name_at, value, length, symbol_info, other, shindex = struct.unpack_from(
                "<IIIBBH", image, position)
            if name_at and shindex:
                symbol_name = names[name_at:names.index(0, name_at)].decode()
                symbols[symbol_name] = (value, symbol_info >> 4)
    value = lambda name: symbols[name][0]
    require(value("_sdata") <= value("__ramfunc_start") < value("__ramfunc_end")
            <= value("_edata") <= value("_sbss") <= value("_ebss")
            <= value("__StackLimit"), "Invalid initialized data/BSS/RAM function layout")
    require(value("__StackLimit") == stack - 4096, "Expected IAR-compatible 4 KiB stack")
    require(value("__HeapBase") == value("__HeapLimit"), "Expected IAR-compatible zero heap")
    for name in ("flash_wait", "flash_program"):
        require(value("__ramfunc_start") <= (value(name) & ~1) < value("__ramfunc_end"),
                f"{name} must execute from initialized RAM")
    # Strong handlers must replace startup weak aliases at the architectural
    # vector indices. These peripherals are used by the production board.
    irqs = {"SysTick_Handler": 15, "TIM5_IRQHandler": 16 + 50,
            "OTG_FS_IRQHandler": 16 + 67, "USART2_IRQHandler": 16 + 38,
            "DMA1_Stream5_IRQHandler": 16 + 16, "DMA1_Stream6_IRQHandler": 16 + 17,
            "I2C1_EV_IRQHandler": 16 + 31, "I2C1_ER_IRQHandler": 16 + 32,
            "EXTI0_IRQHandler": 16 + 6, "EXTI1_IRQHandler": 16 + 7,
            "EXTI2_IRQHandler": 16 + 8, "EXTI3_IRQHandler": 16 + 9,
            "EXTI4_IRQHandler": 16 + 10, "EXTI9_5_IRQHandler": 16 + 23,
            "EXTI15_10_IRQHandler": 16 + 40}
    for name, index in irqs.items():
        if name not in symbols or symbols[name][1] != 1:  # STB_GLOBAL, not weak
            continue
        word = bytes(memory[FLASH_BASE + index * 4 + i] for i in range(4))
        require(int.from_bytes(word, "little") == value(name) and value(name) & 1,
                f"Vector table does not bind the production {name}")
    binary = args.bin.read_bytes()
    require(len(binary) == max(memory) + 1 - FLASH_BASE, "Unexpected BIN size")
    require(all(byte == memory.get(FLASH_BASE + i, 0xFF) for i, byte in enumerate(binary)),
            "BIN differs from ELF or gaps are not filled with 0xff")
    print(f"OK {args.chip}: vectors, device limits, ELF/HEX/BIN agree; "
          f"HEX/BIN leave final 128 KiB NVS sector untouched ({len(memory)} load bytes).")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chip", choices=MEMORY, required=True)
    for name in ("elf", "hex", "bin"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        check(args)
    except (ValueError, KeyError, OSError, struct.error) as error:
        parser.exit(1, f"Firmware validation failed: {error}\n")


if __name__ == "__main__":
    main()
