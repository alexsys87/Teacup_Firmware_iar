#!/usr/bin/env python3
"""Detect drift between the production GCC list/layout and the IAR projects."""
from pathlib import Path
import re
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent.parent
# Discard the comment before parsing the assignment.
text = (root / 'gcc/sources.mk').read_text().split('SOURCES :=', 1)[1]
sources = set(text.replace('\\', '').split()) - {'gcc/syscalls.c'}
for project in sorted((root / 'ewarm').glob('Teacup_*.ewp')):
    names = [node.findtext('name').replace('$PROJ_DIR$\\..\\', '').replace('\\', '/')
             for node in ET.parse(project).findall('.//file')]
    iar = {name for name in names if name.endswith('.c')}
    if iar != sources:
        raise SystemExit(f'{project.name}: C source list differs; '
                         f'missing in GCC: {sorted(iar - sources)}; '
                         f'GCC only: {sorted(sources - iar)}')
for name, flash_end, ram_end in (
        ('stm32f401xc', 0x0801ffff, 0x2000ffff),
        ('stm32f401xe', 0x0805ffff, 0x20017fff),
        ('stm32f411xe', 0x0805ffff, 0x2001ffff)):
    text = (root / f'cmsis/linker/{name}_flash.icf').read_text()
    expected = {'__ICFEDIT_region_ROM_start__': 0x08000000,
                '__ICFEDIT_region_ROM_end__': flash_end,
                '__ICFEDIT_region_RAM_start__': 0x20000000,
                '__ICFEDIT_region_RAM_end__': ram_end,
                '__ICFEDIT_size_cstack__': 4096,
                '__ICFEDIT_size_heap__': 0}
    for symbol, value in expected.items():
        match = re.search(r'define symbol ' + symbol + r'\s*=\s*(0x[0-9a-fA-F]+)', text)
        if not match or int(match[1], 16) != value:
            raise SystemExit(f'{name}: IAR {symbol} changed; review GCC layout')
print(f'OK: {len(sources)} production C files match both IAR projects; '
      'all three ICF layouts match GCC memory, stack and heap settings.')
