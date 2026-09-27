#!/bin/sh
# Host test of the Petit FatFs port (pff/): FAT16 and FAT32 images made with
# mkfs.fat/mcopy (dosfstools, mtools), a 6000 line G-code file read with
# pf_parse_line(), directory listing, pf_lseek() into the middle. TAIL.GCO:
# CR LF, empty lines, last line without line end, and junk after the end
# of the file in its last sector (left there by a deleted file).
set -e
cd "$(dirname "$0")"
P=../../pff
python3 -c "
import random
random.seed(1)
with open('test.gco','w') as f:
    for i in range(6000):
        f.write('G1 X%.3f Y%.3f E%.5f F%d ; line %d%s\n' % (random.uniform(0,200), random.uniform(0,200), random.uniform(0,2), random.choice([1200,3000,9000]), i, 'x'*random.randint(0,40)))
"
echo hi > readme.txt
python3 -c "
open('junk.gco','w').write('M112 junk after the end of file\n' * 64)
open('tail.gco','wb').write(b'G1 X1\r\n\r\n\nG1 X2\n\n   \nM114')
"
for fat in 16 32; do
  size=$([ $fat = 16 ] && echo 32768 || echo 65536)
  rm -f img$fat
  mkfs.fat -C -F $fat img$fat $size > /dev/null
  mcopy -i img$fat test.gco ::TEST.GCO
  mcopy -i img$fat readme.txt ::README.TXT
  mcopy -i img$fat junk.gco ::JUNK.GCO
  mdel -i img$fat ::JUNK.GCO
  mcopy -i img$fat tail.gco ::TAIL.GCO
  gcc -std=c11 -Wall -Wextra -I. -I$P -o test_pff$fat $P/pff.c host_diskio.c test_pff.c
  echo "== FAT$fat"
  ./test_pff$fat img$fat test.gco
done
rm -f img16 img32 test_pff16 test_pff32 test.gco readme.txt junk.gco tail.gco
