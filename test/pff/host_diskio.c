#include <stdio.h>
#include <string.h>
#include "pff_diskio.h"
static FILE *img;
void host_open(const char *p) { img = fopen(p, "rb"); }
DSTATUS disk_initialize(void) { return img ? 0 : STA_NOINIT; }
static int rd(DWORD sector, BYTE *b) { fseek(img, (long)sector * 512, SEEK_SET); return fread(b, 1, 512, img) == 512; }
DRESULT disk_readp(BYTE *buff, DWORD sector, UINT offset, UINT count) {
  BYTE b[512]; if (!rd(sector, b)) return RES_ERROR;
  if (buff) memcpy(buff, b + offset, count);
  return RES_OK;
}
DRESULT disk_parsep(DWORD sector, UINT offset, UINT *count, uint8_t (*parser)(uint8_t)) {
  BYTE b[512]; UINT n = 0; DRESULT r = RES_OK;
  if (!rd(sector, b)) return RES_ERROR;
  while (offset + n < 512) { uint8_t c = b[offset + n++]; if (parser(c)) { r = RES_EOL_FOUND; break; } }
  *count = n; return r;
}
