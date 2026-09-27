#include <stdio.h>
#include <string.h>
#include "pff.h"
void host_open(const char *p);
static char line[256]; static int len; static FILE *ref;
static uint8_t parser(uint8_t c) { if (c == '\n') { line[len] = 0; return 1; } if (len < 255) line[len++] = c; return 0; }
int main(int argc, char **argv) {
  (void)argc;
  FATFS fs; DIR dir; FILINFO fi; FRESULT r; int lines = 0, bad = 0, files = 0; char want[256];
  host_open(argv[1]); ref = fopen(argv[2], "r");
  if ((r = pf_mount(&fs))) { printf("mount %d\n", r); return 1; }
  if ((r = pf_opendir(&dir, "/"))) { printf("opendir %d\n", r); return 1; }
  for (;;) { if (pf_readdir(&dir, &fi) || !fi.fname[0]) break; printf("  %s %lu\n", fi.fname, (unsigned long)fi.fsize); files++; }
  if ((r = pf_open("TEST.GCO"))) { printf("open %d\n", r); return 1; }
  for (;;) {
    len = 0; r = pf_parse_line(parser);
    if (r == FR_END_OF_FILE) break;
    if (r) { printf("parse %d\n", r); return 1; }
    if (!fgets(want, sizeof want, ref)) { bad++; break; }
    want[strcspn(want, "\n")] = 0;
    if (strcmp(want, line)) { if (bad < 3) printf("diff line %d: '%s' vs '%s'\n", lines, line, want); bad++; }
    lines++;
  }
  /* Seek into the middle (like M26) and continue there. */
  { long off = 123457; FRESULT s = pf_lseek((DWORD)off); fseek(ref, off, SEEK_SET);
    len = 0; pf_parse_line(parser); fgets(want, sizeof want, ref); want[strcspn(want, "\n")] = 0;
    printf("lseek %d, partial line ok: %d\n", s, strcmp(want, line) == 0); if (strcmp(want, line)) bad++; }
  /* CR LF, empty lines skipped, last line without EOL, nothing after the
     end of the file (junk of JUNK.GCO in the rest of the sector). */
  { static const char *exp[] = { "G1 X1", "G1 X2", "   ", "M114" }; int k = 0;
    if ((r = pf_open("TAIL.GCO"))) { printf("open tail %d\n", r); return 1; }
    for (;;) { len = 0; r = pf_parse_line(parser); if (r) break;
      if (k >= 4 || strcmp(line, exp[k])) { printf("tail line %d: '%s'\n", k, line); bad++; } k++; }
    printf("tail: lines %d (4), end %s\n", k, r == FR_END_OF_FILE ? "FR_END_OF_FILE" : "?");
    if (k != 4 || r != FR_END_OF_FILE) bad++; }
  pf_unmount(&fs);
  printf("files %d, lines %d, mismatches %d, end %s\n", files, lines, bad, r == FR_END_OF_FILE ? "FR_END_OF_FILE" : "?");
  return bad != 0;
}
