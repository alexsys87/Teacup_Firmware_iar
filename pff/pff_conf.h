/*---------------------------------------------------------------------------/
/  Petit FatFs - Configuration file  R0.03a
/---------------------------------------------------------------------------*/
/* Changes for Teacup see pff.c. */

#ifndef PFCONF_DEF
#define PFCONF_DEF 8088 /* Revision ID */

/*---------------------------------------------------------------------------/
/ Function Configurations (0:Disable, 1:Enable)
/---------------------------------------------------------------------------*/

#define PF_USE_READ     1   /* pf_read() function (Teacup: disk_parsep()) */
#define PF_USE_DIR      1   /* pf_opendir() and pf_readdir() function (M20) */
#define PF_USE_LSEEK    1   /* pf_lseek() function (M26) */
#define PF_USE_WRITE    0   /* pf_write() function */

#define PF_FS_FAT12     0   /* FAT12 */
#define PF_FS_FAT16     1   /* FAT16 */
#define PF_FS_FAT32     1   /* FAT32 */


/*---------------------------------------------------------------------------/
/ Locale and Namespace Configurations
/---------------------------------------------------------------------------*/

#define PF_USE_LCC      0   /* Allow lower case ASCII and non-ASCII chars */

#define PF_CODE_PAGE    437
/* The PF_CODE_PAGE specifies the code page to be used on the target system.
/  SBCS code pages with PF_USE_LCC == 1 requiers a 128 byte of case conversion
/  table. When PF_USE_LCC == 0, PF_CODE_PAGE has no effect.
/  437 - U.S., 850 - Latin 1, 852 - Latin 2, 866 - Russian, ... see
/  the original pffconf.h.
*/

#endif /* PFCONF_DEF */
