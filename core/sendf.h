
#ifndef _SENDF_H
#define _SENDF_H

#include "stdint.h"


// No __attribute__ ((format (printf, 1, 2)) here because %q isn't supported.
void sendf_P(void (*writechar)(uint8_t), const char *format_P, ...);

#endif /* _SENDF_H */
