/* Match IAR's zero-byte heap. Firmware must not depend on malloc/newlib I/O.
 * All other bare-metal syscall stubs are provided by libnosys.
 */
#include <errno.h>
#include <stddef.h>

void *_sbrk(ptrdiff_t increment)
{
    (void)increment;
    errno = ENOMEM;
    return (void *)-1;
}
