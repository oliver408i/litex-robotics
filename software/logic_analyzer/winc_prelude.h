#ifndef WINC_PRELUDE_H_
#define WINC_PRELUDE_H_

/* Force-included into every translation unit (Makefile -include). The WINC
 * driver (m2m_wifi.c et al) calls malloc/free without including <stdlib.h>,
 * and GCC 14+/15 makes implicit declarations a hard error -- so pull it in
 * globally. Guarded by __ASSEMBLER__ so the assembler pass (crt0.S) is not fed
 * C declarations. */
#ifndef __ASSEMBLER__
#include <stdlib.h>
#include <stdio.h>   /* printf prototype for CONF_WINC_PRINTF (driver logging) */
#endif

#endif /* WINC_PRELUDE_H_ */
