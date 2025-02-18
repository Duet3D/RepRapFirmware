/*
 * malloc.h
 *
 *  Created on: 17 Dec 2024
 *      Author: David
 *
 *  This is a stub version of gcc's malloc.h file.
 *  When running eCv we can't include the original one form the gcc compiler installation because it relies on too many gcc-specific defines.
 *  Therefore add folder that contains this header file to the eCv include path.
 */

#ifndef SRC_LIBC_ECVINCLUDE_MALLOC_H_
#define SRC_LIBC_ECVINCLUDE_MALLOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>

/* This version of struct mallinfo must match the one in libc/stdlib/mallocr.c.  */

struct mallinfo {
  size_t arena;    /* total space allocated from system */
  size_t ordblks;  /* number of non-inuse chunks */
  size_t smblks;   /* unused -- always zero */
  size_t hblks;    /* number of mmapped regions */
  size_t hblkhd;   /* total space in mmapped regions */
  size_t usmblks;  /* unused -- always zero */
  size_t fsmblks;  /* unused -- always zero */
  size_t uordblks; /* total allocated space */
  size_t fordblks; /* total non-inuse space */
  size_t keepcost; /* top-most, releasable (via malloc_trim) space */
};

extern struct mallinfo mallinfo (void);

#ifdef __cplusplus
}
#endif

#endif /* SRC_LIBC_ECVINCLUDE_MALLOC_H_ */
