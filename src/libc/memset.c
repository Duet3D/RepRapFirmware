/*
FUNCTION
	<<memset>>---set an area of memory

INDEX
	memset

SYNOPSIS
	#include <string.h>
	void *memset(void *<[dst]>, int <[c]>, size_t <[length]>);

DESCRIPTION
	This function converts the argument <[c]> into an unsigned
	char and fills the first <[length]> characters of the array
	pointed to by <[dst]> to the value.

RETURNS
	<<memset>> returns the value of <[dst]>.

PORTABILITY
<<memset>> is ANSI C.

    <<memset>> requires no supporting OS subroutines.

QUICKREF
	memset ansi pure
*/

#include <ecv_duet3d.h>
#include <string.h>
#include "local.h"

#define LBLOCKSIZE (sizeof(long))
#define UNALIGNED(X)   ((unsigned long)X & (LBLOCKSIZE - 1u))
#define TOO_SMALL(LEN) ((LEN) < LBLOCKSIZE)

void *
__inhibit_loop_to_libcall
memset (void *m,
	int c,
	size_t n) noexcept
{
  char *_ecv_array s = (char *_ecv_array) m;

#if !defined(PREFER_SIZE_OVER_SPEED) && !defined(__OPTIMIZE_SIZE__)
  unsigned int i;
  unsigned long buffer;
  unsigned long *_ecv_array aligned_addr;
  unsigned int d = c & 0xff;	/* To avoid sign extension, copy C to an
				   unsigned variable.  */

  while (UNALIGNED (s))
    {
      if (n-- != 0)
        *s++ = (char) c;
      else
        return m;
    }

  if (!TOO_SMALL (n))
    {
      /* If we get this far, we know that n is large and s is word-aligned. */
      aligned_addr = (unsigned long *_ecv_array) s;

      /* Store D into each char sized location in BUFFER so that
         we can set large blocks quickly.  */
      buffer = (d << 8) | d;
      buffer |= (buffer << 16);
      for (i = 32; i < LBLOCKSIZE * 8; i <<= 1)
        buffer = (buffer << i) | buffer;

      /* Unroll the loop.  */
      while (n >= LBLOCKSIZE*4)
        {
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          *aligned_addr++ = buffer;
          n -= 4*LBLOCKSIZE;
        }

      while (n >= LBLOCKSIZE)
        {
          *aligned_addr++ = buffer;
          n -= LBLOCKSIZE;
        }
      /* Pick up the remainder with a bytewise loop.  */
      s = (char *_ecv_array)aligned_addr;
    }

#endif /* not PREFER_SIZE_OVER_SPEED */

  while (n-- != 0)
    *s++ = (char) c;

  return m;
}
