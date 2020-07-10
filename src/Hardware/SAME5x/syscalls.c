/**
  * \file syscalls.c
  *
  * Implementation of newlib syscall.
  *
  */

#include <stdlib.h>
#include <sys/stat.h>

// Helper macro to mark unused parameters and prevent compiler warnings.
// Appends _UNUSED to the variable name to prevent accidentally using them.
#undef UNUSED
#ifdef __GNUC__
# define UNUSED(x) x ## _UNUSED __attribute__((__unused__))
#endif

/*----------------------------------------------------------------------------
 *        Exported variables
 *----------------------------------------------------------------------------*/

#undef errno
int errno = 0;

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

extern int errno;
extern int _end;

extern caddr_t _sbrk(int incr);
extern int     link(char *old, char *_new);
extern int     _close(int file);
extern int     _fstat(int file, struct stat *st);
extern int     _isatty(int file);
extern int     _lseek(int file, int ptr, int dir);
extern void    _exit(int status);
extern void    _kill(int pid, int sig);
extern int     _getpid(void);

/**
 * \brief Replacement of C library of _sbrk
 */
extern caddr_t _sbrk(int incr)
{
	static unsigned char *heap = NULL;
	unsigned char *       prev_heap;

	if (heap == NULL) {
		heap = (unsigned char *)&_end;
	}
	prev_heap = heap;

	heap += incr;

	return (caddr_t)prev_heap;
}

/**
 * \brief Replacement of C library of link
 */
extern int link(char *old, char *_new)
{
	(void)old, (void)_new;
	return -1;
}

/**
 * \brief Replacement of C library of _close
 */
extern int _close(int file)
{
	(void)file;
	return -1;
}

/**
 * \brief Replacement of C library of _fstat
 */
extern int _fstat(int file, struct stat *st)
{
	(void)file;
	st->st_mode = S_IFCHR;

	return 0;
}

/**
 * \brief Replacement of C library of _isatty
 */
extern int _isatty(int file)
{
	(void)file;
	return 1;
}

/**
 * \brief Replacement of C library of _lseek
 */
extern int _lseek(int file, int ptr, int dir)
{
	(void)file, (void)ptr, (void)dir;
	return 0;
}

/**
 * \brief Replacement of C library of _exit
 */
extern void _exit(int status)
{
	for (;;) { }
}

/**
 * \brief Replacement of C library of _kill
 */
extern void _kill(UNUSED(int pid), UNUSED(int sig))
{
	return;
}

/**
 * \brief Replacement of C library of _getpid
 */
extern int _getpid(void)
{
	return -1;
}

extern int _read(UNUSED(int file), UNUSED(char *ptr), UNUSED(int len))
{
    return 0 ;
}

extern int _write(UNUSED(int file), UNUSED(char *ptr), int len)
{
	return len;
}

// End
