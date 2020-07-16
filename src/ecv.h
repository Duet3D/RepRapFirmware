/* Header file for supporting Escher C Verifier annotations in C and C++ files
 * Copyright(c) 2016 Escher Technologies Limited
 * Escher Technologies Limited permits free use and distribution of this file for the purpose of annotating C and C++ programs
 * in a manner compatible with Escher C Verifier, subject to the following conditions:
 * 1. If you make any alterations to this file, you must make this clear by adding additional comments,
 *    and you must reproduce these conditions and copyright notice in full.
 * 2. No warranty or support is provided except in accordance with the terms of a paid-up support & maintenance agreement
 *    between the user and Escher Technologies Limited.
 *
 * For documentation of the eCv keywords, see the eCv Reference Manual at http://eschertech.com/support/perfect_developer_self_help.php
 * For examples of their use, see http://eschertech.com/articles/index.php
 */

#if !defined(__ECV_H_INCLUDED__)

#define __ECV_H_INCLUDED__

#ifdef __ECV__
#pragma ECV noverify
#endif

/* Define the "normal" versions of the eCv keywords.
 * If any of these clash with identifiers in the user program, you can #undef them after including this file. */

#define any				_ecv_any
#define array			_ecv_array
#define assert			_ecv_assert
#define assume			_ecv_assume
#define decrease		_ecv_decrease
#define disjoint		_ecv_disjoint
#define exists			_ecv_exists
#define forall			_ecv_forall
#define ghost  			_ecv_ghost
#define holds			_ecv_holds
#define idiv			_ecv_idiv
#define imod			_ecv_imod
#define in				_ecv_in
#define integer			_ecv_integer
#define invariant		_ecv_invariant
#define keep			_ecv_keep
#define let				_ecv_let
#define maxof			_ecv_maxof
#define minof			_ecv_minof
#define min_sizeof		_ecv_min_sizeof
#define not_null		_ecv_not_null
#define null			_ecv_null
#define old				_ecv_old
#define out				_ecv_out
#define over			_ecv_over
#define post			_ecv_post
#define pre				_ecv_pre
#define result			_ecv_result
#define returns			_ecv_returns
#define some			_ecv_some
#define spec			_ecv_spec
#define that			_ecv_that
#define those			_ecv_those
#define value			_ecv_value
#define writes			_ecv_writes
#define yield			_ecv_yield
#define zero_init		_ecv_zero_init

#if defined(__ECV__) || !defined(__STDC_VERSION__) || (__STDC_VERSION__ < 19901L)
# define restrict		_ecv_restrict
#endif

#if defined(__cplusplus)
# define early			_ecv_early
# define from			_ecv_from
# if defined(__ECV__) || (__cplusplus < 201103L)
// The C++'11 final, nullptr and override keywords are available whenever processing C++ with eCv
#  define final			_ecv_final
#  define nullptr		_ecv_nullptr
#  define override		_ecv_override
# endif
#endif

#if defined(__ECV__)

/************************ The following definitions are only active when running eCv **********************/

#if defined(_MSC_VER)

/******************** Definitions for Microsoft compilers *********************/

/* Define Microsoft compiler-specific keywords as expanding to nothing */
#define __declspec(_x)
#define __w64
#define __inline		inline
#define __cdecl
#define __fastcall
#define __stdcall

#if _MSC_VER >= 1400

/ * Microsoft file crtdefs.h (which is included by nearly everything else) contains some references to types not defined there.
 * To avoid problems, we include crtdefs.h here along with the files that define the missing types. */
#define __STDC_WANT_SECURE_LIB__	(0)	/* need this to undo some inline declarations that don't compile in eCv */
#define _CRT_SECURE_NO_WARNINGS		(1)
#include "crtdefs.h"
#include "locale.h"						/* defines "struct lconv" */
struct __lc_time_data { int _x; };		/* this is defined only in the CRT source */
struct threadmbcinfostruct { int _x; };	/* this is defined only in the CRT source */
#endif

#endif		/* end Microsoft compiler specific */

#if defined(__GNUC__)

/*********************** Definitions for Gnu Compiler Collection C and C++ compilers ********************/

#define __inline__		inline
#define __restrict__	_ecv_restrict

/* Hide gcc __attribute__ keyword from eCv.
 * WARNING: some header files (e.g. _mingw.h) may "#undef __attribute__", which undoes this. */
#define __attribute__(_x)

/* The following are needed to handle gcc's implementation of variable argument lists */
typedef struct __builtin_va_list__ { int x; } __builtin_va_list;
extern void __builtin_va_start(__builtin_va_list, const char*);
extern void __builtin_va_end(__builtin_va_list);

#endif		/* end gcc specific */

#if defined HI_TECH_C

/****************************** Definitions for HiTech C compilers ********************************/

#define bit bool			/* define HiTech bit type as equivalent to bool */

/* Define the HiTech C extra type qualifiers */
#define interrupt		_ecv_interrupt	/* functions may be flagged as interrupt routines */
#define persistent
#define near
#define bank0
#define bank1
#define bank2
#define bank3
#define eeprom
#define __interrupt		_ecv_interrupt	/* functions may be flagged as interrupt routines */
#define __persistent
#define __near
#define __bank0
#define __bank1
#define __bank2
#define __bank3
#define __eeprom

#define asm(_x)				/* hide assembler from eCv */

#endif		/* end HiTech compiler specific */

#if defined __IAR_SYSTEMS_ICC__

/****************************** Definitions for IAR C/C++ compilers ********************************/

/* Define the IAR extra keywords */
#define __cc_version1
#define __cc_version2
#define __data16
#define __data20
#define __interrupt		_ecv_interrupt	/* functions may be flagged as interrupt routines */
#define __intrinsic
#define __monitor
#define __no_init
#define __no_pic
#define __noreturn
#define __persistent
#define __ramfunc
#define __raw
#define __regvar
#define __root
#define __ro_placement
#define __save_reg20
#define __task

#endif		/* end IAR compiler specific */

/**************************** End of compiler-specific definitions ***************************/

#else		/* not running eCv */

/* Define eCv macros as expanding to nothing */
#define _ecv_array
#define _ecv_assert(_x)		((void)0)
#define _ecv_assume(_x)
#define _ecv_change(_x)
#define _ecv_decrease(_x)
#define _ecv_ghost(_x)
#define _ecv_interrupt
#define _ecv_invariant(_x)
#define _ecv_keep(_x)
#define _ecv_not_null(_x)	(_x)
#define _ecv_null
#define _ecv_out
#define _ecv_pass			((void)0)
#define _ecv_post(_x)
#define _ecv_pre(_x)
#define _ecv_restrict
#define _ecv_returns(_x)
#define _ecv_spec
#define _ecv_writes(_x)

#if defined(__cplusplus)
#define _ecv_early
#define _ecv_final
#define _ecv_from
#define _ecv_nullptr		(0)
#define _ecv_override
#endif

#endif		/* end "if defined(__ECV__) ... else ..." */

#endif		/* end header guard */

/* End of file */
