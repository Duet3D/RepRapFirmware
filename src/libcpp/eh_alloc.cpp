// -*- C++ -*- Allocate exception objects.
// Copyright (C) 2001-2020 Free Software Foundation, Inc.
//
// This file is part of GCC.
//
// GCC is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3, or (at your option)
// any later version.
//
// GCC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

// This is derived from the C++ ABI for IA-64.  Where we diverge
// for cross-architecture compatibility are noted with "@@@".

#include <bits/c++config.h>
#include <cstdlib>
#if _GLIBCXX_HOSTED
#include <cstring>
#endif
#include <climits>
#include <exception>
#include "unwind-cxx.h"
#include <ext/concurrence.h>
#include <new>

#if _GLIBCXX_HOSTED
using std::free;
using std::malloc;
using std::memset;
#else
// In a freestanding environment, these functions may not be available
// -- but for now, we assume that they are.
extern "C" void *malloc (std::size_t);
extern "C" void free(void *);
extern "C" void *memset (void *, int, std::size_t);
#endif

using namespace __cxxabiv1;

// ??? How to control these parameters.

// Guess from the size of basic types how large a buffer is reasonable.
// Note that the basic c++ exception header has 13 pointers and 2 ints,
// so on a system with PSImode pointers we're talking about 56 bytes
// just for overhead.

#if INT_MAX == 32767
# define EMERGENCY_OBJ_SIZE	128
# define EMERGENCY_OBJ_COUNT	16
#elif !defined (_GLIBCXX_LLP64) && LONG_MAX == 2147483647
# define EMERGENCY_OBJ_SIZE	512
# define EMERGENCY_OBJ_COUNT	32
#else
# define EMERGENCY_OBJ_SIZE	1024
# define EMERGENCY_OBJ_COUNT	64
#endif

#ifndef __GTHREADS
# undef EMERGENCY_OBJ_COUNT
# define EMERGENCY_OBJ_COUNT	4
#endif

namespace __gnu_cxx
{
  void
  __freeres() { }
}

extern "C" void *
__cxxabiv1::__cxa_allocate_exception(std::size_t thrown_size) noexcept
{
  void *ret;

  thrown_size += sizeof (__cxa_refcounted_exception);
  ret = malloc (thrown_size);

  if (!ret)
    std::terminate ();

  memset (ret, 0, sizeof (__cxa_refcounted_exception));

  return (void *)((char *)ret + sizeof (__cxa_refcounted_exception));
}


extern "C" void
__cxxabiv1::__cxa_free_exception(void *vptr) noexcept
{
  char *ptr = (char *) vptr - sizeof (__cxa_refcounted_exception);
  free (ptr);
}


extern "C" __cxa_dependent_exception*
__cxxabiv1::__cxa_allocate_dependent_exception() noexcept
{
  __cxa_dependent_exception *ret;

  ret = static_cast<__cxa_dependent_exception*>
    (malloc (sizeof (__cxa_dependent_exception)));

  if (!ret)
    std::terminate ();

  memset (ret, 0, sizeof (__cxa_dependent_exception));

  return ret;
}


extern "C" void
__cxxabiv1::__cxa_free_dependent_exception
  (__cxa_dependent_exception *vptr) noexcept
{
  free (vptr);
}

// End
