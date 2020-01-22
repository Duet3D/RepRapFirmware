/*
 * vterminate.cc
 *
 *  Created on: 22 Jan 2020
 *      Author: David
 */

#include <cstdlib>

namespace __gnu_cxx
{
  // A replacement for the standard terminate_handler which prints
  // more information about the terminating exception (if any) on
  // stderr.
  void __verbose_terminate_handler()
  {
	  abort();
  }
}

// End
