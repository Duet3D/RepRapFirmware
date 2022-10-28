/*
 * errno.c
 *
 *  Created on: 1 Oct 2022
 *      Author: David
 *
 *  This file replaces the one in newlib in order to avoid pulling the reent struct
 */

static int globalErrno = 0;

int * __errno () noexcept
{
	return &globalErrno;
}

// End
