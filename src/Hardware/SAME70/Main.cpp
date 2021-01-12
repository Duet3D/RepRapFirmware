/*
 * Main.cpp
 *  Program entry point
 *  Created on: 11 Jul 2020
 *      Author: David
 *  License: GNU GPL version 3
 */

#include <Core.h>

// Program initialisation
void AppInit() noexcept
{
}

// syscalls.h must be included by exactly one .cpp file in the project
#include <syscalls.h>

[[noreturn]] void OutOfMemoryHandler() noexcept
{
	while (true) { }
}

extern "C" [[noreturn]] void __cxa_pure_virtual() noexcept
{
	while (true) { }
}

extern "C" [[noreturn]] void __cxa_deleted_virtual() noexcept
{
	while (true) { }
}

// End
