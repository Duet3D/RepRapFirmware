/*
 * Filament.cpp
 *
 *  Created on: 13 Jun 2017
 *      Author: Christian
 */

#include "Tool.h"
#include "Filament.h"

#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <Platform/Tasks.h>

#include <ctime>

const char * const Filament::FilamentAssignmentFile = "filaments.csv";
const char * const Filament::FilamentAssignmentFileComment = "RepRapFirmware filament assignment file v1";

Filament *Filament::filamentList = nullptr;


Filament::Filament(int extr) noexcept : extruder(extr)
{
	strcpy(name, "");

	next = filamentList;
	filamentList = this;
}

void Filament::Load(const char *filamentName) noexcept
{
	SafeStrncpy(name, filamentName, ARRAY_SIZE(name));
	Filament::SaveAssignments();
}

void Filament::Unload() noexcept
{
	strcpy(name, "");
	Filament::SaveAssignments();
}

void Filament::LoadAssignment() noexcept
{
#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	FileStore * const file = reprap.GetPlatform().OpenSysFile(FilamentAssignmentFile, OpenMode::read);
	if (file == nullptr)
	{
		// May happen, but not critical
		return;
	}

	char buffer[FilamentNameLength + 64];
	if (file->ReadLine(buffer, sizeof(buffer)) > 0 && StringStartsWith(buffer, FilamentAssignmentFileComment))
	{
		if (file->ReadLine(buffer, sizeof(buffer)) > 0)
		{
			while (file->ReadLine(buffer, sizeof(buffer)) > 0)
			{
				if (isdigit(buffer[0]) && StrToI32(buffer) == extruder)
				{
					const char *filament = buffer;
					while (*filament != 0)
					{
						if (*filament++ == ',')
						{
							break;
						}
					}

					SafeStrncpy(name, filament, ARRAY_SIZE(name));
					break;
				}
			}
		}
	}

	file->Close();
#endif
}

/*static*/ void Filament::SaveAssignments() noexcept
{
	// Update the OM when the filament has been changed
	reprap.MoveUpdated();

#if HAS_MASS_STORAGE || HAS_LINUX_INTERFACE
	FileStore * const file = reprap.GetPlatform().OpenSysFile(FilamentAssignmentFile, OpenMode::write);
	if (file == nullptr)
	{
		// Should never happen
		return;
	}

	char bufferSpace[FilamentNameLength + 64];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));

	// Write header
	buf.copy(FilamentAssignmentFileComment);
	tm timeInfo;
	if (reprap.GetPlatform().GetDateTime(timeInfo))
	{
		buf.catf(" generated at %04u-%02u-%02u %02u:%02u",
						timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday, timeInfo.tm_hour, timeInfo.tm_min);
	}
	buf.cat('\n');
	file->Write(buf.c_str());

	// Write column headers and one row for each loaded filament
	file->Write("extruder,filament\n");
	for (Filament *f = filamentList; f != nullptr; f = f->next)
	{
		if (f->IsLoaded())
		{
			buf.printf("%d,%s\n", f->GetExtruder(), f->GetName());
			file->Write(buf.c_str());
		}
	}

	file->Close();
#endif
}

/*static*/ Filament *Filament::GetFilamentByExtruder(const int extr) noexcept
{
	for (Filament *f = filamentList; f != nullptr; f = f->next)
	{
		if (f->GetExtruder() == extr)
		{
			return f;
		}
	}
	return nullptr;
}

// End
