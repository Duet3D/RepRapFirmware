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

const char *_ecv_array const Filament::FilamentAssignmentFile = "filaments.csv";
const char *_ecv_array const Filament::FilamentAssignmentFileComment = "RepRapFirmware filament assignment file v1";

Filament *_ecv_null Filament::filamentList = nullptr;


Filament::Filament(int extr) noexcept : extruder(extr)
{
	name.Clear();
	next = filamentList;
	filamentList = this;
}

void Filament::Load(const char *_ecv_array filamentName) noexcept
{
	name.copy(filamentName);
	Filament::SaveAssignments();
}

void Filament::Unload() noexcept
{
	name.Clear();
	Filament::SaveAssignments();
}

void Filament::LoadAssignment() noexcept
{
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileStore *_ecv_null const file = reprap.GetPlatform().OpenSysFile(FilamentAssignmentFile, OpenMode::read);
	if (file == nullptr)
	{
		// May happen, but not critical
		return;
	}

	bool filamentLoaded = false;
	char buffer[FilamentNameLength + 64];
	if (file->ReadLine(buffer, sizeof(buffer)) > 0 && StringStartsWith(buffer, FilamentAssignmentFileComment))
	{
		if (file->ReadLine(buffer, sizeof(buffer)) > 0)
		{
			while (file->ReadLine(buffer, sizeof(buffer)) > 0)
			{
				if (isDigit(buffer[0]) && StrToI32(buffer) == extruder)
				{
					const char *_ecv_array filament = buffer;
					while (*filament != 0)
					{
						if (*filament++ == ',')
						{
							break;
						}
					}

					name.copy(filament);
					filamentLoaded = true;
					break;
				}
			}
		}
	}

	file->Close();

	if (filamentLoaded)
	{
		// Filaments are mapped per extruder, so make sure the move key is updated
		reprap.MoveUpdated();
	}
#endif
}

/*static*/ void Filament::SaveAssignments() noexcept
{
	// Update the OM when the filament has been changed
	reprap.MoveUpdated();

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	FileStore *_ecv_null const file = reprap.GetPlatform().OpenSysFile(FilamentAssignmentFile, OpenMode::write);
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
	for (Filament *_ecv_null f = filamentList; f != nullptr; f = f->next)
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

/*static*/ Filament *_ecv_null Filament::GetFilamentByExtruder(const int extr) noexcept
{
	for (Filament *_ecv_null f = filamentList; f != nullptr; f = f->next)
	{
		if (f->GetExtruder() == extr)
		{
			return f;
		}
	}
	return nullptr;
}

// End
