/*
 * Filament.cpp
 *
 *  Created on: 13 Jun 2017
 *      Author: Christian
 */

#include "Tool.h"
#include "Filament.h"

#include "RepRap.h"
#include "Platform.h"

#include <ctime>

const char * const Filament::FilamentAssignmentFile = "filaments.csv";
const char * const Filament::FilamentAssignmentFileComment = "RepRapFirmware filament assignment file v1";

Filament *Filament::filamentList = nullptr;


Filament::Filament(int extr) : extruder(extr)
{
	strcpy(name, "");

	next = filamentList;
	filamentList = this;
}

void Filament::Load(const char *filamentName)
{
	SafeStrncpy(name, filamentName, ARRAY_SIZE(name));
	Filament::SaveAssignments();
}

void Filament::Unload()
{
	strcpy(name, "");
	Filament::SaveAssignments();
}

void Filament::LoadAssignment()
{
	FileStore *file = reprap.GetPlatform().OpenSysFile(FilamentAssignmentFile, OpenMode::read);
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
				if (isdigit(buffer[0]) && atoi(buffer) == extruder)
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
}

/*static*/ void Filament::SaveAssignments()
{
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
	if (reprap.GetPlatform().IsDateTimeSet())
	{
		time_t timeNow = reprap.GetPlatform().GetDateTime();
		const struct tm * const timeInfo = gmtime(&timeNow);
		buf.catf(" generated at %04u-%02u-%02u %02u:%02u",
						timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min);
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
}

/*static*/ bool Filament::IsInUse(const char *filamentName)
{
	for (Filament *f = filamentList; f != nullptr; f = f->next)
	{
		if (StringEqualsIgnoreCase(f->name, filamentName))
		{
			return true;
		}
	}
	return false;
}

/*static*/ Filament *Filament::GetFilamentByExtruder(const int drive)
{
	for (Filament *f = filamentList; f != nullptr; f = f->next)
	{
		if (f->GetExtruder() == drive)
		{
			return f;
		}
	}
	return nullptr;
}

// End
