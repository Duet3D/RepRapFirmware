/*
 * Filament.cpp
 *
 *  Created on: 13 Jun 2017
 *      Author: Christian
 */

#include "Tool.h"
#include "Filament.h"

#include "RepRap.h"
#include "GCodes/GCodes.h"
#include "Platform.h"

#include <ctime>

const char * const Filament::FilamentAssignmentFile = "filaments.csv";
const char * const Filament::FilamentAssignmentFileComment = "RepRapFirmware filament assignment file v1";

Filament *Filament::filamentList = nullptr;


Filament::Filament(int extr) noexcept : extruder(extr)
{
	strcpy(name, "");
	this->used_length = 0.0;
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
#if HAS_MASS_STORAGE
# if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		// Filament configuration is saved on the SBC
		return;
	}
# endif

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
				if (isdigit(buffer[0]) && StrToI32(buffer) == extruder)
				{
#if 1
					char *p, *r;
					if ((p = strchr(buffer, ',')))
					{
						*p++ = 0;
						if (*p && (r = strchr(p, ',')))
						{
							*r++ = 0;
							if (*r && isdigit(*r))
								used_length = SafeStrtof(r);
						}
						SafeStrncpy(name, p, ARRAY_SIZE(name));
					}
#else

					char *filament = buffer, *s, *r;
					while (*filament != 0)
						if (*filament++ == ',')
							break;
					r = filament;
					while (*r++ != 0)
					{
						if (*filament == ',')
						{
							s = filament;
							*s++ = 0;

							while (*s != 0)
							{
								if (*s++ == ',')
								{
									*s++ = 0;
									if (*s && isdigit(*s))
									{
										used_length = SafeStrtof(s);
										break;

									}
								}
							}
							break;
						}
					}

					SafeStrncpy(name, filament, ARRAY_SIZE(name));
					break;
#endif
				}

			}
		}
	}

	file->Close();
#endif
}

void Filament::LoadFilamentUsage(const char *filamentUsageFilename, int extruder) noexcept
{
#if HAS_MASS_STORAGE
	FileStore * const file = reprap.GetPlatform().OpenSysFile(filamentUsageFilename, OpenMode::read);
	if (file == nullptr)
	{
		// Will only occur upon first usage
		return;
	}
	char bufferSpace[64];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));
	Filament *f = GetFilamentByExtruder(extruder);
	char buffer[64];
	if (file->ReadLine(buffer, sizeof(buffer)) > 0)
		if (f) // there better be a result or something odd has happened.
			f->used_length = SafeStrtof(buffer);
	file->Close();
#endif

}

void Filament::SaveFilamentUsage(const char *filamentUsageFilename, int extruder) noexcept
{
#if HAS_MASS_STORAGE
	FileStore * const file = reprap.GetPlatform().OpenSysFile(filamentUsageFilename, OpenMode::write);
	if (file == nullptr)
	{
		// Should never happen
		return;
	}
	char bufferSpace[64];
	StringRef buf(bufferSpace, ARRAY_SIZE(bufferSpace));
	Filament *f = GetFilamentByExtruder(extruder);
	if (f)
	{
		f->used_length += reprap.GetGCodes().GetRawExtruderTotalByDrive(extruder);
		buf.printf("%2.1f\n", (double)f->used_length);
		file->Write(buf.c_str());
	}
	file->Close();
#endif

}

void Filament::SaveAssignments() noexcept
{
	// Update the OM when the filament has been changed
	reprap.MoveUpdated();

#if HAS_MASS_STORAGE
# if HAS_LINUX_INTERFACE
	if (reprap.UsingLinuxInterface())
	{
		// Filament configuration is saved on the SBC
		return;
	}
# endif

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
	file->Write("extruder,filament,usage\n");
	for (Filament *f = filamentList; f != nullptr; f = f->next)
	{
		if (f->IsLoaded())
		{
			f->used_length = reprap.GetGCodes().GetRawExtruderTotalByDrive(f->GetExtruder());
			buf.printf("%d,%s,%2.1f\n", f->GetExtruder(), f->GetName(),(double)f->used_length);
			file->Write(buf.c_str());
		}
	}

	file->Close();
#endif
}

/*static*/ bool Filament::IsInUse(const char *filamentName) noexcept
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

/*static*/ Filament *Filament::GetFilamentByExtruder(const int drive) noexcept
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
