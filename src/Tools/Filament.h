/*
 * Filament.h
 *
 *  Created on: 13 Jun 2017
 *      Author: Christian
 */

#ifndef SRC_TOOLS_FILAMENT_H
#define SRC_TOOLS_FILAMENT_H

#include <cstddef>

const size_t FilamentNameLength = 32;


class Filament
{
public:
	Filament(int extr);

	int GetExtruder() const { return extruder; }				// Returns the assigned extruder drive
	const char *GetName() const { return name; }				// Returns the name of the currently loaded filament

	// TODO: Add support for filament counters, tool restrictions etc.
	// These should be stored in a dedicate file per filament directory like /filaments/<material>/filament.json

	bool IsLoaded() const { return (name[0] != 0); }			// Returns true if a valid filament is assigned to this instance
	void Load(const char *filamentName);						// Loads filament parameters from the SD card
	void Unload();												// Unloads the current filament

	void LoadAssignment();										// Read the assigned material for the given extruder from the SD card

	static void SaveAssignments();								// Rewrite the CSV file containing the extruder <-> filament assignments
	static bool IsInUse(const char *filamentName);				// Check if this material is already assigned to an extruder
	static Filament *GetFilamentByExtruder(const int extruder);	// Retrieve the Filament instance assigned to the given extruder drive

private:
	static const char * const FilamentAssignmentFile;			// In which file the extruder <-> filament assignments are stored
	static const char * const FilamentAssignmentFileComment;	// The comment we write at the start of this file to ensure its integrity

	static Filament *filamentList;
	Filament *next;

	int extruder;
	char name[FilamentNameLength];
};

#endif
