/*
 * GCodes6.cpp
 *
 *  Created on: 19 Jul 2022
 *      Author: David
 *  This file defined member functions of the GCodes class that handle bed probing and height maps
 */

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Movement/Move.h>
#include <Endstops/Zprobe.h>

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex) then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's coordinates.  If no Z is specified use the machine's coordinates.
// If it is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0) then that value is used.
// If it's less than SILLY_Z_VALUE the bed is probed and that value is used.
// We already own the movement lock before this is called.
GCodeResult GCodes::ExecuteG30(GCodeBuffer& gb, const StringRef& reply)
{
	g30SValue = (gb.Seen('S')) ? gb.GetIValue() : -4;		// S-4 or lower is equivalent to having no S parameter
	const MovementState& ms = GetMovementState(gb);
	if (g30SValue == -2 && ms.currentTool == nullptr)
	{
		reply.copy("G30 S-2 commanded with no tool selected");
		return GCodeResult::error;
	}

	g30HValue = (gb.Seen('H')) ? gb.GetFValue() : 0.0;
	g30ProbePointIndex = -1;
	bool seenP = false;
	gb.TryGetIValue('P', g30ProbePointIndex, seenP);
	if (seenP)
	{
		if (g30ProbePointIndex < 0 || g30ProbePointIndex >= (int)MaxProbePoints)
		{
			reply.copy("Z probe point index out of range");
			return GCodeResult::error;
		}
		else
		{
			// Set the specified probe point index to the specified coordinates
			const float x = (gb.Seen(axisLetters[X_AXIS])) ? gb.GetFValue() : ms.currentUserPosition[X_AXIS];
			const float y = (gb.Seen(axisLetters[Y_AXIS])) ? gb.GetFValue() : ms.currentUserPosition[Y_AXIS];
			const float z = (gb.Seen(axisLetters[Z_AXIS])) ? gb.GetFValue() : ms.currentUserPosition[Z_AXIS];
			reprap.GetMove().SetXYBedProbePoint((size_t)g30ProbePointIndex, x, y);

			if (z > SILLY_Z_VALUE)
			{
				// Just set the height error to the specified Z coordinate
				reprap.GetMove().SetZBedProbePoint((size_t)g30ProbePointIndex, z, false, false);
				if (g30SValue >= -1)
				{
					return GetGCodeResultFromError(reprap.GetMove().FinishedBedProbing(g30SValue, reply));
				}
			}
			else
			{
				// Do a Z probe at the specified point.
				const auto zp = SetZProbeNumber(gb, 'K');		// may throw, so do this before changing the state
				gb.SetState(GCodeState::probingAtPoint0);
				if (zp->GetProbeType() != ZProbeType::blTouch)
				{
					DeployZProbe(gb);
				}
			}
		}
	}
	else
	{
		// G30 without P parameter. This probes the current location starting from the current position.
		// If S=-1 it just reports the stopped height, else it resets the Z origin.
		const auto zp = SetZProbeNumber(gb, 'K');				// may throw, so do this before changing the state
		InitialiseTaps(zp->HasTwoProbingSpeeds());
		gb.SetState(GCodeState::probingAtPoint2a);
		if (zp->GetProbeType() != ZProbeType::blTouch)
		{
			DeployZProbe(gb);
		}
	}
	return GCodeResult::ok;
}

// Set up currentZProbeNumber and return the probe
ReadLockedPointer<ZProbe> GCodes::SetZProbeNumber(GCodeBuffer& gb, char probeLetter) THROWS(GCodeException)
{
	const uint32_t probeNumber = (gb.Seen(probeLetter)) ? gb.GetLimitedUIValue(probeLetter, MaxZProbes) : 0;
	auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(probeNumber);
	if (zp.IsNull())
	{
		gb.ThrowGCodeException("Z probe %u not found", probeNumber);
	}
	currentZProbeNumber = (uint8_t)probeNumber;
	return zp;
}

// Decide which device to display a message box on
MessageType GCodes::GetMessageBoxDevice(GCodeBuffer& gb) const
{
	MessageType mt = gb.GetResponseMessageType();
	if (mt == GenericMessage)
	{
		// Command source was the file being printed, or a trigger. Send the message to PanelDue if there is one, else to the web server.
		mt = (lastAuxStatusReportType >= 0) ? AuxMessage : HttpMessage;
	}
	return mt;
}

void GCodes::DoManualProbe(GCodeBuffer& gb, const char *message, const char *title, const AxesBitmap axes)
{
	if (Push(gb, true))													// stack the machine state including the file position and set the state to GCodeState::normal
	{
		gb.WaitForAcknowledgement();									// flag that we are waiting for acknowledgement
		const MessageType mt = GetMessageBoxDevice(gb);
		platform.SendAlert(mt, message, title, 2, 0.0, axes);
	}
}

// Do a manual bed probe. On entry the state variable is the state we want to return to when the user has finished adjusting the height.
void GCodes::DoManualBedProbe(GCodeBuffer& gb)
{
	DoManualProbe(gb, "Adjust height until the nozzle just touches the bed, then press OK", "Manual bed probing", AxesBitmap::MakeFromBits(Z_AXIS));
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
GCodeResult GCodes::ProbeGrid(GCodeBuffer& gb, const StringRef& reply)
{
	if (!defaultGrid.IsValid())
	{
		reply.copy("No valid grid defined for bed probing");
		return GCodeResult::error;
	}

	if (!AllAxesAreHomed())
	{
		reply.copy("Must home printer before bed probing");
		return GCodeResult::error;
	}

	const auto zp = SetZProbeNumber(gb, 'K');			// may throw, so do this before changing the state

	reprap.GetMove().AccessHeightMap().SetGrid(defaultGrid);
	ClearBedMapping();
	gridAxis0index = gridAxis1index = 0;

	gb.SetState(GCodeState::gridProbing1);
	if (zp->GetProbeType() != ZProbeType::blTouch)
	{
		DeployZProbe(gb);
	}
	return GCodeResult::ok;
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

GCodeResult GCodes::LoadHeightMap(GCodeBuffer& gb, const StringRef& reply)
{
	ClearBedMapping();

	String<MaxFilenameLength> heightMapFileName;
	bool seen = false;
	gb.TryGetQuotedString('P', heightMapFileName.GetRef(), seen);
	if (!seen)
	{
		heightMapFileName.copy(DefaultHeightMapFile);
	}

	String<MaxFilenameLength> fullName;
	platform.MakeSysFileName(fullName.GetRef(), heightMapFileName.c_str());
	FileStore * const f = MassStorage::OpenFile(fullName.c_str(), OpenMode::read, 0);
	if (f == nullptr)
	{
		reply.printf("Height map file %s not found", fullName.c_str());
		return GCodeResult::error;
	}
	reply.printf("Failed to load height map from file %s: ", fullName.c_str());	// set up error message to append to

	const bool err = reprap.GetMove().LoadHeightMapFromFile(f, fullName.c_str(), reply);
	f->Close();

	ActivateHeightmap(!err);
	if (err)
	{
		return GCodeResult::error;
	}

	reply.Clear();						// get rid of the error message
	if (!zDatumSetByProbing && platform.GetZProbeOrDefault(0)->GetProbeType() != ZProbeType::none)	//TODO store Z probe number in height map
	{
		reply.copy("the height map was loaded when the current Z=0 datum was not determined by probing. This may result in a height offset.");
		return GCodeResult::warning;
	}

	return GCodeResult::ok;
}

// Save the height map and append the success or error message to 'reply', returning true if an error occurred
bool GCodes::TrySaveHeightMap(const char *filename, const StringRef& reply) const noexcept
{
	String<MaxFilenameLength> fullName;
	platform.MakeSysFileName(fullName.GetRef(), filename);
	FileStore * const f = MassStorage::OpenFile(fullName.c_str(), OpenMode::write, 0);
	bool err;
	if (f == nullptr)
	{
		reply.catf("Failed to create height map file %s", fullName.c_str());
		err = true;
	}
	else
	{
		err = reprap.GetMove().SaveHeightMapToFile(f, fullName.c_str());
		f->Close();
		if (err)
		{
			MassStorage::Delete(fullName.c_str(), false);
			reply.catf("Failed to save height map to file %s", fullName.c_str());
		}
		else
		{
			reply.catf("Height map saved to file %s", fullName.c_str());
		}
	}
	return err;
}

// Save the height map to the file specified by P parameter
GCodeResult GCodes::SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const
{
	// No need to check if we're using the SBC interface here, because TrySaveHeightMap does that already
	if (gb.Seen('P'))
	{
		String<MaxFilenameLength> heightMapFileName;
		gb.GetQuotedString(heightMapFileName.GetRef());
		return GetGCodeResultFromError(TrySaveHeightMap(heightMapFileName.c_str(), reply));
	}
	return GetGCodeResultFromError(TrySaveHeightMap(DefaultHeightMapFile, reply));
}

# if SUPPORT_PROBE_POINTS_FILE

// Load map of reachable probe points from file
GCodeResult GCodes::LoadProbePointsMap(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	ClearBedMapping();

	String<MaxFilenameLength> probePointsFileName;
	bool seen = false;
	gb.TryGetQuotedString('P', probePointsFileName.GetRef(), seen);
	if (!seen)
	{
		probePointsFileName.copy(DefaultProbeProbePointsFile);
	}

	String<MaxFilenameLength> fullName;
	platform.MakeSysFileName(fullName.GetRef(), probePointsFileName.c_str());
	FileStore * const f = MassStorage::OpenFile(fullName.c_str(), OpenMode::read, 0);
	if (f == nullptr)
	{
		reply.printf("Probe points file %s not found", fullName.c_str());
		return GCodeResult::error;
	}
	reply.printf("Failed to load probe points from file %s: ", fullName.c_str());	// set up error message to append to

	const bool err = reprap.GetMove().LoadProbePointsFromFile(f, fullName.c_str(), reply);
	f->Close();

	return (err) ? GCodeResult::error : GCodeResult::ok;
}

#endif

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Stop using bed compensation
void GCodes::ClearBedMapping()
{
	reprap.GetMove().SetIdentityTransform();
	for (MovementState& ms : moveStates)
	{
		reprap.GetMove().GetCurrentUserPosition(ms.coords, 0, ms.currentTool);
		ToolOffsetInverseTransform(ms);		// update user coordinates to remove any height map offset there was at the current position
	}
}

// End
