/*
 * GCodes6.cpp
 *
 *  Created on: 19 Jul 2022
 *      Author: David
 *
 *  This file defined member functions of the GCodes class that handle bed probing and height maps
 */

#include "GCodes.h"
#include "GCodeBuffer/GCodeBuffer.h"
#include <Movement/Move.h>
#include <Endstops/ZProbe.h>

// This is called to execute a G30.
// It sets wherever we are as the probe point P (probePointIndex) then probes the bed, or gets all its parameters from the arguments.
// If X or Y are specified, use those; otherwise use the machine's coordinates.  If no Z is specified use the machine's coordinates.
// If it is specified and is greater than SILLY_Z_VALUE (i.e. greater than -9999.0) then that value is used.
// If it's less than SILLY_Z_VALUE the bed is probed and that value is used.
// We already own the movement lock before this is called.
GCodeResult GCodes::ExecuteG30(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);

	g30SValue = (gb.Seen('S')) ? gb.GetIValue() : -4;		// S-4 or lower is equivalent to having no S parameter
	if (g30SValue == -2 && ms.currentTool == nullptr)
	{
		reply.copy("G30 S-2 commanded with no tool selected");
		return GCodeResult::error;
	}

	g30HValue = (gb.Seen('H')) ? gb.GetFValue() : 0.0;
	g30ProbePointIndex = -1;

	GCodeState newState = GCodeState::normal;
#if SUPPORT_ASYNC_MOVES
	AxesBitmap axesMoving;
#endif

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
			const float x = (gb.Seen(axisLetters[X_AXIS]))
#if SUPPORT_ASYNC_MOVES
							? (axesMoving.SetBit(X_AXIS), gb.GetFValue())
#else
							? gb.GetFValue()
#endif
								: ms.currentUserPosition[X_AXIS];
			const float y = (gb.Seen(axisLetters[Y_AXIS]))
#if SUPPORT_ASYNC_MOVES
							? (axesMoving.SetBit(Y_AXIS), gb.GetFValue())
#else
							? gb.GetFValue()
#endif
								: ms.currentUserPosition[Y_AXIS];
			const float z = (gb.Seen(axisLetters[Z_AXIS])) ? gb.GetFValue() : ms.currentUserPosition[Z_AXIS];
			reprap.GetMove().SetXYBedProbePoint((size_t)g30ProbePointIndex, x, y);

			if (z > SILLY_Z_VALUE)
			{
				// Just set the height error to the specified Z coordinate
				reprap.GetMove().SetZBedProbePoint((size_t)g30ProbePointIndex, z, false, false);
				if (g30SValue >= -1)
				{
					return GetGCodeResultFromError(reprap.GetMove().FinishedBedProbing(ms.GetMsNumber(), g30SValue, reply));
				}
			}
			else
			{
				// Do a Z probe at the specified point.
				newState = GCodeState::probingAtPoint0;
			}
		}
	}
	else
	{
		// G30 without P parameter. This probes the current location starting from the current position.
		// If S=-1 it just reports the stopped height, else it resets the Z origin.
		newState = GCodeState::probingAtPoint2a;
	}

	// If we get here then we actually need to probe
	const auto zp = SetZProbeNumber(gb, 'K');						// may throw, so do this before changing the state
	InitialiseTaps(zp->FastThenSlowProbing());

#if SUPPORT_ASYNC_MOVES
	// We allocate the axes we are going to move before doing anything else so that we can abort cleanly if they are in use by another motion system.
	// However, this assumes that deploying the probe can't release axes. See issue 978.
	axesMoving.SetBit(Z_AXIS);
	AllocateAxes(gb, ms, axesMoving, ParameterLettersBitmap());		// allocate any axes we are moving but don't bother to record the letters
#endif

	gb.SetState(newState);
	if (zp->GetProbeType() != ZProbeType::blTouch)
	{
		DeployZProbe(gb);
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
		mt = (AuxGCode()->GetLastStatusReportType() != StatusReportType::none) ? AuxMessage : HttpMessage;
	}
	return mt;
}

void GCodes::DoManualProbe(GCodeBuffer& gb, const char *message, const char *title, const AxesBitmap axes)
{
	if (Push(gb, true))													// stack the machine state including the file position and set the state to GCodeState::normal
	{
		const MessageType mt = GetMessageBoxDevice(gb);
		const uint32_t seq = reprap.SendAlert(mt, message, title, 2, 0.0, axes);
		gb.WaitForAcknowledgement(seq);									// flag that we are waiting for acknowledgement
	}
}

// Do a manual bed probe. On entry the state variable is the state we want to return to when the user has finished adjusting the height.
void GCodes::DoManualBedProbe(GCodeBuffer& gb)
{
	DoManualProbe(gb, "Adjust height until the nozzle just touches the bed, then press OK", "Manual bed probing", AxesBitmap::MakeFromBits(Z_AXIS));
}

// Set up to do the first of a possibly multi-tap probe
void GCodes::InitialiseTaps(bool fastThenSlow) noexcept
{
	tapsDone = (fastThenSlow) ? -1 : 0;
	g30zHeightErrorSum = 0.0;
	g30zHeightErrorLowestDiff = 1000.0;
}

#if SUPPORT_SCANNING_PROBES

// Take and store a reading from a scanning Z probe. Called by the Laser task.
void GCodes::TakeScanningProbeReading() noexcept
{
	const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
	float heightError;
	const GCodeResult rslt = zp->GetCalibratedReading(heightError);

	// Scanning Z probes can return bad readings if the probe reports an error. Don't store them in the height map, they mess up XY movement.
	if (rslt != GCodeResult::ok)
	{
		if (scanningResult != GCodeResult::ok)
		{
			scanningResult = rslt;
		}
	}
	else
	{
		reprap.GetMove().AccessHeightMap().SetGridHeight(gridAxis0Index, gridAxis1Index, -heightError);
	}

	if (gridAxis0Index != lastAxis0Index)
	{
		if (gridAxis1Index & 1u)
		{
			--gridAxis0Index;
		}
		else
		{
			++gridAxis0Index;
		}
	}
}

#endif

// Define the probing grid, called when we see an M557 command
GCodeResult GCodes::DefineGrid(GCodeBuffer& gb, const StringRef &reply) THROWS(GCodeException)
{
	if (!LockMovement(gb))							// to ensure that probing is not already in progress
	{
		return GCodeResult::notFinished;
	}

	bool seenR = false, seenP = false, seenS = false;
	char axesLetters[2] = { 'X', 'Y'};
	float axis0Values[2];
	float axis1Values[2];
	float spacings[2] = { DefaultGridSpacing, DefaultGridSpacing };

	size_t axesSeenCount = 0;
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (axisLetters[axis] == 'Z')
			{
				reply.copy("Z axis is not allowed for mesh leveling");
				return GCodeResult::error;
			}
			else if (axesSeenCount > 2)
			{
				reply.copy("Mesh leveling expects exactly two axes");
				return GCodeResult::error;
			}
			bool dummy;
			gb.TryGetFloatArray(axisLetters[axis], 2, (axesSeenCount == 0) ? axis0Values : axis1Values, dummy, false);
			axesLetters[axesSeenCount] = axisLetters[axis];
			++axesSeenCount;
		}
	}
	if (axesSeenCount == 1)
	{
		reply.copy("Specify zero or two axes in M557");
		return GCodeResult::error;
	}
	const bool axesSeen = axesSeenCount > 0;

	uint32_t numPoints[2];
	gb.TryGetUIArray('P', 2, numPoints, seenP, true);
	if (!seenP)
	{
		gb.TryGetFloatArray('S', 2, spacings, seenS, true);
	}

	float radius = -1.0;
	gb.TryGetFValue('R', radius, seenR);

	if (!axesSeen && !seenR && !seenS && !seenP)
	{
		// Just print the existing grid parameters
		if (defaultGrid.IsValid())
		{
			reply.copy("Grid: ");
			defaultGrid.PrintParameters(reply);
		}
		else
		{
			reply.copy("Grid is not defined");
		}
		return GCodeResult::ok;
	}

	if (!axesSeen && !seenR)
	{
		// Must have given just the S or P parameter
		reply.copy("specify at least radius or two axis ranges in M557");
		return GCodeResult::error;
	}

	if (axesSeen)
	{
		// Seen both axes
		if (seenP)
		{
			// In the following, we multiply the spacing by 0.9999 to ensure that when we divide the axis range by the spacing, we get the correct number of points
			// Otherwise, for some values we occasionally get one less point
			if (spacings[0] >= 2 && axis0Values[1] > axis0Values[0])
			{
				spacings[0] = (axis0Values[1] - axis0Values[0])/(numPoints[0] - 1) * 0.9999;
			}
			if (spacings[1] >= 2 && axis1Values[1] > axis1Values[0])
			{
				spacings[1] = (axis1Values[1] - axis1Values[0])/(numPoints[1] - 1) * 0.9999;
			}
		}
	}
	else
	{
		// Seen R
		if (radius > 0.0)
		{
			float effectiveXRadius;
			if (seenP && numPoints[0] >= 2)
			{
				effectiveXRadius = radius - 0.1;
				if (numPoints[1] % 2 == 0)
				{
					effectiveXRadius *= fastSqrtf(1.0 - 1.0/(float)((numPoints[1] - 1) * (numPoints[1] - 1)));
				}
				spacings[0] = (2 * effectiveXRadius)/(numPoints[0] - 1);
			}
			else
			{
				effectiveXRadius = floorf((radius - 0.1)/spacings[0]) * spacings[0];
			}
			axis0Values[0] = -effectiveXRadius;
			axis0Values[1] =  effectiveXRadius + 0.1;

			float effectiveYRadius;
			if (seenP && numPoints[1] >= 2)
			{
				effectiveYRadius = radius - 0.1;
				if (numPoints[0] % 2 == 0)
				{
					effectiveYRadius *= fastSqrtf(1.0 - 1.0/(float)((numPoints[0] - 1) * (numPoints[0] - 1)));
				}
				spacings[1] = (2 * effectiveYRadius)/(numPoints[1] - 1);
			}
			else
			{
				effectiveYRadius = floorf((radius - 0.1)/spacings[1]) * spacings[1];
			}
			axis1Values[0] = -effectiveYRadius;
			axis1Values[1] =  effectiveYRadius + 0.1;
		}
		else
		{
			reply.copy("M577 radius must be positive unless X and Y are specified");
			return GCodeResult::error;
		}
	}

	const bool ok = defaultGrid.Set(axesLetters, axis0Values, axis1Values, radius, spacings);
	reprap.MoveUpdated();
	if (ok)
	{
		return GCodeResult::ok;
	}

	const float axis1Range = axesSeen ? axis0Values[1] - axis0Values[0] : 2 * radius;
	const float axis2Range = axesSeen ? axis1Values[1] - axis1Values[0] : 2 * radius;
	reply.copy("bad grid definition: ");
	defaultGrid.PrintError(axis1Range, axis2Range, reply);
	return GCodeResult::error;
}

// Start probing the grid, returning true if we didn't because of an error.
// Prior to calling this the movement system must be locked.
GCodeResult GCodes::ProbeGrid(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
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

#if SUPPORT_ASYNC_MOVES
	// We allocate the axes we are going to move before doing anything else so that we can abort cleanly if they are in use by another motion system.
	// However, this assumes that deploying the probe can't release axes. See issue 978.
	constexpr AxesBitmap XyzAxes = AxesBitmap::MakeFromBits(X_AXIS, Y_AXIS) | AxesBitmap::MakeFromBits(Z_AXIS);
	AllocateAxes(gb, GetMovementState(gb), XyzAxes, ParameterLetterToBitmap('Z'));		// don't cache axis letters X and Y because they may be mapped
#endif

#if SUPPORT_PROBE_POINTS_FILE
	if (!probePointsFileLoaded)							// if we are using a probe points file then the grid has already been loaded
#endif
	{
		reprap.GetMove().AccessHeightMap().SetGrid(defaultGrid);
	}

	ClearBedMapping();
	gridAxis0Index = gridAxis1Index = 0;
	scanningResult = GCodeResult::ok;

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
			(void)MassStorage::Delete(fullName.GetRef(), ErrorMessageMode::messageAlways);
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
GCodeResult GCodes::SaveHeightMap(GCodeBuffer& gb, const StringRef& reply) const THROWS(GCodeException)
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
	ClearProbePointsInvalid();

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
	if (err)
	{
		return GCodeResult::error;
	}

	reply.Clear();						// get rid of the error message
	probePointsFileLoaded = true;		// use the grid we loaded when we probe
	return GCodeResult::ok;
}

void GCodes::ClearProbePointsInvalid() noexcept
{
	reprap.GetMove().ClearProbePointsInvalid();
	probePointsFileLoaded = false;		// use the default grid when we next probe
}

#endif	// SUPPORT_PROBE_POINTS_FILE

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Stop using bed compensation
void GCodes::ClearBedMapping() noexcept
{
	reprap.GetMove().SetIdentityTransform();
	for (MovementState& ms : moveStates)
	{
		reprap.GetMove().GetCurrentUserPosition(ms.coords, ms.GetMsNumber(), 0, ms.currentTool);
		ToolOffsetInverseTransform(ms);		// update user coordinates to remove any height map offset there was at the current position
	}
}

// Handle G38.[2-5]
GCodeResult GCodes::StraightProbe(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	const int8_t fraction = gb.GetCommandFraction();
	if (fraction < 2 || fraction > 5)
	{
		return GCodeResult::warningNotSupported;
	}
	/*
	 * It is an error if:
	 * # the current point is the same as the programmed point.
	 * # no axis word is used
	 * # the feed rate is zero
	 * # the probe is already in the target state
	 */

	straightProbeSettings.Reset();

	switch (fraction)
	{
	case 2:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::towardsWorkpieceErrorOnFailure);
		break;

	case 3:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::towardsWorkpiece);
		break;

	case 4:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::awayFromWorkpieceErrorOnFailure);
		break;

	case 5:
		straightProbeSettings.SetStraightProbeType(StraightProbeType::awayFromWorkpiece);
		break;
	}

	// Get the target coordinates (as user position) and check if we would move at all
	float userPositionTarget[MaxAxes];
	MovementState& ms = GetMovementState(gb);
	memcpyf(userPositionTarget, ms.currentUserPosition, numVisibleAxes);

	bool seen = false;
	bool doesMove = false;

#if SUPPORT_ASYNC_MOVES
	ParameterLettersBitmap axisLettersSeen;
#endif

	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		const char c = axisLetters[axis];
		if (gb.Seen(c))
		{
			seen = true;
#if SUPPORT_ASYNC_MOVES
			axisLettersSeen.SetBit(ParameterLetterToBitNumber(c));
#endif
			// Get the user provided target coordinate
			// - If prefixed by G53 add the ToolOffset that will be subtracted below in ToolOffsetTransform as we ignore any offsets when G53 is active
			// - otherwise add current workplace offsets so we go where the user expects to go
			// comparable to how DoStraightMove/DoArcMove does it
			const float axisTarget = gb.GetDistance() + (gb.LatestMachineState().g53Active ? ms.GetCurrentToolOffset(axis) : GetWorkplaceOffset(gb, axis));
			if (axisTarget != userPositionTarget[axis])
			{
				doesMove = true;
			}
			userPositionTarget[axis] = axisTarget;
			straightProbeSettings.AddMovingAxis(axis);
		}
	}

	// No axis letters seen
	if (!seen)
	{
		// Signal error for G38.2 and G38.4
		if (straightProbeSettings.SignalError())
		{
			reply.copy("No axis specified.");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}

	// At least one axis seen but it would not result in movement
	else if (!doesMove)
	{
		// Signal error for G38.2 and G38.4
		if (straightProbeSettings.SignalError())
		{
			reply.copy("Target equals current position.");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}

#if SUPPORT_ASYNC_MOVES
	AllocateAxes(gb, ms, straightProbeSettings.GetMovingAxes(), axisLettersSeen);
#endif

	// Convert target user position to machine coordinates and save them in StraightProbeSettings
	ToolOffsetTransform(ms, userPositionTarget, straightProbeSettings.GetTarget());

	// See whether we are using a user-defined Z probe or just current one
	const size_t probeToUse = (gb.Seen('K') || gb.Seen('P')) ? gb.GetUIValue() : 0;

	// Check if this probe exists to not run into a nullptr dereference later
	if (platform.GetEndstops().GetZProbe(probeToUse).IsNull())
	{
		reply.catf("Invalid probe number: %d", probeToUse);
		return GCodeResult::error;
	}
	straightProbeSettings.SetZProbeToUse(probeToUse);

	gb.SetState(GCodeState::straightProbe0);
	return GCodeResult::ok;
}

// Search for and return an axis, throw if none found or that axis hasn't been homed. On return we can fetch the parameter value after the axis letter.
size_t GCodes::FindAxisLetter(GCodeBuffer& gb) THROWS(GCodeException)
{
	for (size_t axis = 0; axis < numVisibleAxes; axis++)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			if (IsAxisHomed(axis))
			{
				return axis;
			}
			throw GCodeException(&gb, -1, "%c axis has not been homed", (uint32_t)axisLetters[axis]);
		}
	}

	throw GCodeException(&gb, -1, "No axis specified");
}

// Deal with a M585
GCodeResult GCodes::ProbeTool(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);
	if (ms.currentTool == nullptr)
	{
		reply.copy("No tool selected!");
		return GCodeResult::error;
	}

	if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

	// Get the feed rate and axis
	gb.MustSee(feedrateLetter);
	m585Settings.feedRate = gb.LatestMachineState().feedRate = gb.GetSpeed();		// don't apply the speed factor to homing and other special moves
	m585Settings.axisNumber = FindAxisLetter(gb);
	m585Settings.offset = gb.GetDistance();

#if SUPPORT_ASYNC_MOVES
	AllocateAxes(gb, ms, AxesBitmap::MakeFromBits(m585Settings.axisNumber), ParameterLettersBitmap());
#endif

	// See whether we are using a Z probe or just endstops
	if (gb.Seen('K'))
	{
		(void)SetZProbeNumber(gb, 'K');						// throws if the probe doesn't exist
		m585Settings.useProbe = true;
	}
	else if (gb.Seen('P'))
	{
		(void)SetZProbeNumber(gb, 'P');						// throws if the probe doesn't exist
		m585Settings.useProbe = true;
	}
	else
	{
		m585Settings.useProbe = false;
	}

	// Decide which way and how far to go
	ToolOffsetTransform(ms);
	m585Settings.probingLimit = (gb.Seen('R')) ? ms.coords[m585Settings.axisNumber] + gb.GetDistance()
								: (gb.Seen('S') && gb.GetIValue() > 0) ? platform.AxisMinimum(m585Settings.axisNumber)
									: platform.AxisMaximum(m585Settings.axisNumber);
	if (m585Settings.useProbe)
	{
		gb.SetState(GCodeState::probingToolOffset1);
		DeployZProbe(gb);
	}
	else
	{
		gb.SetState(GCodeState::probingToolOffset3);		// skip the Z probe stuff
	}

	return GCodeResult::ok;
}

// Set up a probing move for M675. If using a Z probe, it has already been deployed
// Return true if successful, else SetError has been called to save the error message
bool GCodes::SetupM585ProbingMove(GCodeBuffer& gb) noexcept
{
	bool reduceAcceleration;
	if (m585Settings.useProbe)
	{
		const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
		if (zp->Stopped())
		{
			gb.LatestMachineState().SetError("Probe already triggered before probing move started");
			return false;
		}
		if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
		{
			gb.LatestMachineState().SetError("Failed to enable probe");
			return false;
		}
		reduceAcceleration = true;
	}
	else if (!platform.GetEndstops().EnableAxisEndstops(AxesBitmap::MakeFromBits(m585Settings.axisNumber), false, reduceAcceleration))
	{
		gb.LatestMachineState().SetError("Failed to enable endstop");
		return false;
	}

	MovementState& ms = GetMovementState(gb);
	SetMoveBufferDefaults(ms);
	ToolOffsetTransform(ms);
	ms.feedRate = m585Settings.feedRate;
	ms.coords[m585Settings.axisNumber] = m585Settings.probingLimit;
	ms.reduceAcceleration = reduceAcceleration;
	ms.checkEndstops = true;
	ms.canPauseAfter = false;
	zProbeTriggered = false;
	ms.linearAxesMentioned = reprap.GetPlatform().IsAxisLinear(m585Settings.axisNumber);
	ms.rotationalAxesMentioned = reprap.GetPlatform().IsAxisRotational(m585Settings.axisNumber);
	NewSingleSegmentMoveAvailable(ms);
	return true;
}

GCodeResult GCodes::FindCenterOfCavity(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	MovementState& ms = GetMovementState(gb);
	if (ms.currentTool == nullptr)
	{
		reply.copy("No tool selected!");
		return GCodeResult::error;
	}

	if (!LockCurrentMovementSystemAndWaitForStandstill(gb))
	{
		return GCodeResult::notFinished;
	}

	// Get the feed rate, backoff distance, and axis
	gb.MustSee(feedrateLetter);
	m675Settings.feedRate = gb.LatestMachineState().feedRate = gb.GetSpeed();		// don't apply the speed factor to homing and other special moves
	m675Settings.backoffDistance = gb.Seen('R') ? gb.GetDistance() : 5.0;
	m675Settings.axisNumber = FindAxisLetter(gb);

#if SUPPORT_ASYNC_MOVES
	AllocateAxes(gb, ms, AxesBitmap::MakeFromBits(m585Settings.axisNumber), ParameterLettersBitmap());
#endif

	// Get the probe number from the K or P parameter
	const char probeLetter = gb.MustSee('K', 'P');				// throws if neither character is found
	(void)SetZProbeNumber(gb, probeLetter);						// throws if the probe doesn't exist
	gb.SetState(GCodeState::findCenterOfCavity1);
	DeployZProbe(gb);

	return GCodeResult::ok;
}

// Set up a probing move for M675. If using a Z probe, it has already been deployed
// Return true if successful, else SetError has been called to save the error message
bool GCodes::SetupM675ProbingMove(GCodeBuffer& gb, bool towardsMin) noexcept
{
	const auto zp = platform.GetZProbeOrDefault(currentZProbeNumber);
	if (zp->Stopped())
	{
		gb.LatestMachineState().SetError("Probe already triggered before probing move started");
		return false;
	}
	if (!platform.GetEndstops().EnableZProbe(currentZProbeNumber) || !zp->SetProbing(true))
	{
		gb.LatestMachineState().SetError("Failed to enable probe");
		return false;
	}

	MovementState& ms = GetMovementState(gb);
	SetMoveBufferDefaults(ms);
	ToolOffsetTransform(ms);
	ms.coords[m675Settings.axisNumber] = towardsMin ? platform.AxisMinimum(m675Settings.axisNumber) : platform.AxisMaximum(m675Settings.axisNumber);
	ms.feedRate = m675Settings.feedRate;
	ms.checkEndstops = true;
	ms.canPauseAfter = false;
	zProbeTriggered = false;
	ms.linearAxesMentioned = reprap.GetPlatform().IsAxisLinear(m675Settings.axisNumber);
	ms.rotationalAxesMentioned = reprap.GetPlatform().IsAxisRotational(m675Settings.axisNumber);
	NewSingleSegmentMoveAvailable(ms);						// kick off the move
	return true;
}

void GCodes::SetupM675BackoffMove(GCodeBuffer& gb, float position) noexcept
{
	MovementState& ms = GetMovementState(gb);
	SetMoveBufferDefaults(ms);
	ToolOffsetTransform(ms);
	ms.coords[m675Settings.axisNumber] = position;
	ms.feedRate = m675Settings.feedRate;
	ms.canPauseAfter = false;
	ms.linearAxesMentioned = reprap.GetPlatform().IsAxisLinear(m675Settings.axisNumber);
	ms.rotationalAxesMentioned = reprap.GetPlatform().IsAxisRotational(m675Settings.axisNumber);
	NewSingleSegmentMoveAvailable(ms);
}

#if SUPPORT_SCANNING_PROBES

// Calibrate height vs. reading for a scanning Z probe. We have already checked the probe is a scanning one and that scanningRange is a sensible value.
GCodeResult GCodes::HandleM558Point1or2(GCodeBuffer& gb, const StringRef &reply, unsigned int probeNumber) THROWS(GCodeException)
{
	const auto zp = platform.GetEndstops().GetZProbe(probeNumber);
	if (zp.IsNull())
	{
		reply.copy("invalid Z probe index");
		return GCodeResult::error;
	}

	if (!zp->IsScanning())
	{
		reply.printf("probe %u is not a scanning probe", probeNumber);
		return GCodeResult::error;
	}

	if (gb.GetCommandFraction() == 1)
	{
		// Calibrate height vs. reading
		if (gb.Seen('A'))
		{
			const float aParam = gb.GetFValue();
			const float bParam = (gb.Seen('B')) ? gb.GetFValue() : 0.0;
			const float cParam = (gb.Seen('C')) ? gb.GetFValue() : 0.0;
			return zp->SetScanningCoefficients(aParam, bParam, cParam);
		}

		if (gb.Seen('S'))
		{
			const float requestedScanningRange = gb.GetLimitedFValue('S', 0.1, zp->GetConfiguredTriggerHeight());

#if SUPPORT_ASYNC_MOVES
			AxesBitmap axesMoving;
			axesMoving.SetBit(Z_AXIS);
			MovementState& ms = GetMovementState(gb);
			AllocateAxes(gb, ms, axesMoving, ParameterLettersBitmap());		// allocate the Z axis
#endif
			currentZProbeNumber = probeNumber;
			zp->PrepareForUse(false);										// needed to set actual trigger height allowing for temperature compensation

			// Set the scanning range to a whole number of microsteps and calculate the microsteps per point
			const unsigned int microstepsPerHalfScan = (unsigned int)(requestedScanningRange * platform.DriveStepsPerUnit(Z_AXIS));
			constexpr unsigned int MaxCalibrationPointsPerHalfScan = (MaxScanningProbeCalibrationPoints - 1)/2;
			const unsigned int microstepsPerPoint = max<unsigned int>((microstepsPerHalfScan + MaxCalibrationPointsPerHalfScan - 1)/MaxCalibrationPointsPerHalfScan, 1);
			heightChangePerPoint = microstepsPerPoint/platform.DriveStepsPerUnit(Z_AXIS);
			const size_t pointsPerHalfScan = microstepsPerHalfScan/microstepsPerPoint;
			calibrationStartingHeight = zp->GetActualTriggerHeight() + pointsPerHalfScan * heightChangePerPoint;
			numPointsToCollect = 2 * pointsPerHalfScan + 1;
			RRF_ASSERT(numPointsToCollect <= MaxScanningProbeCalibrationPoints);

			// Deploy the probe and start the state machine
			gb.SetState(GCodeState::probeCalibration1);
			DeployZProbe(gb);
			return GCodeResult::ok;
		}
		return zp->ReportScanningCoefficients(reply);
	}

	// Else must be M558.2: Calibrate drive level
	return zp->CalibrateDriveLevel(gb, reply);
}

#endif

// Decode whether we have done enough taps and will accept the reading. Sets member variable acceptReading accordingly.
// Returns true if we need to give a "Z probe readings not consistent" warning.
void GCodes::CheckIfMoreTapsNeeded(GCodeBuffer& gb, const ZProbe& zp) noexcept
{
	if (zp.GetMaxTaps() < 2 && tapsDone == 1)
	{
		acceptReading = true;
		return;
	}

	if (tapsDone >= 2)
	{
		g30zHeightErrorLowestDiff = min<float>(g30zHeightErrorLowestDiff, fabsf(g30zHeightError - g30PrevHeightError));
		if (zp.GetTolerance() > 0.0 && g30zHeightErrorLowestDiff <= zp.GetTolerance())
		{
			g30zHeightError = (g30zHeightError + g30PrevHeightError)/2;
			acceptReading = true;
			return;
		}
	}

	if (tapsDone == (int)zp.GetMaxTaps())
	{
		// We no longer flag this as a probing error, instead we take the average and issue a warning
		acceptReading = true;
		g30zHeightError = g30zHeightErrorSum/tapsDone;
		if (zp.GetTolerance() > 0.0)				// zero or negative tolerance means always average all readings, so no warning message
		{
			gb.LatestMachineState().SetError("Z probe readings not consistent");
		}
	}
	else
	{
		acceptReading = false;							// more taps needed
	}
}

// End
