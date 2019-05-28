/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David

 A note on bed levelling:

 As at version 1.21 we support two types of bed compensation:
 1. The old 3, 4 and 5-point compensation using a RandomProbePointSet. We will probably discontinue this soon.
 2. Mesh bed levelling

 There is an interaction between using G30 to home Z or set a precise Z=0 height just before a print, and bed compensation.
 Consider the following sequence:
 1. Home Z, using either G30 or an endstop.
 2. Run G29 to generate a height map. If the Z=0 point has drifted off, the height map may have a Z offset.
 3. Use G30 to get an accurate Z=0 point. We want to keep the shape of the height map, but get rid of the offset.
 4. Run G29 to generate a height map. This should generate a height map with on offset at the point we just probed.
 5. Cancel bed compensation. The height at the point we just probed should be zero.

 So as well as maintaining a height map, we maintain a Z offset from it. The procedure is:
 1. Whenever bed compensation is not being used, the Z offset should be zero.
 2. Whenever we run G29 to probe the bed, we have a choice:
 (a) accept that the map may have a height offset; and set the Z offset to zero. This is what we do currently.
 (b) normalise the height map to zero, adjust the Z=0 origin, and set the Z offset to zero.
 3. When we run G30 to reset the Z=0 height, and we have a height map loaded, we adjust the Z offset to be the negative of the
    height map indication of that point.
 4. If we now cancel the height map, we also clear the Z offset, and the height at the point we probed remains correct.
 5. If we now run G29 to probe again, the height map should have near zero offset at the point we probed, if there has been no drift.

 Before we introduced the Z offset, at step 4 we would have a potentially large Z error as if the G30 hadn't been run,
 and at step 5 the new height map would have an offset again.

 */

#include "Move.h"
#include "StepTimer.h"
#include "Platform.h"
#include "GCodes/GCodeBuffer.h"
#include "Tools/Tool.h"

#if SUPPORT_CAN_EXPANSION
# include "CAN/CanInterface.h"
#endif

#if SUPPORT_OBJECT_MODEL

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(_ret) OBJECT_MODEL_FUNC_BODY(Move, _ret)

const ObjectModelTableEntry Move::objectModelTable[] =
{
	// These entries must be in alphabetical order
	{ "drcEnabled", OBJECT_MODEL_FUNC(&(self->drcEnabled)), TYPE_OF(bool), ObjectModelTableEntry::none },
	{ "drcMinimumAcceleration", OBJECT_MODEL_FUNC(&(self->drcMinimumAcceleration)), TYPE_OF(float), ObjectModelTableEntry::none },
	{ "drcPeriod", OBJECT_MODEL_FUNC(&(self->drcPeriod)), TYPE_OF(float), ObjectModelTableEntry::none },
	{ "maxPrintingAcceleration", OBJECT_MODEL_FUNC(&(self->maxPrintingAcceleration)), TYPE_OF(float), ObjectModelTableEntry::none },
	{ "maxTravelAcceleration", OBJECT_MODEL_FUNC(&(self->maxTravelAcceleration)), TYPE_OF(float), ObjectModelTableEntry::none },
};

DEFINE_GET_OBJECT_MODEL_TABLE(Move)

#endif

Move::Move() : active(false)
{
	// Kinematics must be set up here because GCodes::Init asks the kinematics for the assumed initial position
	kinematics = Kinematics::Create(KinematicsType::cartesian);			// default to Cartesian
	mainDDARing.Init1(DdaRingLength);
	DriveMovement::InitialAllocate(NumDms);
}

void Move::Init()
{
	mainDDARing.Init2();

	maxPrintingAcceleration = maxTravelAcceleration = 10000.0;
	drcEnabled = false;											// disable dynamic ringing cancellation
	drcMinimumAcceleration = 10.0;
	drcPeriod = 50.0;

	// Clear the transforms
	SetIdentityTransform();
	tanXY = tanYZ = tanXZ = 0.0;

	usingMesh = false;
	useTaper = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	lastStateChangeTime = millis();
	idleCount = 0;

	simulationMode = 0;
	longestGcodeWaitInterval = 0;
	numHiccups = 0;
	bedLevellingMoveAvailable = false;

	active = true;
}

void Move::Exit()
{
	StepTimer::DisableStepInterrupt();
	mainDDARing.Exit();
	active = false;												// don't accept any more moves
}

void Move::Spin()
{
	if (!active)
	{
		GCodes::RawMove nextMove;
		(void) reprap.GetGCodes().ReadMove(nextMove);			// throw away any move that GCodes tries to pass us
		return;
	}

	if (idleCount < 1000)
	{
		++idleCount;
	}

	// Recycle the DDAs for completed moves, checking for DDA errors to print if Move debug is enabled
	mainDDARing.RecycleDDAs();

	// See if we can add another move to the ring
	bool canAddMove = (
#if SUPPORT_ROLAND
						  !reprap.GetRoland()->Active() &&
#endif
						  mainDDARing.CanAddMove()
					  );
	if (canAddMove)
	{
		// OK to add another move. First check if a special move is available.
		if (bedLevellingMoveAvailable)
		{
			if (simulationMode < 2)
			{
				if (mainDDARing.AddSpecialMove(reprap.GetPlatform().MaxFeedrate(Z_AXIS), specialMoveCoords))
				{
					if (moveState == MoveState::idle || moveState == MoveState::timing)
					{
						// We were previously idle, so we have a state change
						moveState = MoveState::collecting;
						const uint32_t now = millis();
						const uint32_t timeWaiting = now - lastStateChangeTime;
						if (timeWaiting > longestGcodeWaitInterval)
						{
							longestGcodeWaitInterval = timeWaiting;
						}
						lastStateChangeTime = now;
					}
				}
			}
			bedLevellingMoveAvailable = false;
		}
		else
		{
			// If there's a G Code move available, add it to the DDA ring for processing.
			GCodes::RawMove nextMove;
			if (reprap.GetGCodes().ReadMove(nextMove))		// if we have a new move
			{
				if (simulationMode < 2)		// in simulation mode 2 and higher, we don't process incoming moves beyond this point
				{
					if (nextMove.moveType == 0)
					{
						AxisAndBedTransform(nextMove.coords, nextMove.xAxes, nextMove.yAxes, true);
					}

					if (mainDDARing.AddStandardMove(nextMove, !IsRawMotorMove(nextMove.moveType)))
					{
						idleCount = 0;
						if (moveState == MoveState::idle || moveState == MoveState::timing)
						{
							moveState = MoveState::collecting;
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - lastStateChangeTime;
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							lastStateChangeTime = now;
						}
					}
				}
			}
		}
	}

	mainDDARing.Spin(simulationMode, idleCount > 10);	// let the DDA ring process moves. Better to have a few moves in the queue so that we can do lookahead, hence the test on idleCount.

	// Reduce motor current to standby if the rings have been idle for long enough
	if (mainDDARing.IsIdle())
	{
		if (moveState == MoveState::executing && !reprap.GetGCodes().IsPaused())
		{
			lastStateChangeTime = millis();				// record when we first noticed that the machine was idle
			moveState = MoveState::timing;
		}
		else if (moveState == MoveState::timing && millis() - lastStateChangeTime >= idleTimeout)
		{
			reprap.GetPlatform().SetDriversIdle();		// put all drives in idle hold
			moveState = MoveState::idle;
		}
	}
	else
	{
		moveState = MoveState::executing;
	}
}

// Return the number of currently used probe points
unsigned int Move::GetNumProbePoints() const
{
	return probePoints.GetNumBedCompensationPoints();
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
float Move::PushBabyStepping(size_t axis, float amount)
{
	return mainDDARing.PushBabyStepping(axis, amount);
}

// Change the kinematics to the specified type if it isn't already
// If it is already correct leave its parameters alone.
// This violates our rule on no dynamic memory allocation after the initialisation phase,
// however this function is normally called only when M665, M667 and M669 commands in config.g are processed.
bool Move::SetKinematics(KinematicsType k)
{
	if (kinematics->GetKinematicsType() != k)
	{
		Kinematics *nk = Kinematics::Create(k);
		if (nk == nullptr)
		{
			return false;
		}
		delete kinematics;
		kinematics = nk;
	}
	return true;
}

// Return true if this is a raw motor move
bool Move::IsRawMotorMove(uint8_t moveType) const
{
	return moveType == 2 || ((moveType == 1 || moveType == 3) && kinematics->GetHomingMode() != HomingMode::homeCartesianAxes);
}

// Return true if the specified point is accessible to the Z probe
bool Move::IsAccessibleProbePoint(float x, float y) const
{
	const ZProbe& params = reprap.GetPlatform().GetCurrentZProbeParameters();
	return kinematics->IsReachable(x - params.xOffset, y - params.yOffset, false);
}

// Pause the print as soon as we can, returning true if we are able to skip any moves and updating 'rp' to the first move we skipped.
bool Move::PausePrint(RestorePoint& rp)
{
	return mainDDARing.PauseMoves(rp);
}

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Pause the print immediately, returning true if we were able to skip or abort any moves and setting up to the move we aborted
bool Move::LowPowerOrStallPause(RestorePoint& rp)
{
	return mainDDARing.LowPowerOrStallPause(rp);
}

#endif

void Move::Diagnostics(MessageType mtype)
{
	Platform& p = reprap.GetPlatform();
	p.MessageF(mtype, "=== Move ===\nHiccups: %" PRIu32 ", FreeDm: %d, MinFreeDm: %d, MaxWait: %" PRIu32 "ms\n",
						numHiccups, DriveMovement::NumFree(), DriveMovement::MinFree(), longestGcodeWaitInterval);
	numHiccups = 0;
	longestGcodeWaitInterval = 0;
	DriveMovement::ResetMinFree();

#if defined(__ALLIGATOR__)
	// Motor Fault Diagnostic
	reprap.GetPlatform().MessageF(mtype, "Motor Fault status: %s\n", digitalRead(MotorFaultDetectPin) ? "none" : "FAULT detected!" );
#endif

	// Show the current probe position heights and type of bed compensation in use
	String<StringLength40> bedCompString;
	if (usingMesh)
	{
		bedCompString.copy("mesh");
	}
	else if (probePoints.GetNumBedCompensationPoints() != 0)
	{
		bedCompString.printf("%d point", probePoints.GetNumBedCompensationPoints());
	}
	else
	{
		bedCompString.copy("none");
	}
	p.MessageF(mtype, "Bed compensation in use: %s, comp offset %.3f\n", bedCompString.c_str(), (double)zShift);

	// Only print the probe point heights if we are using old-style compensation
	if (!usingMesh && probePoints.GetNumBedCompensationPoints() != 0)
	{
		// To keep the response short so that it doesn't get truncated when sending it via HTTP, we only show the first 5 bed probe points
		bedCompString.Clear();
		for (size_t i = 0; i < 5; ++i)
		{
			bedCompString.catf(" %.3f", (double)probePoints.GetZHeight(i));
		}
		p.MessageF(mtype, "Bed probe heights:%s\n", bedCompString.c_str());
	}

#if DDA_LOG_PROBE_CHANGES
	// Temporary code to print Z probe trigger positions
	p.Message(mtype, "Probe change coordinates:");
	char ch = ' ';
	for (size_t i = 0; i < DDA::numLoggedProbePositions; ++i)
	{
		float xyzPos[XYZ_AXES];
		MotorStepsToCartesian(DDA::loggedProbePositions + (XYZ_AXES * i), XYZ_AXES, XYZ_AXES, xyzPos);
		p.MessageF(mtype, "%c%.2f,%.2f", ch, xyzPos[X_AXIS], xyzPos[Y_AXIS]);
		ch = ',';
	}
	p.Message(mtype, "\n");
#endif

	mainDDARing.Diagnostics(mtype, "");
}

// Set the current position to be this
void Move::SetNewPosition(const float positionNow[MaxTotalDrivers], bool doBedCompensation)
{
	float newPos[MaxTotalDrivers];
	memcpy(newPos, positionNow, sizeof(newPos));			// copy to local storage because Transform modifies it
	AxisAndBedTransform(newPos, reprap.GetCurrentXAxes(), reprap.GetCurrentYAxes(), doBedCompensation);
	SetLiveCoordinates(newPos);
	SetPositions(newPos);
}

// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered and DDA::SetPositions
void Move::EndPointToMachine(const float coords[], int32_t ep[], size_t numDrives) const
{
	if (CartesianToMotorSteps(coords, ep, true))
	{
		const size_t numAxes = reprap.GetGCodes().GetTotalAxes();
		for (size_t drive = numAxes; drive < numDrives; ++drive)
		{
			ep[drive] = MotorMovementToSteps(drive, coords[drive]);
		}
	}
}

// Convert distance to steps for a particular drive
int32_t Move::MotorMovementToSteps(size_t drive, float coord)
{
	return lrintf(coord * reprap.GetPlatform().DriveStepsPerUnit(drive));
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// This is computationally expensive on a delta or SCARA machine, so only call it when necessary, and never from the step ISR.
void Move::MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const
{
	kinematics->MotorStepsToCartesian(motorPos, reprap.GetPlatform().GetDriveStepsPerUnit(), numVisibleAxes, numTotalAxes, machinePos);
	if (reprap.Debug(moduleMove) && !inInterrupt())
	{
		debugPrintf("Forward transformed %" PRIi32 " %" PRIi32 " %" PRIi32 " to %.2f %.2f %.2f\n",
			motorPos[0], motorPos[1], motorPos[2], (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
	}
}

// Convert Cartesian coordinates to motor steps, axes only, returning true if successful.
// Used to perform movement and G92 commands.
// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered, DDA::SetPositions and Move::EndPointToMachine
bool Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const
{
	const bool b = kinematics->CartesianToMotorSteps(machinePos, reprap.GetPlatform().GetDriveStepsPerUnit(),
														reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), motorPos, isCoordinated);
	if (reprap.Debug(moduleMove) && !inInterrupt())
	{
		if (!b)
		{
			debugPrintf("Unable to transform");
			for (size_t i = 0; i < reprap.GetGCodes().GetVisibleAxes(); ++i)
			{
				debugPrintf(" %.2f", (double)machinePos[i]);
			}
			debugPrintf("\n");
		}
		else if (reprap.Debug(moduleDda))
		{
			debugPrintf("Transformed");
			for (size_t i = 0; i < reprap.GetGCodes().GetVisibleAxes(); ++i)
			{
				debugPrintf(" %.2f", (double)machinePos[i]);
			}
			debugPrintf(" to");
			for (size_t i = 0; i < reprap.GetGCodes().GetTotalAxes(); ++i)
			{
				debugPrintf(" %" PRIi32, motorPos[i]);
			}
			debugPrintf("\n");
		}
	}
	return b;
}

void Move::AxisAndBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes, bool useBedCompensation) const
{
	AxisTransform(xyzPoint, xAxes, yAxes);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, xAxes, yAxes);
	}
}

void Move::InverseAxisAndBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	InverseBedTransform(xyzPoint, xAxes, yAxes);
	InverseAxisTransform(xyzPoint, xAxes, yAxes);
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	// Identify the lowest Y axis
	const size_t NumVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t yAxis = Y_AXIS; yAxis < NumVisibleAxes; ++yAxis)
	{
		if (IsBitSet(yAxes, yAxis))
		{
			// Found a Y axis. Use this one when correcting the X coordinate.
			for (size_t axis = 0; axis < NumVisibleAxes; ++axis)
			{
				if (IsBitSet(xAxes, axis))
				{
					xyzPoint[axis] += tanXY*xyzPoint[yAxis] + tanXZ*xyzPoint[Z_AXIS];
				}
				if (IsBitSet(yAxes, axis))
				{
					xyzPoint[axis] += tanYZ*xyzPoint[Z_AXIS];
				}
			}
			break;
		}
	}
}

// Get the height error at an XY position
float Move::GetInterpolatedHeightError(float xCoord, float yCoord) const
{
	return (usingMesh) ? heightMap.GetInterpolatedHeightError(xCoord, yCoord) : probePoints.GetInterpolatedHeightError(xCoord, yCoord);
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	// Identify the lowest Y axis
	const size_t NumVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	for (size_t yAxis = Y_AXIS; yAxis < NumVisibleAxes; ++yAxis)
	{
		if (IsBitSet(yAxes, yAxis))
		{
			// Found a Y axis. Use this one when correcting the X coordinate.
			for (size_t axis = 0; axis < NumVisibleAxes; ++axis)
			{
				if (IsBitSet(yAxes, axis))
				{
					xyzPoint[axis] -= tanYZ*xyzPoint[Z_AXIS];
				}
				if (IsBitSet(xAxes, axis))
				{
					xyzPoint[axis] -= (tanXY*xyzPoint[yAxis] + tanXZ*xyzPoint[Z_AXIS]);
				}
			}
			break;
		}
	}
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	if (!useTaper || xyzPoint[Z_AXIS] < taperHeight)
	{
		float zCorrection = 0.0;
		const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
		unsigned int numCorrections = 0;

		// Transform the Z coordinate based on the average correction for each axis used as an X axis.
		// We are assuming that the tool Y offsets are small enough to be ignored.
		for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
		{
			if (IsBitSet(xAxes, xAxis))
			{
				const float xCoord = xyzPoint[xAxis];
				for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
				{
					if (IsBitSet(yAxes, yAxis))
					{
						const float yCoord = xyzPoint[yAxis];
						zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
						++numCorrections;
					}
				}
			}
		}

		if (numCorrections > 1)
		{
			zCorrection /= numCorrections;			// take an average
		}

		zCorrection += zShift;
		xyzPoint[Z_AXIS] += (useTaper) ? (taperHeight - xyzPoint[Z_AXIS]) * recipTaperHeight * zCorrection : zCorrection;
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MaxAxes], AxesBitmap xAxes, AxesBitmap yAxes) const
{
	float zCorrection = 0.0;
	const size_t numAxes = reprap.GetGCodes().GetVisibleAxes();
	unsigned int numCorrections = 0;

	// Transform the Z coordinate based on the average correction for each axis used as an X axis.
	// We are assuming that the tool Y offsets are small enough to be ignored.
	for (uint32_t xAxis = 0; xAxis < numAxes; ++xAxis)
	{
		if (IsBitSet(xAxes, xAxis))
		{
			const float xCoord = xyzPoint[xAxis];
			for (uint32_t yAxis = 0; yAxis < numAxes; ++yAxis)
			{
				if (IsBitSet(yAxes, yAxis))
				{
					const float yCoord = xyzPoint[yAxis];
					zCorrection += GetInterpolatedHeightError(xCoord, yCoord);
					++numCorrections;
				}
			}
		}
	}

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;				// take an average
	}

	zCorrection += zShift;

	if (!useTaper || zCorrection >= taperHeight)	// need check on zCorrection to avoid possible divide by zero
	{
		xyzPoint[Z_AXIS] -= zCorrection;
	}
	else
	{
		const float zreq = (xyzPoint[Z_AXIS] - zCorrection)/(1.0 - (zCorrection * recipTaperHeight));
		if (zreq < taperHeight)
		{
			xyzPoint[Z_AXIS] = zreq;
		}
	}
}

// Normalise the bed transform to have zero height error at these coordinates
void Move::SetZeroHeightError(const float coords[MaxAxes])
{
	float tempCoords[MaxAxes];
	memcpy(tempCoords, coords, sizeof(tempCoords));
	AxisTransform(tempCoords, DefaultXAxisMapping, DefaultYAxisMapping);
	zShift = -GetInterpolatedHeightError(tempCoords[X_AXIS], tempCoords[Y_AXIS]);
}

void Move::SetIdentityTransform()
{
	probePoints.SetIdentity();
	heightMap.ClearGridHeights();
	heightMap.UseHeightMap(false);
	usingMesh = false;
	zShift = 0.0;
}

// Load the height map from file, returning true if an error occurred with the error reason appended to the buffer
bool Move::LoadHeightMapFromFile(FileStore *f, const StringRef& r)
{
	const bool ret = heightMap.LoadFromFile(f, r);
	if (ret)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
	}
	else
	{
		zShift = 0.0;
	}
	return ret;
}

// Save the height map to a file returning true if an error occurred
bool Move::SaveHeightMapToFile(FileStore *f) const
{
	return heightMap.SaveToFile(f, zShift);
}

void Move::SetTaperHeight(float h)
{
	useTaper = (h > 1.0);
	if (useTaper)
	{
		taperHeight = h;
		recipTaperHeight = 1.0/h;
	}
}

// Enable mesh bed compensation
bool Move::UseMesh(bool b)
{
	usingMesh = heightMap.UseHeightMap(b);
	return usingMesh;
}

float Move::AxisCompensation(unsigned int axis) const
{
	return (axis < ARRAY_SIZE(tangents)) ? tangents[axis] : 0.0;
}

void Move::SetAxisCompensation(unsigned int axis, float tangent)
{
	if (axis < ARRAY_SIZE(tangents))
	{
		tangents[axis] = tangent;
	}
}

// Calibrate or set the bed equation after probing, returning true if an error occurred
// sParam is the value of the S parameter in the G30 command that provoked this call.
// Caller already owns the GCode movement lock.
bool Move::FinishedBedProbing(int sParam, const StringRef& reply)
{
	bool error = false;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		probePoints.ReportProbeHeights(numPoints, reply);
	}
	else if (numPoints < (size_t)sParam)
	{
		reply.printf("Bed calibration : %d factor calibration requested but only %d points provided\n", sParam, numPoints);
		error = true;
	}
	else
	{
		if (reprap.Debug(moduleMove))
		{
			probePoints.DebugPrint(numPoints);
		}

		if (sParam == 0)
		{
			sParam = numPoints;
		}

		if (!probePoints.GoodProbePoints(numPoints))
		{
			reply.copy("Compensation or calibration cancelled due to probing errors");
			error = true;
		}
		else if (kinematics->SupportsAutoCalibration())
		{
			error = kinematics->DoAutoCalibration(sParam, probePoints, reply);
		}
		else
		{
			error = probePoints.SetProbedBedEquation(sParam, reply);
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	probePoints.ClearProbeHeights();
	return error;
}

// This is the function that is called by the timer interrupt to step the motors.
// This may occasionally get called prematurely.
void Move::Interrupt()
{
	const uint32_t isrStartTime = StepTimer::GetInterruptClocksInterruptsDisabled();
	Platform& p = reprap.GetPlatform();
	bool repeat;
	do
	{
		mainDDARing.Interrupt(p);
		std::optional<uint32_t> nextStepTime = mainDDARing.GetNextInterruptTime();
		if (!nextStepTime.has_value())
		{
			break;
		}

		// Check whether we have been in this ISR for too long already and need to take a break
		const uint16_t clocksTaken = StepTimer::GetInterruptClocks16() - (uint16_t)isrStartTime;
		if (clocksTaken >= DDA::MaxStepInterruptTime && (nextStepTime.value() - isrStartTime) < (clocksTaken + DDA::MinInterruptInterval))
		{
			// Force a break by updating the move start time
			mainDDARing.InsertHiccup(DDA::HiccupTime);
			nextStepTime = nextStepTime.value() + DDA::HiccupTime;
#if SUPPORT_CAN_EXPANSION
			CanInterface::InsertHiccup(DDA::HiccupTime);
#endif
			++numHiccups;
		}

		// 8. Schedule next interrupt, or if it would be too soon, generate more steps immediately
		// If we have already spent too much time in the ISR, delay the interrupt
		repeat = StepTimer::ScheduleStepInterrupt(nextStepTime.value());
	} while (repeat);
}

/*static*/ float Move::MotorStepsToMovement(size_t drive, int32_t endpoint)
{
	return ((float)(endpoint))/reprap.GetPlatform().DriveStepsPerUnit(drive);
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[MaxAxes], uint8_t moveType, AxesBitmap xAxes, AxesBitmap yAxes) const
{
	GetCurrentMachinePosition(m, IsRawMotorMove(moveType));
	if (moveType == 0)
	{
		InverseAxisAndBedTransform(m, xAxes, yAxes);
	}
}

// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and isPrinting is true unless we are currently executing an extruding but non-printing move
int32_t Move::GetAccumulatedExtrusion(size_t extruder, bool& isPrinting)
{
	const size_t drive = extruder + reprap.GetGCodes().GetTotalAxes();
	if (drive < MaxTotalDrivers)
	{
		return mainDDARing.GetAccumulatedExtrusion(extruder, drive, isPrinting);
	}

	isPrinting = false;
	return 0;
}

void Move::SetXYBedProbePoint(size_t index, float x, float y)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point index out of range\n");
	}
	else
	{
		probePoints.SetXYBedProbePoint(index, x, y);
	}
}

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError)
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point Z index out of range\n");
	}
	else
	{
		probePoints.SetZBedProbePoint(index, z, wasXyCorrected, wasError);
	}
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
// If 'wantNozzlePosition is true then we return the nozzle position when the point is probed, else we return the probe point itself
float Move::GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const
{
	x = probePoints.GetXCoord(count);
	y = probePoints.GetYCoord(count);
	if (wantNozzlePosition)
	{
		const ZProbe& rp = reprap.GetPlatform().GetCurrentZProbeParameters();
		x -= rp.xOffset;
		y -= rp.yOffset;
	}
	return probePoints.GetZHeight(count);
}

// Enter or leave simulation mode
void Move::Simulate(uint8_t simMode)
{
	simulationMode = simMode;
	if (simMode != 0)
	{
		mainDDARing.ResetSimulationTime();
	}
}

// Adjust the leadscrews
// This is only ever called after bed probing, so we can assume that no such move is already pending.
void Move::AdjustLeadscrews(const floatc_t corrections[])
{
	for (float& smc : specialMoveCoords)
	{
		smc = 0.0;
	}
	const AxisDriversConfig& config = reprap.GetPlatform().GetAxisDriversConfig(Z_AXIS);
	for (size_t i = 0; i < config.numDrivers; ++i)
	{
		if (config.driverNumbers[i] < MaxTotalDrivers)
		{
			specialMoveCoords[config.driverNumbers[i]] = corrections[i];
		}
	}
	bedLevellingMoveAvailable = true;
}

// Return the idle timeout in seconds
float Move::IdleTimeout() const
{
	return (float)idleTimeout * 0.001;
}

// Set the idle timeout in seconds
void Move::SetIdleTimeout(float timeout)
{
	idleTimeout = (uint32_t)lrintf(timeout * 1000.0);
}

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

// Process M204
GCodeResult Move::ConfigureAccelerations(GCodeBuffer&gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('S'))
	{
		// For backwards compatibility with old versions of Marlin (e.g. for Cura and the Prusa fork of slic3r), set both accelerations
		seen = true;
		maxTravelAcceleration = maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('P'))
	{
		seen = true;
		maxPrintingAcceleration = gb.GetFValue();
	}
	if (gb.Seen('T'))
	{
		seen = true;
		maxTravelAcceleration = gb.GetFValue();
	}
	if (!seen)
	{
		reply.printf("Maximum printing acceleration %.1f, maximum travel acceleration %.1f", (double)maxPrintingAcceleration, (double)maxTravelAcceleration);
	}
	return GCodeResult::ok;
}

// Process M593
GCodeResult Move::ConfigureDynamicAcceleration(GCodeBuffer& gb, const StringRef& reply)
{
	bool seen = false;
	if (gb.Seen('F'))
	{
		seen = true;
		const float f = gb.GetFValue();
		if (f >= 4.0 && f <= 10000.0)
		{
			drcPeriod = 1.0/f;
			drcEnabled = true;
		}
		else
		{
			drcEnabled = false;
		}
	}
	if (gb.Seen('L'))
	{
		seen = true;
		drcMinimumAcceleration = max<float>(gb.GetFValue(), 1.0);		// very low accelerations cause problems with the maths
	}

	if (!seen)
	{
		if (reprap.GetMove().IsDRCenabled())
		{
			reply.printf("Dynamic ringing cancellation at %.1fHz, min. acceleration %.1f", (double)(1.0/drcPeriod), (double)drcMinimumAcceleration);
		}
		else
		{
			reply.copy("Dynamic ringing cancellation is disabled");
		}
	}
	return GCodeResult::ok;
}

// End
