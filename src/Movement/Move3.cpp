/*
 * Move3.cpp
 *
 *  Created on: 8 Nov 2024
 *      Author: David
 *
 *  This file contains the various transforms used for computing moves, bed compensation and bed probing
 */

#include "Move.h"
#include "MoveDebugFlags.h"
#include <Platform/RepRap.h>
#include <Platform/Platform.h>
#include <GCodes/GCodes.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>

void Move::AxisAndBedTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool, bool useBedCompensation) const noexcept
{
	AxisTransform(xyzPoint, tool);
	if (useBedCompensation)
	{
		BedTransform(xyzPoint, tool);
	}
}

void Move::InverseAxisAndBedTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	InverseBedTransform(xyzPoint, tool);
	InverseAxisTransform(xyzPoint, tool);
}

// Do the Axis transform BEFORE the bed transform
void Move::AxisTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	// Identify the lowest Y axis
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	const size_t lowestYAxis = yAxes.LowestSetBit();
	if (lowestYAxis < numVisibleAxes)
	{
		// Found a Y axis. Use this one when correcting the X coordinate.
		const AxesBitmap xAxes = Tool::GetXAxes(tool);
		const size_t lowestXAxis = xAxes.LowestSetBit();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += (compensateXY ? tanXY() * xyzPoint[lowestYAxis] : 0.0) + tanXZ() * xyzPoint[Z_AXIS];
			}
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] += (compensateXY ? 0.0 : tanXY() * xyzPoint[lowestXAxis]) + tanYZ() * xyzPoint[Z_AXIS];
			}
		}
	}
}

// Invert the Axis transform AFTER the bed transform
void Move::InverseAxisTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	// Identify the lowest Y axis
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const AxesBitmap yAxes = Tool::GetYAxes(tool);
	const size_t lowestYAxis = yAxes.LowestSetBit();
	if (lowestYAxis < numVisibleAxes)
	{
		// Found a Y axis. Use this one when correcting the X coordinate.
		const AxesBitmap xAxes = Tool::GetXAxes(tool);
		const size_t lowestXAxis = xAxes.LowestSetBit();
		for (size_t axis = 0; axis < numVisibleAxes; ++axis)
		{
			if (yAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= ((compensateXY ? 0.0 : tanXY() * xyzPoint[lowestXAxis]) + tanYZ() * xyzPoint[Z_AXIS]);
			}
			if (xAxes.IsBitSet(axis))
			{
				xyzPoint[axis] -= ((compensateXY ? tanXY() * xyzPoint[lowestYAxis] : 0.0) + tanXZ() * xyzPoint[Z_AXIS]);
			}
		}
	}
}

// Compute the height correction needed at a point, ignoring taper
float Move::ComputeHeightCorrection(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	float zCorrection = 0.0;
	unsigned int numCorrections = 0;
	const GridDefinition& grid = GetGrid();
	const AxesBitmap axis1Axes = Tool::GetAxisMapping(tool, grid.GetAxisNumber(1));

	// Transform the Z coordinate based on the average correction for each axis used as an X or Y axis.
	Tool::GetAxisMapping(tool, grid.GetAxisNumber(0))
		.Iterate([this, xyzPoint, tool, axis1Axes, &zCorrection, &numCorrections](unsigned int axis0Axis, unsigned int) noexcept
					{
						const float axis0Coord = xyzPoint[axis0Axis] + Tool::GetOffset(tool, axis0Axis);
						axis1Axes.Iterate([this, xyzPoint, tool, axis0Coord, &zCorrection, &numCorrections](unsigned int axis1Axis, unsigned int) noexcept
											{
												const float axis1Coord = xyzPoint[axis1Axis] + Tool::GetOffset(tool, axis1Axis);
												zCorrection += heightMap.GetInterpolatedHeightError(axis0Coord, axis1Coord);
												++numCorrections;
											}
										);
					}
				);

	if (numCorrections > 1)
	{
		zCorrection /= numCorrections;			// take an average
	}

	return zCorrection + zShift;
}

// Do the bed transform AFTER the axis transform
void Move::BedTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	if (usingMesh)
	{
		const float toolHeight = xyzPoint[Z_AXIS] + Tool::GetOffset(tool, Z_AXIS);			// the requested nozzle height above the bed
		if (!useTaper || toolHeight < taperHeight)
		{
			const float zCorrection = ComputeHeightCorrection(xyzPoint, tool);
			xyzPoint[Z_AXIS] += (useTaper && zCorrection < taperHeight) ? (taperHeight - toolHeight) * recipTaperHeight * zCorrection : zCorrection;
		}
	}
}

// Invert the bed transform BEFORE the axis transform
void Move::InverseBedTransform(float xyzPoint[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	if (usingMesh)
	{
		const float zCorrection = ComputeHeightCorrection(xyzPoint, tool);
		if (!useTaper || zCorrection >= taperHeight)	// need check on zCorrection to avoid possible divide by zero
		{
			xyzPoint[Z_AXIS] -= zCorrection;
		}
		else
		{
			const float toolZoffset = Tool::GetOffset(tool, Z_AXIS);
			const float zreq = (xyzPoint[Z_AXIS] - (taperHeight - toolZoffset) * zCorrection * recipTaperHeight)/(1.0 - zCorrection * recipTaperHeight);
			if (zreq + toolZoffset < taperHeight)
			{
				xyzPoint[Z_AXIS] = zreq;
			}
		}
	}
}

// Normalise the bed transform to have zero height error at these bed coordinates
void Move::SetZeroHeightError(const float coords[MaxAxes]) noexcept
{
	if (usingMesh)
	{
		float tempCoords[MaxAxes];
		memcpyf(tempCoords, coords, ARRAY_SIZE(tempCoords));
		AxisTransform(tempCoords, nullptr);
		const GridDefinition& grid = GetGrid();
		zShift = -heightMap.GetInterpolatedHeightError(tempCoords[grid.GetAxisNumber(0)], tempCoords[grid.GetAxisNumber(1)]);
	}
	else
	{
		zShift = 0.0;
	}
}

void Move::SetIdentityTransform() noexcept
{
	probePoints.SetIdentity();
	heightMap.ClearGridHeights();
	heightMap.UseHeightMap(false);
	usingMesh = false;
	zShift = 0.0;
	reprap.MoveUpdated();
}

void Move::SetXYBedProbePoint(size_t index, float x, float y) noexcept
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

void Move::SetZBedProbePoint(size_t index, float z, bool wasXyCorrected, bool wasError) noexcept
{
	if (index >= MaxProbePoints)
	{
		reprap.GetPlatform().Message(ErrorMessage, "Z probe point index out of range\n");
	}
	else
	{
		probePoints.SetZBedProbePoint(index, z, wasXyCorrected, wasError);
	}
}

// This returns the (X, Y) points to probe the bed at probe point count.  When probing, it returns false.
// If called after probing has ended it returns true, and the Z coordinate probed is also returned.
// If 'wantNozzlePosition is true then we return the nozzle position when the point is probed, else we return the probe point itself
float Move::GetProbeCoordinates(int count, float& x, float& y, bool wantNozzlePosition) const noexcept
{
	x = probePoints.GetXCoord(count);
	y = probePoints.GetYCoord(count);
	if (wantNozzlePosition)
	{
		const auto zp = reprap.GetPlatform().GetEndstops().GetZProbe(reprap.GetGCodes().GetCurrentZProbeNumber());
		if (zp.IsNotNull())
		{
			x -= zp->GetOffset(X_AXIS);
			y -= zp->GetOffset(Y_AXIS);
		}
	}
	return probePoints.GetZHeight(count);
}

const char *_ecv_array Move::GetCompensationTypeString() const noexcept
{
	return (usingMesh) ? "mesh" : "none";
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Load the height map from file, returning true if an error occurred with the error reason appended to the buffer
bool Move::LoadHeightMapFromFile(FileStore *f, const char *_ecv_array fname, const StringRef& r) noexcept
{
	const bool err = heightMap.LoadFromFile(f, fname, r
#if SUPPORT_PROBE_POINTS_FILE
											, false				// loading the height map, not the probe points file
#endif
											);
	if (err)
	{
		heightMap.ClearGridHeights();							// make sure we don't end up with a partial height map
	}
	else
	{
		zShift = 0.0;
	}
	float minError, maxError;
	(void)heightMap.GetStatistics(latestMeshDeviation, minError, maxError);
	reprap.MoveUpdated();
	return err;
}

// Save the height map to a file returning true if an error occurred
bool Move::SaveHeightMapToFile(FileStore *f, const char *_ecv_array fname) noexcept
{
	return heightMap.SaveToFile(f, fname, zShift);
}

# if SUPPORT_PROBE_POINTS_FILE

// Load the probe points map from a file returning true if an error occurred
bool Move::LoadProbePointsFromFile(FileStore *f, const char *_ecv_array fname, const StringRef& r) noexcept
{
	return heightMap.LoadFromFile(f, fname, r, true);
}

void Move::ClearProbePointsInvalid() noexcept
{
	heightMap.ClearProbePointsInvalid();
}

# endif

#endif	// HAS_MASS_STORAGE || HAS_SBC_INTERFACE

void Move::SetTaperHeight(float h) noexcept
{
	useTaper = (h > 1.0);
	if (useTaper)
	{
		taperHeight = h;
		recipTaperHeight = 1.0/h;
	}
	reprap.MoveUpdated();
}

// Enable mesh bed compensation
bool Move::UseMesh(bool b) noexcept
{
	usingMesh = heightMap.UseHeightMap(b);
	reprap.MoveUpdated();
	return usingMesh;
}

float Move::AxisCompensation(unsigned int axis) const noexcept
{
	return (axis < ARRAY_SIZE(tangents)) ? tangents[axis] : 0.0;
}

void Move::SetAxisCompensation(unsigned int axis, float tangent) noexcept
{
	if (axis < ARRAY_SIZE(tangents))
	{
		tangents[axis] = tangent;
		reprap.MoveUpdated();
	}
}

bool Move::IsXYCompensated() const
{
	return compensateXY;
}

void Move::SetXYCompensation(bool xyCompensation)
{
	compensateXY = xyCompensation;
	reprap.MoveUpdated();
}

// Calibrate or set the bed equation after probing, returning true if an error occurred
// sParam is the value of the S parameter in the G30 command that provoked this call.
// Caller already owns the GCode movement lock.
GCodeResult Move::FinishedBedProbing(MovementState& ms, int sParam, const StringRef& reply) noexcept
{
	GCodeResult ret = GCodeResult::ok;
	const size_t numPoints = probePoints.NumberOfProbePoints();

	if (sParam < 0)
	{
		// A negative sParam just prints the probe heights
		probePoints.ReportProbeHeights(numPoints, reply);
	}
	else if (numPoints < (size_t)sParam)
	{
		reply.printf("Bed calibration : %d factor calibration requested but only %d points provided\n", sParam, numPoints);
		ret = GCodeResult::error;
	}
	else
	{
		if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::ZProbing))
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
			ret = GCodeResult::error;
		}
		else if (kinematics->SupportsAutoCalibration())
		{
			if (kinematics->DoAutoCalibration(ms, sParam, probePoints, reply))
			{
				ret = GCodeResult::error;
			}
		}
		else
		{
			reply.copy("This kinematics does not support auto-calibration");
			ret = GCodeResult::error;
		}
	}

	// Clear out the Z heights so that we don't re-use old points.
	// This allows us to use different numbers of probe point on different occasions.
	probePoints.ClearProbeHeights();
	return ret;
}

// End
