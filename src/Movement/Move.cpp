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
#include "MoveDebugFlags.h"
#include "StepTimer.h"
#include <Platform/Platform.h>
#include <Platform/Event.h>
#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Tools/Tool.h>
#include <Endstops/ZProbe.h>
#include <Platform/TaskPriorities.h>
#include <AppNotifyIndices.h>
#include "StepperDrivers/SmartDrivers.h"

#if SUPPORT_IOBITS
# include <Platform/PortControl.h>
#endif

#if SUPPORT_CAN_EXPANSION
# include <CAN/CanMotion.h>
# include <CAN/CanInterface.h>
#endif

#ifdef DUET3_MB6XD
# include <pmc/pmc.h>
#endif

#include <limits>

#define AVOID_SHORT_SEGMENTS	(0)

constexpr float MinStepPulseTiming = 0.2;												// we assume that we always generate step high and low times at least this wide without special action

Task<Move::MoveTaskStackWords> Move::moveTask;

// Object model table and functions
// Note: if using GCC version 7.3.1 20180622 and lambda functions are used in this table, you must compile this file with option -std=gnu++17.
// Otherwise the table will be allocated in RAM instead of flash, which wastes too much RAM.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)					OBJECT_MODEL_FUNC_BODY(Move, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition, ...)	OBJECT_MODEL_FUNC_IF_BODY(Move, _condition, __VA_ARGS__)
#define OBJECT_MODEL_ARRAY_COUNT(_value)		OBJECT_MODEL_ARRAY_COUNT_BODY(Move, _value)
#define OBJECT_MODEL_ARRAY_VALUE(...)			OBJECT_MODEL_ARRAY_VALUE_BODY(Move, __VA_ARGS__)

constexpr ObjectModelArrayTableEntry Move::objectModelArrayTable[] =
{
	// 0. Axes
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetTotalAxes()),
		OBJECT_MODEL_ARRAY_VALUE(self, 9)
	},
	// 1. Extruders
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetNumExtruders()),
		OBJECT_MODEL_ARRAY_VALUE(self, 10)
	},
	// 2. Motion system queues
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(ARRAY_SIZE(rings)),
		OBJECT_MODEL_ARRAY_VALUE(&self->rings[context.GetLastIndex()])
	},

	// 3. Axis drivers
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT(self->axisDrivers[context.GetLastIndex()].numDrivers),
		OBJECT_MODEL_ARRAY_VALUE(self->axisDrivers[context.GetIndex(1)].driverNumbers[context.GetLastIndex()])
	},

	// 4. Workplace coordinate offsets
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(NumCoordinateSystems),
		OBJECT_MODEL_ARRAY_VALUE_NOSELF(reprap.GetGCodes().GetWorkplaceOffset(context.GetIndex(1), context.GetLastIndex()), 3)
	},

	// 5. Motion systems
	{
		nullptr,					// no lock needed
#if SUPPORT_ASYNC_MOVES
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetNumMotionSystemsUsed()),
#else
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(1),
#endif
		OBJECT_MODEL_ARRAY_VALUE_NOSELF(&reprap.GetGCodes().GetMovementState(context.GetLastIndex())),
	},

#if SUPPORT_COORDINATE_ROTATION
	// 6. Rotation centre coordinates
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(2),
		OBJECT_MODEL_ARRAY_VALUE_NOSELF(reprap.GetGCodes().GetPrimaryMovementState().g68Centre[context.GetLastIndex()])
	},
#elif SUPPORT_KEEPOUT_ZONES
	{
		nullptr,
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(0),
		OBJECT_MODEL_ARRAY_VALUE_NOSELF(nullptr)
	}
#endif

#if SUPPORT_KEEPOUT_ZONES
	// 7. Keepout zone list
	{
		nullptr,					// no lock needed
		OBJECT_MODEL_ARRAY_COUNT_NOSELF(reprap.GetGCodes().GetNumKeepoutZones()),
		OBJECT_MODEL_ARRAY_VALUE_NOSELF((reprap.GetGCodes().IsKeepoutZoneDefined(context.GetLastIndex())) ? ExpressionValue(reprap.GetGCodes().GetKeepoutZone(context.GetLastIndex())) : ExpressionValue(nullptr))
	},
#endif
};

DEFINE_GET_OBJECT_MODEL_ARRAY_TABLE(Move)

size_t Move::GetMaxElementsToReturn(const ObjectModelArrayTableEntry *entry, const ObjectExplorationContext& context) const noexcept
{
	// If we are returning the move.axes array, and we are not just returning frequently-changed data, then limit the number of axes returned.
	// PanelDue doesn't know that we return a limited number of axes, so if the requester is PanelDue then return an extra axis, which we can do because we omit many fields.
	return (entry == &objectModelArrayTable[0] && context.ShouldTruncateArrays())
				? ((context.ForPanelDue()) ? MaxReportedAxesForPanelDue : MaxReportedAxes)
					: 0;
}

static inline const char *_ecv_array GetFilamentName(size_t extruder) noexcept
{
	const Filament *_ecv_null fil = Filament::GetFilamentByExtruder(extruder);
	return (fil == nullptr) ? "" : fil->GetName();
}

constexpr ObjectModelTableEntry Move::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. Move members
#if SUPPORT_S_CURVE
	{ "accelerationTime",		OBJECT_MODEL_FUNC(self->accelerationTime * StepClocksToSeconds, 3),								ObjectModelEntryFlags::notPanelDue },
#endif
	{ "axes",					OBJECT_MODEL_FUNC_ARRAY(0), 																	ObjectModelEntryFlags::live },
	{ "backlashFactor",			OBJECT_MODEL_FUNC((int32_t)self->GetBacklashCorrectionDistanceFactor()),						ObjectModelEntryFlags::none },
	{ "calibration",			OBJECT_MODEL_FUNC(self, 3),																		ObjectModelEntryFlags::notPanelDue },
	{ "compensation",			OBJECT_MODEL_FUNC(self, 6),																		ObjectModelEntryFlags::none },
	{ "currentMove",			OBJECT_MODEL_FUNC(self, 2),																		ObjectModelEntryFlags::live },
	{ "extruders",				OBJECT_MODEL_FUNC_ARRAY(1),																		ObjectModelEntryFlags::liveNotPanelDue },
	{ "idle",					OBJECT_MODEL_FUNC(self, 1),																		ObjectModelEntryFlags::none },
#if SUPPORT_KEEPOUT_ZONES
	{ "keepout",				OBJECT_MODEL_FUNC_ARRAY(7),																		ObjectModelEntryFlags::notPanelDue },
#endif
	{ "kinematics",				OBJECT_MODEL_FUNC(self->kinematics),															ObjectModelEntryFlags::none },
	{ "limitAxes",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().LimitAxes()),										ObjectModelEntryFlags::none },
	{ "motionSystems",			OBJECT_MODEL_FUNC_ARRAY(5),																		ObjectModelEntryFlags::liveNotPanelDue },
	{ "noMovesBeforeHoming",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().NoMovesBeforeHoming()),								ObjectModelEntryFlags::none },
	{ "printingAcceleration",	OBJECT_MODEL_FUNC_NOSELF(InverseConvertAcceleration(reprap.GetGCodes().GetCurrentMovementState(context).raw.maxPrintingAcceleration), 1),	ObjectModelEntryFlags::verbose },
	{ "queue",					OBJECT_MODEL_FUNC_ARRAY(2),																		ObjectModelEntryFlags::none },
#if SUPPORT_COORDINATE_ROTATION
	{ "rotation",				OBJECT_MODEL_FUNC(self, 16),																	ObjectModelEntryFlags::notPanelDue | ObjectModelEntryFlags::verbose },
#endif
	{ "shaping",				OBJECT_MODEL_FUNC(&self->axisShaper, 0),														ObjectModelEntryFlags::none },
	{ "speedFactor",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetCurrentMovementState(context).speedFactor, 2),	ObjectModelEntryFlags::none },
	{ "travelAcceleration",		OBJECT_MODEL_FUNC_NOSELF(InverseConvertAcceleration(reprap.GetGCodes().GetCurrentMovementState(context).raw.maxTravelAcceleration), 1),		ObjectModelEntryFlags::verbose },
#if SUPPORT_S_CURVE
	{ "usingSCurve",			OBJECT_MODEL_FUNC(self->usingSCurve),															ObjectModelEntryFlags::none },
#endif
	{ "virtualEPos",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetCurrentMovementState(context).latestVirtualExtruderPosition, 5),		ObjectModelEntryFlags::liveNotPanelDue | ObjectModelEntryFlags::verbose },
	{ "workplaceNumber",		OBJECT_MODEL_FUNC_NOSELF((int32_t)reprap.GetGCodes().GetCurrentMovementState(context).currentCoordinateSystem),		ObjectModelEntryFlags::verbose },

	// 1. Move.Idle members
	{ "factor",					OBJECT_MODEL_FUNC(self->GetIdleCurrentFactor(), 1),												ObjectModelEntryFlags::none },
	{ "timeout",				OBJECT_MODEL_FUNC(0.001f * (float)self->idleTimeout, 1),										ObjectModelEntryFlags::none },

	// 2. move.currentMove members
	{ "acceleration",			OBJECT_MODEL_FUNC(self->GetAccelerationMmPerSecSquared(0), 1),									ObjectModelEntryFlags::liveNotPanelDue },
	{ "deceleration",			OBJECT_MODEL_FUNC(self->GetDecelerationMmPerSecSquared(0), 1),									ObjectModelEntryFlags::liveNotPanelDue },
	{ "distance",				OBJECT_MODEL_FUNC(self->GetCurrentMoveDistance(0), 2),											ObjectModelEntryFlags::liveNotPanelDue },
	{ "duration",				OBJECT_MODEL_FUNC(self->GetCurrentMoveDuration(0), 2),											ObjectModelEntryFlags::liveNotPanelDue },
	{ "extrusionRate",			OBJECT_MODEL_FUNC(self->GetTotalExtrusionRate(0), 2),											ObjectModelEntryFlags::liveNotPanelDue },
	{ "filePosition",			OBJECT_MODEL_FUNC_IF(self->GetCurrentMoveFilePosition(0) != noFilePosition, self->GetCurrentMoveFilePosition(0)),		ObjectModelEntryFlags::liveNotPanelDue },
# if SUPPORT_LASER
	{ "laserPwm",				OBJECT_MODEL_FUNC_IF_NOSELF(reprap.GetGCodes().GetMachineType() == MachineType::laser,
															reprap.GetPlatform().GetLaserPwm(), 2),								ObjectModelEntryFlags::liveNotPanelDue },
# endif
	{ "requestedSpeed",			OBJECT_MODEL_FUNC(self->GetRequestedSpeedMmPerSec(0), 1),										ObjectModelEntryFlags::liveNotPanelDue },
	{ "topSpeed",				OBJECT_MODEL_FUNC(self->GetTopSpeedMmPerSec(0), 1),												ObjectModelEntryFlags::liveNotPanelDue },

	// 3. move.calibration members
	{ "final",					OBJECT_MODEL_FUNC(self, 5),																		ObjectModelEntryFlags::none },
	{ "initial",				OBJECT_MODEL_FUNC(self, 4),																		ObjectModelEntryFlags::none },
	{ "numFactors",				OBJECT_MODEL_FUNC((int32_t)self->numCalibratedFactors),											ObjectModelEntryFlags::none },

	// 4. move.calibration.initialDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetDeviationFromMean(), 3),					ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->initialCalibrationDeviation.GetMean(), 3),								ObjectModelEntryFlags::none },

	// 5. move.calibration.finalDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetDeviationFromMean(), 3),					ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestCalibrationDeviation.GetMean(), 3),								ObjectModelEntryFlags::none },

	// 6. move.compensation members
	{ "fadeHeight",				OBJECT_MODEL_FUNC((self->useTaper) ? self->taperHeight : std::numeric_limits<float>::quiet_NaN(), 1),	ObjectModelEntryFlags::none },
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	{ "file",					OBJECT_MODEL_FUNC_IF(self->usingMesh, self->heightMap.GetFileName()),							ObjectModelEntryFlags::none },
#endif
	{ "liveGrid",				OBJECT_MODEL_FUNC_IF(self->usingMesh, (const GridDefinition *)&self->GetGrid()),				ObjectModelEntryFlags::none },
	{ "meshDeviation",			OBJECT_MODEL_FUNC_IF(self->usingMesh, self, 7),													ObjectModelEntryFlags::none },
	{ "probeGrid",				OBJECT_MODEL_FUNC_NOSELF((const GridDefinition *)&reprap.GetGCodes().GetDefaultGrid()),			ObjectModelEntryFlags::none },
	{ "skew",					OBJECT_MODEL_FUNC(self, 8),																		ObjectModelEntryFlags::none },
	{ "type",					OBJECT_MODEL_FUNC(self->GetCompensationTypeString()),											ObjectModelEntryFlags::none },

	// 7. move.compensation.meshDeviation members
	{ "deviation",				OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetDeviationFromMean(), 3),							ObjectModelEntryFlags::none },
	{ "mean",					OBJECT_MODEL_FUNC(self->latestMeshDeviation.GetMean(), 3),										ObjectModelEntryFlags::none },

	// 8. move.compensation.skew members
	{ "compensateXY",			OBJECT_MODEL_FUNC(self->compensateXY),															ObjectModelEntryFlags::none },
	{ "tanXY",					OBJECT_MODEL_FUNC(self->tanXY(), 4),															ObjectModelEntryFlags::none },
	{ "tanXZ",					OBJECT_MODEL_FUNC(self->tanXZ(), 4),															ObjectModelEntryFlags::none },
	{ "tanYZ",					OBJECT_MODEL_FUNC(self->tanYZ(), 4),															ObjectModelEntryFlags::none },

	// 9. move.axes[] members
	{ "acceleration",		OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->NormalAcceleration(context.GetLastIndex())), 1),				ObjectModelEntryFlags::none },
	{ "babystep",			OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetTotalBabyStepOffset(context.GetLastIndex()), 3),					ObjectModelEntryFlags::none },
	{ "backlash",			OBJECT_MODEL_FUNC(self->backlashMm[context.GetLastIndex()], 3),													ObjectModelEntryFlags::notPanelDue },
	{ "current",			OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(context.GetLastIndex(), 906))),								ObjectModelEntryFlags::notPanelDue },
	{ "drivers",			OBJECT_MODEL_FUNC_ARRAY(3),																						ObjectModelEntryFlags::notPanelDue },
	{ "homed",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().IsAxisHomed(context.GetLastIndex())),								ObjectModelEntryFlags::none },
	{ "jerk",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->GetMaxInstantDv(context.GetLastIndex())), 1),				ObjectModelEntryFlags::none },
	{ "letter",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetAxisLetters()[context.GetLastIndex()]),							ObjectModelEntryFlags::none },
	{ "machinePosition",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetCurrentMovementState(context).LiveMachineCoordinate(context.GetLastIndex()), 3),	ObjectModelEntryFlags::live },
	{ "max",				OBJECT_MODEL_FUNC(self->AxisMaximum(context.GetLastIndex()), 2),												ObjectModelEntryFlags::none },
	{ "maxProbed",			OBJECT_MODEL_FUNC(self->axisMaximaProbed.IsBitSet(context.GetLastIndex())),										ObjectModelEntryFlags::notPanelDue },
	{ "microstepping",		OBJECT_MODEL_FUNC(self, 12),																					ObjectModelEntryFlags::notPanelDue },
	{ "min",				OBJECT_MODEL_FUNC(self->AxisMinimum(context.GetLastIndex()), 2),												ObjectModelEntryFlags::none },
	{ "minProbed",			OBJECT_MODEL_FUNC(self->axisMinimaProbed.IsBitSet(context.GetLastIndex())),										ObjectModelEntryFlags::notPanelDue },
	{ "percentCurrent",		OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(context.GetLastIndex(), 913))),								ObjectModelEntryFlags::notPanelDue },
#ifndef DUET_NG
	{ "percentStstCurrent",	OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(context.GetLastIndex(), 917))),								ObjectModelEntryFlags::notPanelDue },
#endif
#if SUPPORT_PHASE_STEPPING
	{ "phaseStep",			OBJECT_MODEL_FUNC(self->GetStepMode(context.GetLastIndex()) == StepMode::phase),								ObjectModelEntryFlags::notPanelDue },
#endif
	{ "printingJerk",		OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->GetPrintingInstantDv(context.GetLastIndex())), 1),		ObjectModelEntryFlags::none },
	{ "reducedAcceleration", OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->Acceleration(context.GetLastIndex(), true)), 1),			ObjectModelEntryFlags::none },
	{ "speed",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->MaxFeedrate(context.GetLastIndex())), 1),					ObjectModelEntryFlags::none },
	{ "stepPos",			OBJECT_MODEL_FUNC(self->GetLiveMotorPosition(context.GetLastIndex())),											ObjectModelEntryFlags::liveNotPanelDue },
	{ "stepsPerMm",			OBJECT_MODEL_FUNC(self->DriveStepsPerMm(context.GetLastIndex()), 2),											ObjectModelEntryFlags::none },
	{ "userPosition",		OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetUserCoordinate(reprap.GetGCodes().GetCurrentMovementState(context), context.GetLastIndex()), 3), ObjectModelEntryFlags::live },
	{ "visible",			OBJECT_MODEL_FUNC_NOSELF(context.GetLastIndex() < (int32_t)reprap.GetGCodes().GetVisibleAxes()),				ObjectModelEntryFlags::none },
	{ "workplaceOffsets",	OBJECT_MODEL_FUNC_ARRAY(4),																						ObjectModelEntryFlags::notPanelDue },

	// 10. move.extruders[] members
	{ "acceleration",		OBJECT_MODEL_FUNC(InverseConvertAcceleration(self->NormalAcceleration(ExtruderToLogicalDrive(context.GetLastIndex()))), 1),			ObjectModelEntryFlags::none },
	{ "current",			OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(ExtruderToLogicalDrive(context.GetLastIndex()), 906))),							ObjectModelEntryFlags::notPanelDue },
	{ "driver",				OBJECT_MODEL_FUNC(self->extruderDrivers[context.GetLastIndex()]),																	ObjectModelEntryFlags::notPanelDue },
	{ "factor",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetExtrusionFactor(context.GetLastIndex()), 3),											ObjectModelEntryFlags::none },
	{ "filament",			OBJECT_MODEL_FUNC_NOSELF(GetFilamentName(context.GetLastIndex())),																	ObjectModelEntryFlags::none },
	{ "filamentDiameter",	OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetFilamentDiameter(context.GetLastIndex()), 3),										ObjectModelEntryFlags::none },
	{ "jerk",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->GetMaxInstantDv(ExtruderToLogicalDrive(context.GetLastIndex()))), 1),			ObjectModelEntryFlags::none },
	{ "microstepping",		OBJECT_MODEL_FUNC(self, 13),																										ObjectModelEntryFlags::notPanelDue },
	{ "nonlinear",			OBJECT_MODEL_FUNC(self, 11),																										ObjectModelEntryFlags::notPanelDue },
	{ "percentCurrent",		OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(context.GetLastIndex(), 913))),													ObjectModelEntryFlags::notPanelDue },
#ifndef DUET_NG
	{ "percentStstCurrent",	OBJECT_MODEL_FUNC((int32_t)(self->GetMotorCurrent(context.GetLastIndex(), 917))),													ObjectModelEntryFlags::notPanelDue },
#endif
#if SUPPORT_PHASE_STEPPING
	{ "phaseStep",			OBJECT_MODEL_FUNC(self->GetStepMode(ExtruderToLogicalDrive(context.GetLastIndex())) == StepMode::phase),							ObjectModelEntryFlags::notPanelDue },
#endif
	{ "position",			OBJECT_MODEL_FUNC_NOSELF(ExpressionValue(reprap.GetGCodes().GetCurrentMovementState(context).LiveMachineCoordinate(ExtruderToLogicalDrive(context.GetLastIndex())), 1)),	ObjectModelEntryFlags::liveNotPanelDue },
	{ "pressAdv",			OBJECT_MODEL_FUNC(&(self->GetExtruderShaperForExtruder(context.GetLastIndex()))),													ObjectModelEntryFlags::none },
	{ "pressureAdvance",	OBJECT_MODEL_FUNC(self->GetExtruderShaperForExtruder(context.GetLastIndex()).GetK0Seconds(), 3),									ObjectModelEntryFlags::obsolete },
	{ "printingJerk",		OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->GetPrintingInstantDv(ExtruderToLogicalDrive(context.GetLastIndex()))), 1),	ObjectModelEntryFlags::none },
	{ "rawPosition",		OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetRawExtruderTotalByDrive(context.GetLastIndex()), 1), 								ObjectModelEntryFlags::liveNotPanelDue },
	{ "speed",				OBJECT_MODEL_FUNC(InverseConvertSpeedToMmPerMin(self->MaxFeedrate(ExtruderToLogicalDrive(context.GetLastIndex()))), 1),				ObjectModelEntryFlags::none },
	{ "stepsPerMm",			OBJECT_MODEL_FUNC(self->DriveStepsPerMm(ExtruderToLogicalDrive(context.GetLastIndex())), 2),										ObjectModelEntryFlags::none },

	// 11. move.extruders[].nonlinear members
#if SUPPORT_NONLINEAR_EXTRUSION
	{ "a",					OBJECT_MODEL_FUNC(self->nonlinearExtrusion[context.GetLastIndex()].A, 3),									ObjectModelEntryFlags::none },
	{ "b",					OBJECT_MODEL_FUNC(self->nonlinearExtrusion[context.GetLastIndex()].B, 3),									ObjectModelEntryFlags::none },
	{ "upperLimit",			OBJECT_MODEL_FUNC(self->nonlinearExtrusion[context.GetLastIndex()].limit, 2),								ObjectModelEntryFlags::none },
#endif

	// 12. move.axes[].microstepping members
	{ "interpolated",		OBJECT_MODEL_FUNC(self->GetMicrostepInterpolation(context.GetLastIndex())),									ObjectModelEntryFlags::none },
	{ "value",				OBJECT_MODEL_FUNC((int32_t)self->GetMicrostepping(context.GetLastIndex())),									ObjectModelEntryFlags::none },

	// 13. move.extruders[].microstepping members
	{ "interpolated",		OBJECT_MODEL_FUNC(self->GetMicrostepInterpolation(ExtruderToLogicalDrive(context.GetLastIndex()))),			ObjectModelEntryFlags::none },
	{ "value",				OBJECT_MODEL_FUNC((int32_t)self->GetMicrostepping(ExtruderToLogicalDrive(context.GetLastIndex()))),			ObjectModelEntryFlags::none },

	// 14. boards[0].drivers[] members
	{ "config",				OBJECT_MODEL_FUNC(self, 15),																				ObjectModelEntryFlags::none },
	{ "status",				OBJECT_MODEL_FUNC(self->GetLocalDriverStatus(context.GetLastIndex()).all),									ObjectModelEntryFlags::liveNotPanelDue },

	// 15. boards[0].drivers[].config members
	{ "direction",			OBJECT_MODEL_FUNC((int32_t)self->directions[context.GetLastIndex()]), 										ObjectModelEntryFlags::none },
#if HAS_SMART_DRIVERS
	{ "mode",				OBJECT_MODEL_FUNC_NOSELF((int32_t)SmartDrivers::GetDriverMode(context.GetLastIndex())), 					ObjectModelEntryFlags::none },
#endif

#if SUPPORT_COORDINATE_ROTATION
	// 16. move.rotation members
	{ "angle",				OBJECT_MODEL_FUNC_NOSELF(reprap.GetGCodes().GetPrimaryMovementState().g68Angle),							ObjectModelEntryFlags::none },
	{ "centre",				OBJECT_MODEL_FUNC_ARRAY(6),																					ObjectModelEntryFlags::none },
#endif
};

constexpr uint8_t Move::objectModelTableDescriptor[] =
{
	16 + SUPPORT_COORDINATE_ROTATION,										// number of sections
	18 + SUPPORT_COORDINATE_ROTATION + SUPPORT_KEEPOUT_ZONES + 2 * SUPPORT_S_CURVE,		// section 0
	2,																		// section 1
	8 + SUPPORT_LASER,														// section 2
	3,																		// section 3
	2,																		// section 4
	2,																		// section 5
	6 + (int)(HAS_MASS_STORAGE || HAS_SBC_INTERFACE),						// section 6
	2,																		// section 7
	4,																		// section 8
#ifdef DUET_NG	// Duet WiFi/Ethernet doesn't have settable standstill current and doesn't support phase stepping
	23,																		// section 9: move.axes[]
	17,																		// section 10: move.extruders[]
#else
	24 + SUPPORT_PHASE_STEPPING,											// section 9: move.axes[]
	18 + SUPPORT_PHASE_STEPPING,											// section 10: move.extruders[]
#endif
#if SUPPORT_NONLINEAR_EXTRUSION
	3,																		// section 11: move.extruders[].nonlinear
#else
	0,
#endif
	2,																		// section 12: move.axes[].microstepping
	2,																		// section 13: move.extruders[].microstepping
	2,																		// section 14: boards.drivers[]
	1 + HAS_SMART_DRIVERS,													// section 15: boards.drivers[].config
#if SUPPORT_COORDINATE_ROTATION
	2,																		// section 16: move.rotation
#endif
};

DEFINE_GET_OBJECT_MODEL_TABLE(Move)

// The Move task starts executing here
[[noreturn]] static void MoveStart(void *param) noexcept
{
	static_cast<Move *>(param)->MoveLoop();
}

Move::Move() noexcept
	:
#ifdef DUET3_MB6XD
	  lastStepHighTime(0),
#else
	  lastStepLowTime(0),
#endif
	  lastDirChangeTime(0),
#if SUPPORT_ASYNC_MOVES
	  heightController(nullptr),
#endif
	  jerkPolicy(0),
	  numCalibratedFactors(0)
{
#if VARIABLE_NUM_DRIVERS
	numActualDirectDrivers = NumDirectDrivers;						// assume they are all available until we know otherwise
#endif

	// Kinematics must be set up here because GCodes::Init asks the kinematics for the assumed initial position
	kinematics = Kinematics::Create(KinematicsType::cartesian);		// default to Cartesian
	for (DDARing& ring : rings)
	{
		ring.Init(InitialDdaRingLength);
	}
}

void Move::Init() noexcept
{
	// Axes
#if SUPPORT_S_CURVE
	accelerationTime = 0.0;
#endif

	for (size_t axis = 0; axis < MaxAxes; ++axis)
	{
		axisMinima[axis] = DefaultAxisMinimum;
		axisMaxima[axis] = DefaultAxisMaximum;

		maxFeedrates[axis] = ConvertSpeedFromMmPerSec(DefaultAxisMaxFeedrate);
		reducedAccelerations[axis] = normalAccelerations[axis] = ConvertAcceleration(DefaultAxisAcceleration);
		printingInstantDvs[axis] = maxInstantDvs[axis] = ConvertSpeedFromMmPerSec(DefaultAxisInstantDv);
#if SUPPORT_S_CURVE
		// The jerks get initialised when the acceleration time is set, however VectorBoxIntersection may fail if jerks for unused logical drives are zero or negative
		jerks[axis] = 1.0;
#endif

		backlashMm[axis] = 0.0;
		backlashSteps[axis] = 0;
		targetBacklashSteps[axis] = currentBacklashSteps[axis] = 0;
	}

	backlashCorrectionDistanceFactor = DefaultBacklashCorrectionDistanceFactor;

	// We use different defaults for the Z axis
	maxFeedrates[Z_AXIS] = ConvertSpeedFromMmPerSec(DefaultZMaxFeedrate);
	reducedAccelerations[Z_AXIS] = normalAccelerations[Z_AXIS] = ConvertAcceleration(DefaultZAcceleration);
	printingInstantDvs[Z_AXIS] = maxInstantDvs[Z_AXIS] = ConvertSpeedFromMmPerSec(DefaultZInstantDv);

	// Extruders
	for (size_t drive = MaxAxes; drive < MaxAxesPlusExtruders; ++drive)
	{
		maxFeedrates[drive] = ConvertSpeedFromMmPerSec(DefaultEMaxFeedrate);
		normalAccelerations[drive] = reducedAccelerations[drive] = ConvertAcceleration(DefaultEAcceleration);
		printingInstantDvs[drive] = maxInstantDvs[drive] = ConvertSpeedFromMmPerSec(DefaultEInstantDv);
#if SUPPORT_S_CURVE
		// The jerks get initialised when the acceleration time is set, however VectorBoxIntersection may fail if jerks for unused logical drives are zero or negative
		jerks[drive] = 1.0;
#endif
	}

	minimumMovementSpeed = ConvertSpeedFromMmPerSec(DefaultMinFeedrate);
	axisMaximaProbed.Clear();
	axisMinimaProbed.Clear();
	idleCurrentFactor = DefaultIdleCurrentFactor;

	// Motors

#ifdef DUET3_MB6XD
	DriverEnablePins = (reprap.GetPlatform().GetBoardType() == BoardType::Duet3_6XD_v01) ? ENABLE_PINS_v01 : ENABLE_PINS_v100;
	unsigned int numErrorHighDrivers = 0;
#endif
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		directions[driver] = true;														// drive moves forwards by default
#ifdef DUET3_MB6XD
		SetPinMode(DriverEnablePins[driver], INPUT, false);								// temporarily set up the enable pin for reading
		SetPinMode(DRIVER_ERR_PINS[driver], INPUT, false);								// set up the error pin for reading
		const bool activeHighEnable = !digitalRead(DriverEnablePins[driver]);			// test whether we have a pullup or pulldown on the Enable pin
		enableValues[driver] = activeHighEnable;
		SetPinMode(DriverEnablePins[driver], (activeHighEnable) ? OUTPUT_LOW : OUTPUT_HIGH);	// set driver disabled
		if (digitalRead(DRIVER_ERR_PINS[driver]))
		{
			++numErrorHighDrivers;
		}

		// Set the default driver step timings
		driverTimingMicroseconds[driver][0] = DefaultStepWidthMicroseconds;
		driverTimingMicroseconds[driver][1] = DefaultStepIntervalMicroseconds;
		driverTimingMicroseconds[driver][2] = DefaultSetupTimeMicroseconds;
		driverTimingMicroseconds[driver][3] = DefaultHoldTimeMicroseconds;
#else
		enableValues[driver] = 0;														// assume active low enable signal
#endif
		// Set up the control pins
		SetPinMode(STEP_PINS[driver], OUTPUT_LOW);
		SetPinMode(DIRECTION_PINS[driver], OUTPUT_LOW);
#if !defined(DUET3) && !defined(DUET3MINI)
		SetPinMode(DriverEnablePins[driver], OUTPUT_HIGH);									// this is OK for the TMC2660 CS pins too
#endif

		brakeOffDelays[driver] = 0;
		motorOffDelays[driver] = DefaultDelayAfterBrakeOn;

#if SUPPORT_BRAKE_PWM
		currentBrakePwm[driver] = 0.0;
		brakeVoltages[driver] = FullyOnBrakeVoltage;
#endif
	}

#ifdef DUET3_MB6XD
	driverErrPinsActiveLow = (numErrorHighDrivers >= NumDirectDrivers/2);				// determine the error signal polarity by assuming most drivers are not in the error state
	pmc_enable_periph_clk(STEP_GATE_TC_ID);												// need to do this before we set up the step gate TC
	UpdateDriverTimings();																// this also initialises the step gate TC
	SetPinFunction(StepGatePin, StepGatePinFunction);
#endif

	// Initialise the DMs before we make any changes to them
	for (size_t drv = 0; drv < MaxAxesPlusExtruders + NumDirectDrivers; ++drv)
	{
		dms[drv].Init(drv);
		if (drv < MaxAxesPlusExtruders)
		{
			const float stepsPerMm = (drv >= MaxAxes) ? DefaultEDriveStepsPerUnit
										: (drv == Z_AXIS) ? DefaultZDriveStepsPerUnit
											: DefaultAxisDriveStepsPerUnit;
			driveStepsPerMm[drv] = stepsPerMm;
		}
	}

	// Set up the axis+extruder arrays
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; drive++)
	{
		driverState[drive] = DriverStatus::disabled;
		motorCurrents[drive] = 0.0;
		motorCurrentFraction[drive] = 1.0;
#if HAS_SMART_DRIVERS || SUPPORT_CAN_EXPANSION
		standstillCurrentPercent[drive] = (float)DefaultStandstillCurrentPercent;
#endif
		SetDriverMicrostepping(drive, 16, true);				// x16 with interpolation
	}

	// Set up the bitmaps for direct driver access
	for (size_t driver = 0; driver < NumDirectDrivers; ++driver)
	{
		dms[driver + MaxAxesPlusExtruders].driversNormallyUsed = StepPins::CalcDriverBitmap(driver);
	}

	// Set up default axis mapping
	for (size_t axis = 0; axis < MinAxes; ++axis)
	{
#ifdef PCCB
		const size_t driver = (axis + 1) % 3;					// on PCCB we map axes X Y Z to drivers 1 2 0
#else
		const size_t driver = axis;								// on most boards we map axes straight through to drives
#endif
		axisDrivers[axis].numDrivers = 1;
		axisDrivers[axis].driverNumbers[0].SetLocal(driver);
		dms[axis].driversNormallyUsed = StepPins::CalcDriverBitmap(driver);	// overwrite the default value set up earlier
	}
	linearAxes = AxesBitmap::MakeLowestNBits(3);				// XYZ axes are linear

	for (size_t axis = MinAxes; axis < MaxAxes; ++axis)
	{
		axisDrivers[axis].numDrivers = 0;
	}

	// Set up default extruders
	for (size_t extr = 0; extr < MaxExtruders; ++extr)
	{
		extruderDrivers[extr].SetLocal(extr + MinAxes);			// set up default extruder drive mapping
		dms[ExtruderToLogicalDrive(extr)].driversNormallyUsed = StepPins::CalcDriverBitmap(extr + MinAxes);
#if SUPPORT_NONLINEAR_EXTRUSION
		nonlinearExtrusion[extr].A = nonlinearExtrusion[extr].B = 0.0;
		nonlinearExtrusion[extr].limit = DefaultNonlinearExtrusionLimit;
#endif
	}

#ifndef DUET3_MB6XD
	for (uint32_t& entry : slowDriverStepTimingClocks)
	{
		entry = 0;												// reset all to zero as we have no known slow drivers yet
	}
	slowDriversBitmap = 0;										// assume no drivers need extended step pulse timing
#endif

#if HAS_SMART_DRIVERS
	// Initialise TMC driver module
# if SUPPORT_TMC51xx || SUPPORT_TMC2240_SPI
	SmartDrivers::Init();
# elif SUPPORT_TMC22xx
#  if TMC22xx_VARIABLE_NUM_DRIVERS
	SmartDrivers::Init(numSmartDrivers);
#  elif SUPPORT_TMC2240 && defined(DUET3MINI)
	SmartDrivers::Init(reprap.GetPlatform().HasTmc2240Expansion());
#  else
	SmartDrivers::Init();
#  endif
# else
	SmartDrivers::Init(DriverEnablePins, numSmartDrivers);
# endif
	temperatureShutdownDrivers.Clear();
	temperatureWarningDrivers.Clear();
	shortToGroundDrivers.Clear();
#endif

#if HAS_STALL_DETECT
	logOnStallDrivers.Clear();
	eventOnStallDrivers.Clear();
#endif

#if SUPPORT_ASYNC_MOVES
	auxMoveAvailable = false;
	auxMoveLocked = false;
#endif

	// Clear the transforms
	SetIdentityTransform();
	compensateXY = true;
	tangents[0] = tangents[1] = tangents[2] = 0.0;

	usingMesh = useTaper = false;
	zShift = 0.0;

	idleTimeout = DefaultIdleTimeout;
	moveState = MoveState::idle;
	const uint32_t now = millis();
	whenIdleTimerStarted = now;
	for (uint32_t& w : whenLastMoveAdded)
	{
		w = now;
	}

	simulationMode = SimulationMode::off;
	longestGcodeWaitInterval = 0;
	numInterruptHiccups = numPrepareHiccups = 0;
	bedLevellingMoveAvailable = false;
	activeDMs = nullptr;
	for (uint16_t& ms : microstepping)
	{
		ms = 16 | 0x8000;
	}

#if SUPPORT_PHASE_STEPPING
	phaseStepDMs = nullptr;
	for (float& rm : phaseStepMultiplier)
	{
		rm = -(1.0/16.0);
	}
	ResetPhaseStepMonitoringVariables();
#endif

	moveTask.Create(MoveStart, "Move", this, TaskPriority::MovePriority);
}

void Move::Exit() noexcept
{
	StepTimer::DisableTimerInterrupt();
	stepsTimer.CancelCallback();
#if HAS_SMART_DRIVERS
	SmartDrivers::Exit();
#endif
	for (DDARing& ring : rings)
	{
		ring.Exit();
	}
#if SUPPORT_LASER || SUPPORT_IOBITS
	delete laserTask;
	laserTask = nullptr;
#endif
	moveTask.TerminateAndUnlink();
}

[[noreturn]] void Move::MoveLoop() noexcept
{
	stepsTimer.SetCallback(Move::TimerCallback, CallbackParameter(this));
	for (;;)
	{
		if (reprap.IsStopped() || hadStepError)
		{
			// Emergency stop has been commanded, so terminate this task to prevent new moves being prepared and executed
			moveTask.TerminateAndUnlink();
		}

#if SUPPORT_REMOTE_COMMANDS
		if (CanInterface::InExpansionMode())
		{
			// In expansion mode we don't need the Move task to do anything, and in particular we must not perform idle detection.
			// We could terminate the Move task here but currently we don't because:
			// (a) if we do then we must make sure that any attempts to wake it up are benign
			// (b) in future we may wish to use the Move task to queue movement commands
			// So for now we just delay.
			delay(10000);
			continue;
		}
#endif

		bool moveRead = false;

		// See if we can add another move to ring 0
		const bool canAddRing0Move = rings[0].CanAddMove();
		if (canAddRing0Move)
		{
			// OK to add another move. First check if a special move is available.
			if (bedLevellingMoveAvailable)
			{
				moveRead = true;
				if (simulationMode < SimulationMode::partial)
				{
					if (rings[0].AddSpecialMove(MaxFeedrate(Z_AXIS), specialMoveCoords))
					{
						const uint32_t now = millis();
						const uint32_t timeWaiting = now - whenLastMoveAdded[0];
						if (timeWaiting > longestGcodeWaitInterval)
						{
							longestGcodeWaitInterval = timeWaiting;
						}
						whenLastMoveAdded[0] = now;
						moveState = MoveState::collecting;
					}
				}
				bedLevellingMoveAvailable = false;
			}
			else
			{
				// If there's a G Code move available, add it to the DDA ring for processing.
				RawMove nextMove;
				GCodes& gcodes = reprap.GetGCodes();
				if (gcodes.ReadMove(0, nextMove))							// if we have a new move
				{
					moveRead = true;
					if (simulationMode < SimulationMode::partial)			// in simulation mode partial, we don't process incoming moves beyond this point
					{
						if (nextMove.moveType == 0)
						{
							AxisAndBedTransform(nextMove.coords, nextMove.movementTool,
#if SUPPORT_SCANNING_PROBES
													!nextMove.scanningProbeMove
#else
													true
#endif
													);
						}

						const MovementError err = rings[0].AddStandardMove(nextMove, !IsRawMotorMove(nextMove.moveType));
						if (err == MovementError::ok)
						{
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - whenLastMoveAdded[0];
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							whenLastMoveAdded[0] = now;
							moveState = MoveState::collecting;
						}
						else if (err != MovementError::noMovement)
						{
							gcodes.ReportMovementError(err);
						}
					}
				}
			}
		}

		// Let ring 0 process moves
		// When there is a gap between moves it can be that we try to prepare the second move while a segment of the first move that has been delayed by input shaping is still executing.
		// To avoid this we must ensure that we prepare moves at least half an input shaper period in advance. This avoids the problem because any delayed segment of the first move
		// will be half a shaper period long. In order to handle CAN delays etc. we prepare moves [half a shaper period plus MoveTiming::AbsoluteMinimumPreparedTime] in advance,
		// with a minimum of MoveTiming::UsualMinimumPreparedTime.
		const uint32_t prepareAdvanceTime = axisShaper.GetPrepareAdvanceTime();
		uint32_t nextPrepareDelay = rings[0].Spin(prepareAdvanceTime, simulationMode, !canAddRing0Move, millis() - whenLastMoveAdded[0] >= rings[0].GetGracePeriod());

#if SUPPORT_ASYNC_MOVES
		const bool canAddRing1Move = rings[1].CanAddMove();
		if (canAddRing1Move)
		{
			if (auxMoveAvailable)
			{
				moveRead = true;
				if (rings[1].AddAsyncMove(auxMove))
				{
					const uint32_t now = millis();
					const uint32_t timeWaiting = now - whenLastMoveAdded[1];
					if (timeWaiting > longestGcodeWaitInterval)
					{
						longestGcodeWaitInterval = timeWaiting;
					}
					whenLastMoveAdded[1] = now;
					moveState = MoveState::collecting;
				}
				auxMoveAvailable = false;
			}
			else
			{
				// If there's a G Code move available, add it to the DDA ring for processing.
				RawMove nextMove;
				GCodes& gcodes = reprap.GetGCodes();
				if (gcodes.ReadMove(1, nextMove))							// if we have a new move
				{
					moveRead = true;
					if (simulationMode < SimulationMode::partial)			// in simulation mode partial, we don't process incoming moves beyond this point
					{
						if (nextMove.moveType == 0)
						{
							AxisAndBedTransform(nextMove.coords, nextMove.movementTool, true);
						}

						const MovementError err = rings[1].AddStandardMove(nextMove, !IsRawMotorMove(nextMove.moveType));
						if (err == MovementError::ok)
						{
							const uint32_t now = millis();
							const uint32_t timeWaiting = now - whenLastMoveAdded[1];
							if (timeWaiting > longestGcodeWaitInterval)
							{
								longestGcodeWaitInterval = timeWaiting;
							}
							whenLastMoveAdded[1] = now;
							moveState = MoveState::collecting;
						}
						else if (err != MovementError::noMovement)
						{
							gcodes.ReportMovementError(err);
						}
					}
				}
			}
		}

		const uint32_t auxPrepareDelay = rings[1].Spin(prepareAdvanceTime, simulationMode, !canAddRing1Move,  millis() - whenLastMoveAdded[1] >= rings[1].GetGracePeriod());
		if (auxPrepareDelay < nextPrepareDelay)
		{
			nextPrepareDelay = auxPrepareDelay;
		}
#endif

		if (simulationMode == SimulationMode::debug && reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::SimulateSteppingDrivers))
		{
			while (activeDMs != nullptr)
			{
				SimulateSteppingDrivers(reprap.GetPlatform());
			}
		}

		// Reduce motor current to standby if the rings have been idle for long enough
		if (   rings[0].IsIdle()
#if SUPPORT_ASYNC_MOVES
			&& rings[1].IsIdle()
#endif
		   )
		{
			if (   moveState == MoveState::executing
				&& reprap.GetGCodes().GetPauseState() == PauseState::notPaused	// for now we don't go into idle hold when we are paused (is this sensible?)
			   )
			{
				whenIdleTimerStarted = millis();				// record when we first noticed that the machine was idle
				moveState = MoveState::timing;
			}
			else if (moveState == MoveState::timing && millis() - whenIdleTimerStarted >= idleTimeout)
			{
				SetDriversIdle();								// put all drives in idle hold
				moveState = MoveState::idle;
			}
		}
		else
		{
			moveState = MoveState::executing;
		}

		// We need to be woken when one of the following is true:
		// 1. If moves are being executed and there are unprepared moves in the queue, when it is time to prepare more moves.
		// 2. If the queue was full and all moves in it were prepared, when we have completed one or more moves.
		// 3. In order to implement idle timeout, we must wake up regularly anyway, say every half second (MoveTiming::StandardMoveWakeupInterval)
		if (!moveRead && nextPrepareDelay != 0)
		{
			TaskBase::TakeIndexed(NotifyIndices::Move, nextPrepareDelay);
		}
	}
}

// This is called from GCodes to tell the Move task that a move is available
void Move::MoveAvailable() noexcept
{
	if (moveTask.IsRunning())
	{
		moveTask.Give(NotifyIndices::Move);
	}
}

// Tell the lookahead ring we are waiting for it to empty and return true if it is. Called from the Main task.
bool Move::WaitingForAllMovesFinished(MovementSystemNumber msNumber
#if SUPPORT_ASYNC_MOVES
										, LogicalDrivesBitmap logicalDrivesOwned
#endif
									 ) noexcept
{
	if (!rings[msNumber].SetWaitingToEmpty())
	{
		return false;
	}

	// If input shaping is enabled then movement may continue for a little while longer
#if SUPPORT_ASYNC_MOVES
	return logicalDrivesOwned.IterateWhile([this](unsigned int axisOrExtruder, unsigned int) noexcept -> bool
												{
													return !dms[axisOrExtruder].MotionPending();
												}
											 );
#else
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		if (dms[drive].MotionPending())
		{
			return false;
		}
	}
#endif
	return true;
}

// Return the number of actually probed probe points
unsigned int Move::GetNumProbedProbePoints() const noexcept
{
	return probePoints.NumberOfProbePoints();
}

// Try to push some babystepping through the lookahead queue, returning the amount pushed
// This is called by the Main task, so we need to lock out the Move task while doing this
float Move::PushBabyStepping(MovementSystemNumber msNumber,size_t axis, float amount) noexcept
{
	TaskCriticalSectionLocker lock;						// lock out the Move task

	return rings[msNumber].PushBabyStepping(axis, amount);
}

// Change the kinematics to the specified type if it isn't already
// If it is already correct leave its parameters alone.
// This violates our rule on no dynamic memory allocation after the initialisation phase,
// however this function is normally called only when M665, M667 and M669 commands in config.g are processed.
bool Move::SetKinematics(KinematicsType k) noexcept
{
	if (kinematics->GetKinematicsType() != k)
	{
		Kinematics *_ecv_from _ecv_null const nk = Kinematics::Create(k);
		if (nk == nullptr)
		{
			return false;
		}
		delete kinematics;
		kinematics = _ecv_not_null(nk);
		reprap.MoveUpdated();
	}
	return true;
}

// Return true if this is a raw motor move
bool Move::IsRawMotorMove(uint8_t moveType) const noexcept
{
	return moveType == 2 || (moveType != 0 && kinematics->GetHomingMode() != HomingMode::homeCartesianAxes);
}

// Return true if the specified point is accessible to the Z probe
bool Move::IsAccessibleProbePoint(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept
{
	return kinematics->IsReachable(axesCoords, axes);
}

// Pause the print as soon as we can, returning true if we are able to skip any moves and updating ms.pauseRestorePoint to the first move we skipped.
bool Move::PausePrint(MovementState& ms) noexcept
{
	return rings[ms.GetNumber()].PauseMoves(ms);
}

#if HAS_VOLTAGE_MONITOR || HAS_STALL_DETECT

// Pause the print immediately, returning true if we were able to skip or abort any moves and setting up to the move we aborted
bool Move::LowPowerOrStallPause(MovementState& ms) noexcept
{
	return rings[ms.GetNumber()].LowPowerOrStallPause(ms);
}

// Stop generating steps
void Move::CancelStepping() noexcept
{
	StepTimer::DisableTimerInterrupt();
}

#endif

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP

// Helper function to convert a time period (expressed in StepTimer::Ticks) to a frequency in Hz
static inline uint32_t TickPeriodToFreq(StepTimer::Ticks tickPeriod) noexcept
{
	return StepTimer::GetTickRate()/tickPeriod;
}

#endif

void Move::Diagnostics(unsigned int part, const StringRef& reply) noexcept
{
	switch (part)
	{
	case 0:
		{
			const float totalDelayToReport = (float)StepTimer::GetMovementDelay() * (1000.0/(float)StepTimer::GetTickRate());

#if SUPPORT_CAN_EXPANSION
			const float ownDelayToReport = (float)StepTimer::GetOwnMovementDelay() * (1000.0/(float)StepTimer::GetTickRate());
#endif
			reply.printf("=== Move ===\nSegments created %u, maxWait %" PRIu32 "ms, bed comp in use: %s, height map offset %.3f, hiccups added %u/%u"
#if SUPPORT_CAN_EXPANSION
						" (%.2f/%.2fms)"
#else
						" (%.2fms)"
#endif
						", max steps late %" PRIi32
#if 1	//debug
						", ebfmin %.2f, ebfmax %.2f"
#endif
						,

						MoveSegment::NumCreated(), longestGcodeWaitInterval, GetCompensationTypeString(), (double)zShift, numPrepareHiccups, numInterruptHiccups,
#if SUPPORT_CAN_EXPANSION
						(double)ownDelayToReport, (double)totalDelayToReport,
#else
						(double)totalDelayToReport,
#endif
						DriveMovement::GetAndClearMaxStepsLate()
#if 1
						, (double)minExtrusionPending, (double)maxExtrusionPending
#endif
			);

			longestGcodeWaitInterval = 0;
			numInterruptHiccups = numPrepareHiccups = 0;
#if 1	//debug
			minExtrusionPending = maxExtrusionPending = 0.0;
#endif

			DriveMovement::DiagnosticHeader(reply);
			for (size_t drive = 0; drive < reprap.GetGCodes().GetTotalAxes(); ++drive)
			{
				dms[drive].Diagnostics(reply);
			}
			for (size_t drive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); drive < MaxAxesPlusExtruders; ++drive)
			{
				dms[drive].Diagnostics(reply);
			}

#if 0	// DEBUG
			reply.lcat("ADM:");
			for (const DriveMovement *dm = activeDMs; dm != nullptr; dm = dm->nextDM)
			{
				reply.catf(" %u,%u,%lu", dm->drive, (unsigned int)dm->state, dm->nextStepTime);
			}
			reply.cat(" PDM:");
			for (const DriveMovement *dm = phaseStepDMs; dm != nullptr; dm = dm->nextDM)
			{
				reply.catf(" %u,%u,%lu", dm->drive, (unsigned int)dm->state, dm->nextStepTime);
			}
			reply.catf(" Now: %lu", StepTimer::GetMovementTimerTicks());
#endif
			StepTimer::Diagnostics(reply);
		}
		break;

	case 1:
	case 2:
	case 3:
		// Show the driver diagnostics. We can fit 4 in each response. Duet 2 has 12 drivers so we need up to 3 responses.
		for (size_t drive = 4 * (part - 1); drive < min<size_t>(NumDirectDrivers, 4 * part); ++drive)
		{
			reply.lcatf("Driver %u: ", drive);
#ifdef DUET3_MB6XD
			reply.cat((HasDriverError(drive)) ? "error" : "ok");
#elif HAS_SMART_DRIVERS
			if (drive < numSmartDrivers)
			{
				const StandardDriverStatus status = SmartDrivers::GetStatus(drive, false, false);
				status.AppendText(reply, 0);
				if (!status.notPresent)
				{
					SmartDrivers::AppendDriverStatus(drive, reply);
				}
			}
#endif
		}
		break;

	case 4:
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		reply.lcatf("Phase step loop runtime (us): min=%" PRIu32 ", max=%" PRIu32 ", frequency (Hz): min=%" PRIu32 ", max=%" PRIu32,
				StepTimer::TicksToIntegerMicroseconds(minPSControlLoopRuntime), StepTimer::TicksToIntegerMicroseconds(maxPSControlLoopRuntime),
				TickPeriodToFreq(maxPSControlLoopCallInterval), TickPeriodToFreq(minPSControlLoopCallInterval));
		ResetPhaseStepMonitoringVariables();
#endif
		// Show the status of each DDA ring
		for (size_t i = 0; i < ARRAY_SIZE(rings); ++i)
		{
			rings[i].Diagnostics(reply, i);
		}
		break;
	}
}

// Convert distance to steps for a particular drive
MovementError Move::MotorMovementToSteps(size_t drive, float coord, int32_t& whereToStore) const noexcept
{
	const float pos = coord * driveStepsPerMm[drive];
	constexpr float limit = (float)(std::numeric_limits<int32_t>::max() - 10);
	if (fabsf(pos) <= limit)
	{
		whereToStore = lrintf(pos);
		return MovementError::ok;
	}
	return MovementError::microstep_position_too_large;
}

// Convert motor coordinates to machine coordinates. Used after homing and after individual motor moves.
// This is computationally expensive on a delta or SCARA machine, so only call it when necessary, and never from the step ISR.
void Move::MotorStepsToCartesian(const int32_t motorPos[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept
{
	kinematics->MotorStepsToCartesian(motorPos, driveStepsPerMm, numVisibleAxes, numTotalAxes, machinePos);
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintTransforms))
	{
		debugPrintf("Forward transformed %" PRIi32 " %" PRIi32 " %" PRIi32 " to %.2f %.2f %.2f\n",
			motorPos[0], motorPos[1], motorPos[2], (double)machinePos[0], (double)machinePos[1], (double)machinePos[2]);
	}
}

// Convert Cartesian coordinates to motor steps, axes only, returning true if successful.
// Used to perform movement and G92 commands.
// This may be called from an ISR, e.g. via Kinematics::OnHomingSwitchTriggered, DDA::SetPositions and Move::EndPointToMachine
// If isCoordinated is false then multi-mode kinematics such as SCARA are allowed to switch mode if necessary to make the specified machine position reachable
MovementError Move::CartesianToMotorSteps(const float machinePos[MaxAxes], int32_t motorPos[MaxAxes], bool isCoordinated) const noexcept
{
	const MovementError rslt = kinematics->CartesianToMotorSteps(machinePos, driveStepsPerMm, reprap.GetGCodes().GetVisibleAxes(), reprap.GetGCodes().GetTotalAxes(), motorPos, isCoordinated);
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PrintTransforms))
	{
		if (rslt != MovementError::ok)
		{
			debugPrintf("Unable to transform");
			for (size_t i = 0; i < reprap.GetGCodes().GetVisibleAxes(); ++i)
			{
				debugPrintf(" %.2f", (double)machinePos[i]);
			}
			debugPrintf("\n");
		}
		else
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
	return rslt;
}

float Move::MotorStepsToMovement(size_t drive, int32_t endpoint) const noexcept
{
	return ((float)(endpoint))/driveStepsPerMm[drive];
}

// Return the transformed machine coordinates
void Move::GetCurrentUserPosition(float m[MaxAxes], MovementSystemNumber msNumber, bool doBedCompensation, const Tool *tool) const noexcept
{
	GetCurrentMachinePosition(m, msNumber);
	if (doBedCompensation)
	{
		InverseAxisAndBedTransform(m, tool);
	}
}

void Move::SetMotorPosition(size_t drive, int32_t pos, bool clearBacklash) noexcept
{
#if SUPPORT_PHASE_STEPPING
	uint32_t now = StepTimer::GetTimerTicks();
	uint16_t currentPhases[MaxSmartDrivers] = {0};
	DriveMovement *dm = &dms[drive];

	if (dm->IsPhaseStepEnabled())
	{
		UpdateCurrentMotion(drive, now, dm->phaseStepControl.mParams);
		IterateLocalDrivers(drive, [dm, &currentPhases](uint8_t driver) noexcept -> void {
			currentPhases[driver] = dm->phaseStepControl.CalculateStepPhase((size_t)driver);
			dm->phaseStepControl.SetPhaseOffset(driver, 0);
		});
	}
#endif

	if (drive < MaxAxes)
	{
		if (clearBacklash)
		{
			targetBacklashSteps[drive] = currentBacklashSteps[drive] = 0;
		}
		dms[drive].SetMotorPosition(pos + currentBacklashSteps[drive]);
	}
	else
	{
		dms[drive].SetMotorPosition(pos);				// we don't store backlash info for extruders
	}

#if SUPPORT_PHASE_STEPPING
	if (dm->IsPhaseStepEnabled())
	{
		UpdateCurrentMotion(drive, now, dm->phaseStepControl.mParams);
		IterateLocalDrivers(drive, [dm, &currentPhases](uint8_t driver) noexcept -> void {
			uint16_t newPhase = dm->phaseStepControl.CalculateStepPhase((size_t)driver);

			dm->phaseStepControl.SetPhaseOffset(driver, currentPhases[driver] - newPhase);
		});
	}
#endif
}

void Move::SetMotorPositions(LogicalDrivesBitmap drives, const int32_t *_ecv_array positions, bool clearBacklash) noexcept
{
	drives.Iterate([this, positions, clearBacklash](unsigned int drive, unsigned int count) noexcept -> void { SetMotorPosition(drive, positions[drive], clearBacklash); });
}

void Move::SetLastEndpoints(MovementSystemNumber msNumber, LogicalDrivesBitmap logicalDrives, const int32_t *_ecv_array ep) noexcept
{
	rings[msNumber].SetLastEndpoints(logicalDrives, ep);
}

void Move::GetLastEndpoints(MovementSystemNumber msNumber, LogicalDrivesBitmap logicalDrives, int32_t returnedEndpoints[MaxAxesPlusExtruders]) const noexcept
{
	rings[msNumber].GetLastEndpoints(logicalDrives, returnedEndpoints);
}

int32_t Move::GetLastEndpoint(MovementSystemNumber msNumber, size_t drive) const noexcept
{
	return rings[msNumber].GetLastEndpoint(drive);
}

void Move::ChangeEndpointsAfterHoming(MovementSystemNumber msNumber, LogicalDrivesBitmap drives, const int32_t endpoints[MaxAxes]) noexcept
{
	rings[msNumber].SetLastEndpoints(drives, endpoints);
	SetMotorPositions(drives, endpoints, true);
}

void Move::ChangeSingleEndpointAfterHoming(MovementSystemNumber msNumber, size_t drive, int32_t ep) noexcept
{
	rings[msNumber].SetLastEndpoint(drive, ep);
	SetMotorPosition(drive, ep, true);
}

// Enter or leave simulation mode
void Move::Simulate(SimulationMode simMode) noexcept
{
	simulationMode = simMode;
	if (simMode != SimulationMode::off)
	{
		rings[0].ResetSimulationTime();
	}
}

// Adjust the leadscrews
// This is only ever called after bed probing, so we can assume that no such move is already pending.
void Move::AdjustLeadscrews(const floatc_t corrections[]) noexcept
{
	const size_t numZdrivers = GetAxisDriversConfig(Z_AXIS).numDrivers;
	for (size_t i = 0; i < MaxDriversPerAxis; ++i)
	{
		specialMoveCoords[i] = (i < numZdrivers) ? (float)corrections[i] : 0.0;
	}
	bedLevellingMoveAvailable = true;
	MoveAvailable();
}

// Return the idle timeout in seconds
float Move::IdleTimeout() const noexcept
{
	return (float)idleTimeout * 0.001;
}

// Set the idle timeout in seconds
void Move::SetIdleTimeout(float timeout) noexcept
{
	idleTimeout = (uint32_t)lrintf(timeout * 1000.0);
	reprap.MoveUpdated();
}

#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE

// Write settings for resuming the print
// The GCodes module deals with the head position so all we need worry about is the bed compensation
// We don't handle random probe point bed compensation, and we assume that if a height map is being used it is the default one.
bool Move::WriteResumeSettings(FileStore *f) const noexcept
{
	return kinematics->WriteResumeSettings(f) && (!usingMesh || f->Write("G29 S1\n"));
}

bool Move::WriteMoveParameters(FileStore *f) const noexcept
{
	if (axisMinimaProbed.IsNonEmpty() || axisMaximaProbed.IsNonEmpty())
	{
		bool ok = f->Write("; Probed axis limits\n");
		if (ok)
		{
			ok = WriteAxisLimits(f, axisMinimaProbed, axisMinima, 1);
		}
		if (ok)
		{
			ok = WriteAxisLimits(f, axisMaximaProbed, axisMaxima, 0);
		}
		return ok;
	}

	return true;
}

bool Move::WriteAxisLimits(FileStore *f, AxesBitmap axesProbed, const float limits[MaxAxes], int sParam) noexcept
{
	if (axesProbed.IsEmpty())
	{
		return true;
	}

	String<StringLength100> scratchString;
	scratchString.printf("M208 S%d", sParam);
	axesProbed.Iterate([&scratchString, limits](unsigned int axis, unsigned int) noexcept -> void { scratchString.catf(" %c%.2f", reprap.GetGCodes().GetAxisLetters()[axis], (double)limits[axis]); });
	scratchString.cat('\n');
	return f->Write(scratchString.c_str());
}

#endif

#if SUPPORT_REMOTE_COMMANDS

GCodeResult Move::EutSetRemotePressureAdvanceV1(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) noexcept -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								dms[driver].extruderShaper.SetParametersSimple(msg.values[count]);
							}
						}
				   );
	return rslt;
}

GCodeResult Move::EutSetRemotePressureAdvanceV2(const CanMessageMultipleDrivesRequest<ShortPressureAdvanceParameters>& msg, size_t dataLength, const StringRef& reply) noexcept
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) noexcept -> void
						{
							if (driver >= NumDirectDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								dms[driver].extruderShaper.SetParameters(msg.values[count]);
							}
						}
				   );
	return rslt;
}

void Move::RevertPosition(const CanMessageRevertPosition& msg) noexcept
{
	// Construct a MovementLinearShaped message to revert the position. The move must be shorter than clocksAllowed.
	// When writing this, clocksAllowed was equivalent to 40ms.
	// We allow 10ms delay time to allow the motor to stop and reverse direction, 10ms acceleration time, 5ms steady time and 10ms deceleration time.
	CanMessageMovementLinearShaped msg2;
	msg2.accelerationClocks = msg2.decelClocks = msg.clocksAllowed/4;
	msg2.steadyClocks = msg.clocksAllowed/8;
	msg2.whenToExecute = StepTimer::GetMovementTimerTicks() + msg.clocksAllowed/4;
	msg2.extruderDrives = 0;
	msg2.seq = 0;

	// We start and finish at zero speed, so we move (3/8)*clocksAllowed*topSpeed distance. Since we normalise moves to unit distance, this is equal to one.
	// So topSpeed is 8/(3 * clocksAllowed) and acceleration is (8/(3 * clocksAllowed))/(clocksAllowed/4) = 32/(3 * clocksAllowed^2).
	msg2.acceleration = msg2.deceleration = 32.0/(3.0 * msg.clocksAllowed * msg.clocksAllowed);

	constexpr size_t numDriversToRevert = min<size_t>(NumDirectDrivers, MaxLinearDriversPerCanSlave);
	msg2.numDrivers = numDriversToRevert;

	size_t index = 0;
	bool needSteps = false;
	for (size_t driver = 0; driver < numDriversToRevert; ++driver)
	{
		int32_t steps = 0;
		if ((msg.whichDrives & (1u << driver)) !=0)
		{
			const int32_t stepsWanted = msg.finalStepCounts[index++];
			const int32_t stepsTaken = GetLastMoveStepsTaken(driver);
			if (((stepsWanted >= 0 && stepsTaken > stepsWanted) || (stepsWanted <= 0 && stepsTaken < stepsWanted)))
			{
				steps = stepsWanted - stepsTaken;
				needSteps = true;
			}
		}
		msg2.perDrive[driver].steps = steps;
	}

	if (needSteps)
	{
		AddMoveFromRemote(msg2);
	}
}

#endif

// Get the current machine coordinates, independently of the above functions, so not affected by other tasks calling them
void Move::GetLiveMachineCoordinates(float coords[MaxAxes]) const noexcept
{
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	const size_t numTotalAxes = reprap.GetGCodes().GetTotalAxes();

	// Get the positions of each motor
	int32_t currentMotorPositions[MaxAxes];
	{
		AtomicCriticalSectionLocker lock; // to make sure we get a consistent set of coordinates
		const uint32_t now = StepTimer::GetTimerTicks();
		for (size_t i = 0; i < numTotalAxes; ++i)
		{
			const float position = dms[i].GetCurrentPosition(now);
			currentMotorPositions[i] = std::lrint(position - currentBacklashSteps[i]);
		}
	}

	MotorStepsToCartesian(currentMotorPositions, numVisibleAxes, numTotalAxes, coords);
}

// Fetch the machine coordinates. We need to pass a tool context because that affects the inverse bed transform.
void Move::UpdateLiveMachineCoordinates(float coords[MaxAxes], const Tool *_ecv_null tool) const noexcept
{
	GetLiveMachineCoordinates(coords);
	InverseAxisAndBedTransform(coords, tool);

	for (size_t i = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); i < MaxAxesPlusExtruders; ++i)
	{
		coords[i] = (float)dms[i].currentMotorPosition / driveStepsPerMm[i];
	}
}

void Move::SetLatestCalibrationDeviation(const Deviation& d, uint8_t numFactors) noexcept
{
	latestCalibrationDeviation = d;
	numCalibratedFactors = numFactors;
	reprap.MoveUpdated();
}

void Move::SetInitialCalibrationDeviation(const Deviation& d) noexcept
{
	initialCalibrationDeviation = d;
	reprap.MoveUpdated();
}

// Set the mesh deviation. Caller must call MoveUpdated() after calling this. We don't do that here because the caller may change Move in other ways first.
void Move::SetLatestMeshDeviation(const Deviation& d) noexcept
{
	latestMeshDeviation = d;
}

inline void Move::WakeMoveTask() noexcept
{
	// No need to check whether the Move task is running because GiveFromISR does that
#if SUPPORT_REMOTE_COMMANDS
	if (!inExpansionMode)
#endif
	{
		if (inInterrupt())
		{
			moveTask.GiveFromISR(NotifyIndices::Move);
		}
		else
		{
			moveTask.Give(NotifyIndices::Move);
		}
	}
}

// Laser, IOBits and scanning Z probe support

Task<Move::LaserTaskStackWords> *_ecv_null Move::laserTask = nullptr;		// the task used to manage laser power or IOBits

extern "C" [[noreturn]] void LaserTaskStart(void * pvParameters) noexcept
{
	reprap.GetMove().LaserTaskRun();
}

// This is called when laser mode is selected, or IOBits is enabled, or a scanning Z probe is configured, or extruder heater feedforward is configured, or M571 is used
void Move::CreateLaserTask() noexcept
{
	TaskCriticalSectionLocker lock;
	if (laserTask == nullptr)
	{
		laserTask = new Task<LaserTaskStackWords>;
		laserTask->Create(LaserTaskStart, "LASER", nullptr, TaskPriority::LaserPriority);
	}
}

// Wake up the laser task, if there is one (must check!). Call this at the start of a new move from standstill (not from an ISR)
void Move::WakeLaserTask() noexcept
{
	if (laserTask != nullptr)
	{
		laserTask->Give(NotifyIndices::Laser);
	}
}

#if SUPPORT_SCANNING_PROBES

static void ScanningProbeGlobalTimerCallback(CallbackParameter cp) noexcept
{
	((Move *)cp.vp)->ScanningProbeTimerCallback();
}

void Move::ScanningProbeTimerCallback() noexcept
{
	probeControl.readingNeeded = true;
	if (inInterrupt())
	{
		laserTask->GiveFromISR(NotifyIndices::Laser);
	}
	else
	{
		laserTask->Give(NotifyIndices::Laser);
	}
}

// This is called by DDA::Prepare when committing a scanning probe reading.
// The first reading has already been taken and axis0index has been incremented or decremented to account for this.
void Move::PrepareScanningProbeDataCollection(const DDA& dda, const PrepParams& params) noexcept
{
	probeControl.numReadingsNeeded = reprap.GetGCodes().GetNumScanningProbeReadingsLeftToTake();
	probeControl.nextReadingNeeded = 1;							// we already took the first reading
	if (probeControl.numReadingsNeeded != 0)
	{
		probeControl.accelClocks = params.TotalAccelClocks();
#if SUPPORT_S_CURVE
		if (dda.flags.useScurve)
		{
			// The following is only approximate but should be good enough
			probeControl.acceleration = (float)((params.peakAcceleration * params.TotalAccelClocks() - (motioncalc_t)0.5 * params.jerk * (msquare((motioncalc_t)params.phaseClocks[0]) + msquare((motioncalc_t)params.phaseClocks[2])))/params.TotalAccelClocks());
			probeControl.deceleration = (float)((params.peakDeceleration * params.TotalDecelClocks() + (motioncalc_t)0.5 * params.jerk * (msquare((motioncalc_t)params.phaseClocks[4]) + msquare((motioncalc_t)params.phaseClocks[6])))/params.TotalDecelClocks());
		}
		else
		{
			probeControl.acceleration = (float)params.peakAcceleration;
			probeControl.deceleration = (float)params.peakDeceleration;
		}
#else
		probeControl.acceleration = (float)params.acceleration;
		probeControl.deceleration = (float)params.deceleration;
#endif
		probeControl.initialSpeed = dda.startSpeed;
		probeControl.topSpeed = dda.topSpeed;
		probeControl.steadyClocks = params.SteadyClocks();
		probeControl.distancePerReading = (float)(params.totalDistance/(motioncalc_t)probeControl.numReadingsNeeded);
#if SUPPORT_S_CURVE
		probeControl.accelDistance = (float)params.TotalAccelDistance();
		probeControl.decelStartDistance = (float)(params.totalDistance - params.TotalDecelDistance());
#else
		probeControl.accelDistance = (float)params.accelDistance;
		probeControl.decelStartDistance = (float)params.decelStartDistance;
#endif
		probeControl.startTime = dda.afterPrepare.moveStartTime;
		probeControl.timer.SetCallback(ScanningProbeGlobalTimerCallback, CallbackParameter(this));
#if 0
		debugPrintf("nrn=%u ac=%" PRIu32 " sc=%" PRIu32 " is=%.3e ts=%.3e a=%.3e d=%.3e dpr=%.3f ad=%.3f dsd=%.3f\n",
					probeControl.numReadingsNeeded, probeControl.accelClocks, probeControl.steadyClocks,
					(double)probeControl.initialSpeed, (double)probeControl.topSpeed, (double)probeControl.acceleration, (double)probeControl.deceleration,
					(double)probeControl.distancePerReading, (double)probeControl.accelDistance, (double)probeControl.decelStartDistance);
#endif
		SetupNextScanningProbeReading();
	}
}

inline float fastLimSqrtf(float f) noexcept
{
	return (f > 0.0) ? fastSqrtf(f) : 0.0;
}

// Set up a laser task wakeup for the next reading we need to take
void Move::SetupNextScanningProbeReading() noexcept
{
	if (probeControl.nextReadingNeeded <= probeControl.numReadingsNeeded)
	{
		const float distance = probeControl.distancePerReading * probeControl.nextReadingNeeded;
		uint32_t wakeupTime;
		if (distance < probeControl.accelDistance)
		{
			// 0.5*a*t^2 + ut - s = 0
			// so t = -u + sqrt(u^2 + 2*a*s)/a
			wakeupTime = (uint32_t)((fastLimSqrtf(fsquare(probeControl.initialSpeed) + 2 * probeControl.acceleration * distance) - probeControl.initialSpeed)/probeControl.acceleration);
		}
		else if (distance <= probeControl.decelStartDistance)
		{
			wakeupTime = (uint32_t)((distance - probeControl.accelDistance)/probeControl.topSpeed) + probeControl.accelClocks;
		}
		else
		{
			// 0.5*a*t^2 + ut - s = 0
			// so t = -u + sqrt(u^2 + 2*a*s)/a
			// However the operand of the square root may be slightly negative for the final point because of rounding error, so we need to limit it
			wakeupTime = (uint32_t)((fastLimSqrtf(fsquare(probeControl.topSpeed) + 2 * probeControl.deceleration * (distance - probeControl.decelStartDistance)) - probeControl.topSpeed)/probeControl.deceleration)
							+ probeControl.accelClocks + probeControl.steadyClocks;
		}
//		debugPrintf("nr=%u, wt=%" PRIu32 ", dec=%.3e\n", probeControl.nextReadingNeeded, wakeupTime, (double)probeControl.deceleration);
		++probeControl.nextReadingNeeded;
		if (probeControl.timer.ScheduleMovementCallbackFromIsr(wakeupTime + probeControl.startTime))
		{
			ScanningProbeTimerCallback();
		}
	}
}

#endif

void Move::LaserTaskRun() noexcept
{
#if SUPPORT_SCANNING_PROBES || SUPPORT_LASER
	GCodes& gcodes = reprap.GetGCodes();
#endif
	Platform& platform = reprap.GetPlatform();
	for (;;)
	{
		uint32_t ticks = portMAX_DELAY;
#if SUPPORT_SCANNING_PROBES
		if (probeControl.readingNeeded)
		{
			probeControl.readingNeeded = false;
			gcodes.TakeScanningProbeReading();
			SetupNextScanningProbeReading();
		}
		else
#endif

#if SUPPORT_LASER
			if (gcodes.GetMachineType() == MachineType::laser)
		{
			// Manage the laser power
			ticks = rings[0].ManageLaserPower(platform);
		}
		else
#endif
		{
			// Manage the feedforward, output on extrude, and IOBits
			ticks = rings[0].ManageIOBitsAndFeedForward(platform);
		}

		// Sleep until it is time to wake up again
		(void)TaskBase::TakeIndexed(NotifyIndices::Laser, ticks);
	}
}

// Extruder and filament monitor support

// Clear the movement pending value for an extruder
void Move::ClearExtruderMovementPending(size_t extruder) noexcept
{
	dms[ExtruderToLogicalDrive(extruder)].ClearMovementPending();
}

// Return when we started doing normal moves after the most recent extruder-only move, in millisecond ticks
uint32_t Move::ExtruderPrintingSince(size_t logicalDrive) const noexcept
{
	return dms[logicalDrive].extruderPrintingSince;
}

// Get the accumulated extruder motor steps taken by an extruder since the last call to this function. Used by the filament monitoring code.
// Returns the number of motor steps moved since the last call, and sets isPrinting true unless we are currently executing an extruding but non-printing move
// This is called from the filament monitor ISR and from FilamentMonitor::Spin
int32_t Move::GetAccumulatedExtrusion(size_t logicalDrive, bool& isPrinting) noexcept
{
	DriveMovement& dm = dms[logicalDrive];
	AtomicCriticalSectionLocker lock;							// we don't want a move to complete and the ISR update the movement accumulators while we are doing this
	const int32_t ret = dm.movementAccumulator.load();
	const int32_t adjustment = dm.GetNetStepsTakenThisSegment();
	dm.movementAccumulator.store(-adjustment);
	isPrinting = dms[logicalDrive].extruderPrinting;
	return ret + adjustment;
}

#if SUPPORT_S_CURVE

// Calculate the initial speed given the duration, distance, acceleration and jerk
static inline motioncalc_t CalcInitialSpeed(uint32_t duration, motioncalc_t distance, motioncalc_t a, motioncalc_t j) noexcept
{
	return distance/(motioncalc_t)duration - (OneHalf * a + OneSixth * j * (motioncalc_t)duration) * (motioncalc_t)duration;
}

#else

// Calculate the initial speed given the duration, distance and acceleration
static inline motioncalc_t CalcInitialSpeed(uint32_t duration, motioncalc_t distance, motioncalc_t a) noexcept
{
	return distance/(motioncalc_t)duration - OneHalf * a * (motioncalc_t)duration;
}

#endif

// Add a segment into a segment list, which may be empty.
// If the list is not empty then the new segment may overlap segments already in the list.
// The units of the input parameters are steps for distance and step clocks for time.
MoveSegment *Move::AddSegment(MoveSegment *list, uint32_t startTime, uint32_t duration, motioncalc_t distance, motioncalc_t a,
#if SUPPORT_S_CURVE
	 	 	 	 	 	 	 	 motioncalc_t j, MovementFlags moveFlags, motioncalc_t pressureAdvanceClocks
#else
								 	 	 	 	 MovementFlags moveFlags, motioncalc_t pressureAdvanceClocksTimesDuration
#endif
							) noexcept
{
	if ((int32_t)duration <= 0)
	{
		const StringRef& dbgRef = Platform::genericDebugBuffer.GetRef();
		dbgRef.printf("Adding zero or negative duration segment: d=%3e a=%.3e\n", (double)distance, (double)a);
		Platform::hasGenericDebug = true;
	}

#if SUPPORT_S_CURVE
	// Adjust the distance and acceleration (and implicitly the initial speed) to account for pressure advance
	distance += (a + (motioncalc_t)0.5 * j * (motioncalc_t)duration) * pressureAdvanceClocks * (motioncalc_t)duration;
	a += j * pressureAdvanceClocks;
#else
	// Adjust the distance (and implicitly the initial speed) to account for pressure advance
	distance += a * pressureAdvanceClocksTimesDuration;
#endif

#if !SEGMENT_DEBUG
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Segments))
#endif
	{
#if SUPPORT_S_CURVE
		debugPrintf("Add seg: st=%" PRIu32 " t=%7" PRIu32 " dist=%9.3f u=%10.4e a=%10.4e j=%10.4e f=x%02" PRIx32 "\n",
					startTime, duration, (double)distance, (double)CalcInitialSpeed(duration, distance, a, j), (double)a, (double)j, moveFlags.all);
#else
		debugPrintf("Add seg: st=%" PRIu32 " t=%7" PRIu32 " dist=%9.3f u=%10.4e a=%10.4e f=x%02" PRIx32 "\n",
					startTime, duration, (double)distance, (double)CalcInitialSpeed(duration, distance, a), (double)a, moveFlags.all);
#endif
	}

	MoveSegment *_ecv_null prev = nullptr;
	MoveSegment *_ecv_null seg = list;

	// Loop until we find the earliest existing segment that the new one will come before (i.e. new one starts before existing one start) or will overlap (i.e. the new one starts before the existing segment ends)
	while (seg != nullptr)
	{
		int32_t offset = (int32_t)(startTime - seg->GetStartTime());			// how much later the segment we want to add starts after the existing one starts
		if (offset < 0)															// if the new segment starts before the existing one starts
		{
			if (offset + (int32_t)duration <= 0)
			{
				break;															// new segment fits entirely before the existing one
			}
#if AVOID_SHORT_SEGMENTS
			if (offset >= -MoveSegment::MinDuration && duration >= 10 * (uint32_t)MoveSegment::MinDuration)	// if it starts only slightly earlier and we can reasonably shorten it
			{
				startTime = seg->GetStartTime();								// then just delay and shorten the new segment slightly, to avoid creating a tiny segment
# if SEGMENT_DEBUG
				debugPrintf("Adjusting(1) t=%" PRIu32 " a=%.4e", duration, (double)a);
# endif
				duration = (uint32_t)((int32_t)duration + offset);
# if SEGMENT_DEBUG
				debugPrintf(" to t=%" PRIu32 " a=%.4e\n", duration, (double)a);
# endif
			}
			else																// new segment starts before the existing one and can't be delayed/shortened so that it doesn't
#endif
			{
				// Insert part of the new segment before the existing one, then merge the rest
				seg = MoveSegment::Allocate(seg);
				const uint32_t firstDuration = -offset;
				const motioncalc_t mFirstDuration = (motioncalc_t)firstDuration;
#if SUPPORT_S_CURVE
				const motioncalc_t firstDistance = (CalcInitialSpeed(duration, distance, a, j) + (OneHalf * a + OneSixth * j * mFirstDuration) * mFirstDuration) * mFirstDuration;
#else
				const motioncalc_t firstDistance = (CalcInitialSpeed(duration, distance, a) + OneHalf * a * mFirstDuration) * mFirstDuration;
#endif
				seg->SetParameters(startTime, firstDuration, firstDistance, a J_ACTUAL_PARAMETER(j), moveFlags);
#if SUPPORT_S_CURVE
				a += j * mFirstDuration;
#endif
				if (prev == nullptr)
				{
					list = _ecv_not_null(seg);
				}
				else
				{
					prev->SetNext(seg);
				}
#if CHECK_SEGMENTS
				CheckSegment(__LINE__, prev);
				CheckSegment(__LINE__, seg);
#endif
				duration -= firstDuration;
				startTime += firstDuration;
				distance -= firstDistance;
				prev = seg;
				seg = seg->GetNext();
				if (seg == nullptr)
				{
					break;
				}
			}
			offset = 0;
		}

		// At this point the new segment starts later or at the same time as the existing one (i.e. offset is non-negative)
		if (offset < (int32_t)seg->GetDuration())													// if new segment starts before the existing one ends
		{
#if AVOID_SHORT_SEGMENTS
			if (offset != 0 && offset + MoveSegment::MinDuration >= (int32_t)seg->GetDuration() && duration >= 10 * (uint32_t)MoveSegment::MinDuration)
			{
				// New segment starts just before the existing one ends, but we can delay and shorten it to start when the existing segment ends
# if SEGMENT_DEBUG
				debugPrintf("Adjusting(3) t=%" PRIu32 " a=%.4e", duration, (double)a);
# endif
				const uint32_t startDelay = seg->GetDuration() - (uint32_t)offset;
				startTime += startDelay;																	// postpone and shorten it a little
				duration -= startDelay;
# if SEGMENT_DEBUG
				debugPrintf(" to t=%" PRIu32 " a=%.4e\n", duration, (double)a);
# endif
				// Go round the loop again
			}
			else
#endif
			{
				// The new segment overlaps the existing one and can't be delayed so that it doesn't.
				// If the new segment starts later than the existing one does, split the existing one.
				if (offset != 0)
				{
					prev = seg;
					seg = seg->Split((uint32_t)offset);
#if CHECK_SEGMENTS
					CheckSegment(__LINE__, prev);
					CheckSegment(__LINE__, seg);
#endif
					offset = 0;
				}

				// The segment we wish to add now starts at the same time as 'seg' but it may end earlier or later than the one at 'seg' does.
				int32_t timeDifference = (int32_t)(duration - seg->GetDuration());
#if AVOID_SHORT_SEGMENTS
				if (timeDifference > 0 && timeDifference <= MoveSegment::MinDuration && duration >= 10 * (uint32_t)MoveSegment::MinDuration)
				{
					// New segment is slightly longer then the old one but it can be shortened
# if SEGMENT_DEBUG
					debugPrintf("Adjusting(3) t=%" PRIu32 " a=%.4e", duration, (double)a);
# endif
					duration -= (uint32_t)timeDifference;
# if SEGMENT_DEBUG
					debugPrintf(" to t=%" PRIu32 " a=%.4e\n", duration, (double)a);
# endif
					timeDifference = 0;
				}
#endif
				if (timeDifference > 0)
				{
					// The existing segment is shorter in time than the new one, so add the new segment in two or more parts
					const motioncalc_t segDuration = (motioncalc_t)seg->GetDuration();
#if SUPPORT_S_CURVE
					const motioncalc_t firstDistance = (CalcInitialSpeed(duration, distance, a, j) + (OneHalf * a + OneSixth * j * segDuration) * segDuration) * segDuration;	// distance moved by the first part of the new segment
#else
					const motioncalc_t firstDistance = (CalcInitialSpeed(duration, distance, a) + OneHalf * a * segDuration) * segDuration;		// distance moved by the first part of the new segment
#endif
#if SEGMENT_DEBUG
					debugPrintf("merge1: ");
#endif
					seg->Merge(firstDistance, a J_ACTUAL_PARAMETER(j), moveFlags);
#if SUPPORT_S_CURVE
					a += j * segDuration;
#endif
#if CHECK_SEGMENTS
					CheckSegment(__LINE__, prev);
					CheckSegment(__LINE__, seg);
#endif
					distance -= firstDistance;
					startTime += seg->GetDuration();
					duration = (uint32_t)timeDifference;
					// Now go round the loop again
				}
				else
				{
					// New segment ends earlier or at the same time as the old one
					if (timeDifference != 0)
					{
						// Split the existing segment in two
						seg->Split(duration);
#if CHECK_SEGMENTS
						CheckSegment(__LINE__, prev);
						CheckSegment(__LINE__, seg);
#endif
					}

					// The new segment and the existing one now have the same start time and duration, so merge them
#if SEGMENT_DEBUG
					debugPrintf("merge2: ");
#endif
					seg->Merge(distance, a J_ACTUAL_PARAMETER(j), moveFlags);
					goto finished;								// ugly but saves some code
				}
			}
		}

		prev = seg;
		seg = seg->GetNext();
	}

	// If we get here then the new segment (or what's left of it) needs to be added before 'seg' which may be null
	{
		MoveSegment *newSeg = MoveSegment::Allocate(seg);
		newSeg->SetParameters(startTime, duration, distance, a J_ACTUAL_PARAMETER(j), moveFlags);
		if (prev == nullptr)
		{
			list = newSeg;
		}
		else
		{
			prev->SetNext(newSeg);
		}
	}

finished:
#if CHECK_SEGMENTS
	CheckSegment(__LINE__, prev);
	CheckSegment(__LINE__, seg);
#endif
#if SEGMENT_DEBUG
	MoveSegment::DebugPrintList(segments);
#endif
	return list;
}

// Add some linear segments to be executed by a driver, taking account of possible input shaping. This is used by linear axes and by extruders.
// We never add a segment that starts earlier than the earliest existing segment (if any).
void Move::AddLinearSegments(size_t logicalDrive, uint32_t startTime, const PrepParams& params, motioncalc_t steps, MovementFlags moveFlags) noexcept
{
#if 0	//debug
	if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::Segments))
	{
//		debugPrintf("AddLin: st=%" PRIu32 " steps=%.1f\n", startTime, (double)steps);
//		params.DebugPrint();
	}
#endif

	DriveMovement& dm = dms[logicalDrive];
	MoveSegment *_ecv_null tail;

	// We need to ensure that while we are amending the segment list, the step ISR doesn't start executing a segment that we are amending.
	// We don't want to disable interrupts during the entire process of adding a segment, because that risks provoking hiccups when we re-enable interrupts and the ISR catches up with the overdue steps.
	// Instead we break off the tail of the segment chain containing the segments we need to change, re-enable interrupts, then modify that tail as needed. At the end we put the tail back.
	{
		MoveSegment *_ecv_null prev = nullptr;

		BasePriorityBooster booster(NvicPriorityStep);					// shut out the step interrupt

		tail = dm.segments;
		while (tail != nullptr)
		{
			const uint32_t segStartTime = tail->GetStartTime();
			const uint32_t endTime = segStartTime + tail->GetDuration();
			if ((int32_t)(startTime - endTime) < 0)										// if the segments we want to add start before this segment ends
			{
				if (tail->GetFlags().executing)
				{
					// Error, the segment we are trying to add overlaps an executing one
					const StringRef& dbgRef = Platform::genericDebugBuffer.GetRef();
					dbgRef.printf("Code 3 move error: new: start=%" PRIu32 " overlap=%" PRIu32 " time now=%" PRIu32 ", existing: ",
									startTime, segStartTime + tail->GetDuration() - startTime, StepTimer::GetMovementTimerTicks());
					tail->AppendDetails(dbgRef);
					dbgRef.cat('\n');
					Platform::shouldTurnOffHeaters = true;
					Platform::hasGenericDebug = true;
					StepErrorHalt();
					return;
				}

				if ((int32_t)(startTime - segStartTime) > 0)
				{
					// Split the existing segment
					prev = tail;
					tail = tail->Split(startTime - segStartTime);
					prev->SetNext(nullptr);
				}
				else
				{
					// Split just before this segment
					if (prev == nullptr)
					{
						dm.segments = nullptr;
					}
					else
					{
						prev->SetNext(nullptr);
					}
				}
				break;
			}

			prev = tail;
			tail = tail->GetNext();
		}
	}

	// Now it's safe to insert/merge new segments into 'tail'
#if SUPPORT_S_CURVE
	const uint32_t accelConstantStartTime = startTime + params.phaseClocks[0];
	const uint32_t accelEndStartTime = accelConstantStartTime + params.phaseClocks[1];
	const uint32_t steadyStartTime = accelEndStartTime + params.phaseClocks[2];
	const uint32_t decelStartTime = steadyStartTime + params.phaseClocks[3];
	const uint32_t decelConstantStartTime = decelStartTime + params.phaseClocks[4];
	const uint32_t decelEndStartTime = decelConstantStartTime + params.phaseClocks[5];
#else
	const uint32_t steadyStartTime = startTime + params.TotalAccelClocks();
	const uint32_t decelStartTime = steadyStartTime + params.steadyClocks;
#endif
	const motioncalc_t totalDistance = (motioncalc_t)params.totalDistance;
	const motioncalc_t stepsPerMm = steps/totalDistance;

	// Phases with zero duration will not get executed and may lead to infinities in the calculations. Avoid introducing them. Keep the total distance correct.
	// When using input shaping we can save some FP multiplications by multiplying the acceleration or deceleration time by the pressure advance just once instead of once per impulse
#if SUPPORT_S_CURVE
	motioncalc_t phase0PressureAdvanceClocks, phase1PressureAdvanceClocks, phase2PressureAdvanceClocks, phase4PressureAdvanceClocks, phase5PressureAdvanceClocks, phase6PressureAdvanceClocks;
	if (moveFlags.isExtruder && !moveFlags.nonPrintingMove)
	{
		params.EnsureSpeedsSet();				// calculate the intermediate speeds

		phase0PressureAdvanceClocks = (params.phaseClocks[0] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.startSpeed, params.phase1StartSpeed, steps);
		phase1PressureAdvanceClocks = (params.phaseClocks[1] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.phase1StartSpeed, params.phase1EndSpeed, steps);
		phase2PressureAdvanceClocks = (params.phaseClocks[2] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.phase1EndSpeed, params.topSpeed, steps);
		phase4PressureAdvanceClocks = (params.phaseClocks[4] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.phase5StartSpeed, params.topSpeed, steps);
		phase5PressureAdvanceClocks = (params.phaseClocks[5] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.phase5EndSpeed, params.phase5StartSpeed, steps);
		phase6PressureAdvanceClocks = (params.phaseClocks[6] == 0) ? (motioncalc_t)0.0 : dm.extruderShaper.GetAverageAdvanceClocks(params.endSpeed, params.phase5EndSpeed, steps);
	}
	else
	{
		phase0PressureAdvanceClocks = phase1PressureAdvanceClocks = phase2PressureAdvanceClocks = phase4PressureAdvanceClocks = phase5PressureAdvanceClocks = phase6PressureAdvanceClocks = (motioncalc_t)0.0;
	}
#else
	motioncalc_t accelDistance, accelPressureAdvance;
	if (params.accelClocks == 0)
	{
		accelDistance = (motioncalc_t)0.0;
		accelPressureAdvance = (motioncalc_t)0.0;
	}
	else
	{
		accelDistance = (params.decelClocks + params.steadyClocks == 0) ? totalDistance : (motioncalc_t)params.accelDistance;
		accelPressureAdvance = (moveFlags.isExtruder && !moveFlags.nonPrintingMove)
								? (motioncalc_t)params.accelClocks * dm.extruderShaper.GetAverageAdvanceClocks(params.startSpeed, params.topSpeed, steps)
								: (motioncalc_t)0.0;
	}

	motioncalc_t decelDistance, decelPressureAdvance;
	if (params.decelClocks == 0)
	{
		decelDistance = (motioncalc_t)0.0;
		decelPressureAdvance= (motioncalc_t)0.0;
	}
	else
	{
		decelDistance = totalDistance - ((params.steadyClocks == 0) ? accelDistance : (motioncalc_t)params.decelStartDistance);
		decelPressureAdvance = (moveFlags.isExtruder && !moveFlags.nonPrintingMove)
								? (motioncalc_t)params.decelClocks * dm.extruderShaper.GetAverageAdvanceClocks(params.endSpeed, params.topSpeed, steps)
								: (motioncalc_t)0.0;
	}

	const motioncalc_t steadyDistance = (params.steadyClocks == 0) ? (motioncalc_t)0.0 : totalDistance - accelDistance - decelDistance;
#endif

#if STEPS_DEBUG
	{
		AtomicCriticalSectionLocker lock;
		dm.positionRequested += steps;				// currently we compile for C++17 so we can't make this variable atomic
	}
#endif

	if (moveFlags.noShaping)
	{
#if SUPPORT_S_CURVE
		const motioncalc_t scaledJerk = params.jerk * stepsPerMm;
		if (params.phaseClocks[0] != 0)
		{
			tail = AddSegment(tail, startTime, params.phaseClocks[0], params.distances[0] * stepsPerMm, params.initialAcceleration * stepsPerMm, scaledJerk, moveFlags, phase0PressureAdvanceClocks);
		}
		if (params.phaseClocks[1] != 0)
		{
			tail = AddSegment(tail, accelConstantStartTime, params.phaseClocks[1], params.distances[1] * stepsPerMm, params.peakAcceleration * stepsPerMm, (motioncalc_t)0.0, moveFlags, phase1PressureAdvanceClocks);
		}
		if (params.phaseClocks[2] != 0)
		{
			tail = AddSegment(tail, accelEndStartTime, params.phaseClocks[2], params.distances[2] * stepsPerMm, params.peakAcceleration * stepsPerMm, -scaledJerk, moveFlags, phase2PressureAdvanceClocks);
		}
		if (params.phaseClocks[3] != 0)
		{
			tail = AddSegment(tail, steadyStartTime, params.phaseClocks[3], params.distances[3] * stepsPerMm, (motioncalc_t)0.0, (motioncalc_t)0.0, moveFlags, (motioncalc_t)0.0);
		}
		if (params.phaseClocks[4] != 0)
		{
			tail = AddSegment(tail, decelStartTime, params.phaseClocks[4], params.distances[4] * stepsPerMm, params.initialDeceleration * stepsPerMm, -scaledJerk, moveFlags, phase4PressureAdvanceClocks);
		}
		if (params.phaseClocks[5] != 0)
		{
			tail = AddSegment(tail, decelConstantStartTime, params.phaseClocks[5], params.distances[5] * stepsPerMm, params.peakDeceleration * stepsPerMm, (motioncalc_t)0.0, moveFlags, phase5PressureAdvanceClocks);
		}
		if (params.phaseClocks[6] != 0)
		{
			tail = AddSegment(tail, decelEndStartTime, params.phaseClocks[6], params.distances[6] * stepsPerMm, params.peakDeceleration * stepsPerMm, scaledJerk, moveFlags, phase6PressureAdvanceClocks);
		}
#else
		if (params.accelClocks != 0)
		{
			tail = AddSegment(tail, startTime, params.accelClocks, accelDistance * stepsPerMm, params.acceleration * stepsPerMm, moveFlags, accelPressureAdvance);
		}
		if (params.steadyClocks != 0)
		{
			tail = AddSegment(tail, steadyStartTime, params.steadyClocks, steadyDistance * stepsPerMm, (motioncalc_t)0.0, moveFlags, (motioncalc_t)0.0);
		}
		if (params.decelClocks != 0)
		{
			tail = AddSegment(tail, decelStartTime, params.decelClocks, decelDistance * stepsPerMm, params.deceleration * stepsPerMm, moveFlags, decelPressureAdvance);
		}
#endif
	}
	else
	{
		for (size_t index = 0; index < axisShaper.GetNumImpulses(); ++index)
		{
			const motioncalc_t factor = axisShaper.GetImpulseSize(index) * stepsPerMm;
			const uint32_t startDelay = axisShaper.GetImpulseDelay(index);
#if SUPPORT_S_CURVE
			const motioncalc_t scaledJerk = params.jerk * factor;
			if (params.phaseClocks[0] != 0)
			{
				tail = AddSegment(tail, startTime + startDelay, params.phaseClocks[0], params.distances[0] * factor, params.initialAcceleration * factor, scaledJerk, moveFlags, phase0PressureAdvanceClocks);
			}
			if (params.phaseClocks[1] != 0)
			{
				tail = AddSegment(tail, accelConstantStartTime + startDelay, params.phaseClocks[1], params.distances[1] * factor, params.peakAcceleration * factor, (motioncalc_t)0.0, moveFlags, phase1PressureAdvanceClocks);
			}
			if (params.phaseClocks[2] != 0)
			{
				tail = AddSegment(tail, accelEndStartTime + startDelay, params.phaseClocks[2], params.distances[2] * factor, params.peakAcceleration * factor, -scaledJerk, moveFlags, phase2PressureAdvanceClocks);
			}
			if (params.phaseClocks[3] != 0)
			{
				tail = AddSegment(tail, steadyStartTime + startDelay, params.phaseClocks[3], params.distances[3] * factor, (motioncalc_t)0.0, (motioncalc_t)0.0, moveFlags, (motioncalc_t)0.0);
			}
			if (params.phaseClocks[4] != 0)
			{
				tail = AddSegment(tail, decelStartTime + startDelay, params.phaseClocks[4], params.distances[4] * factor, params.initialDeceleration * factor, -scaledJerk, moveFlags, phase4PressureAdvanceClocks);
			}
			if (params.phaseClocks[5] != 0)
			{
				tail = AddSegment(tail, decelConstantStartTime + startDelay, params.phaseClocks[5], params.distances[5] * factor, params.peakDeceleration * factor, (motioncalc_t)0.0, moveFlags, phase5PressureAdvanceClocks);
			}
			if (params.phaseClocks[6] != 0)
			{
				tail = AddSegment(tail, decelEndStartTime + startDelay, params.phaseClocks[6], params.distances[6] * factor, params.peakDeceleration * factor, scaledJerk, moveFlags, phase6PressureAdvanceClocks);
			}
#else
			if (params.accelClocks != 0)
			{
				tail = AddSegment(tail, startTime + startDelay, params.accelClocks, accelDistance * factor, params.acceleration * factor, moveFlags, accelPressureAdvance);
			}
			if (params.steadyClocks != 0)
			{
				tail = AddSegment(tail, steadyStartTime + startDelay, params.steadyClocks, steadyDistance * factor, (motioncalc_t)0.0, moveFlags, (motioncalc_t)0.0);
			}
			if (params.decelClocks != 0)
			{
				tail = AddSegment(tail, decelStartTime + startDelay, params.decelClocks, decelDistance * factor, params.deceleration * factor, moveFlags, decelPressureAdvance);
			}
#endif
		}
	}

	// If there were no segments attached to this DM initially, we need to schedule the interrupt for the new segment at the start of the list.
	// Don't do this until we have added all the segments for this move, because the first segment we added may have been modified and/or split when we added further segments to implement input shaping
	{
		BasePriorityBooster booster(NvicPriorityStep);								// shut out the step interrupt

		// Join the tail back to the end of the segment list
		{
			MoveSegment *_ecv_null ms = dm.segments;
			if (ms == nullptr)
			{
				dm.segments = tail;
				dm.positionAtMoveStart = dm.currentMotorPosition;						// record the start-of-motion position in case we are checking endstops
#if SUPPORT_PHASE_STEPPING
				dm.phaseStepsTakenSinceMoveStart = (motioncalc_t)0.0;
#endif
			}
			else
			{
				while (ms->GetNext() != nullptr)
				{
					ms = ms->GetNext();
				}
				ms->SetNext(tail);
			}
		}

		if (dm.state == DMState::idle)													// if the DM has no segments
		{
			if (dm.ScheduleFirstSegment())
			{
#if SUPPORT_PHASE_STEPPING
				if (dm.state != DMState::phaseStepping)
#endif
				{
					// Always set the direction when starting the first move
					dm.directionChanged = false;
					SetDirection(dm.drive, dm.direction);
				}
				InsertDM(&dm);
				if (activeDMs == &dm && simulationMode == SimulationMode::off)			// if this is now the first DM in the active list
				{
					if (ScheduleNextStepInterrupt())
					{
						Interrupt();
					}
				}
			}
		}
	}		// End of boosted base priority section
}

// Return true if none of the drives passed has any movement pending
bool Move::AreDrivesStopped(LogicalDrivesBitmap drives) const noexcept
{
	return drives.IterateWhile([this](unsigned int drive, unsigned int index) noexcept -> bool
								{
									return dms[drive].segments == nullptr;
								}
							  );
}

#if HAS_STALL_DETECT

void Move::CheckStallDetectionViable(uint8_t localDriver, float speed) const THROWS(GCodeException)
{
	const char *_ecv_array _ecv_null msg = SmartDrivers::CheckStallDetectionEnabled(localDriver, fabsf(speed));
	if (msg != nullptr)
	{
		ThrowGCodeException(msg, localDriver);
	}
}

#endif

#if SUPPORT_CLOSED_LOOP

// if the driver is idle, enable it; return true if driver enabled on return
bool Move::EnableIfIdle(size_t driver) noexcept
{
#if 0
	if (driverStates[driver] == DriverStateControl::driverIdle)
	{
		driverStates[driver] = DriverStateControl::driverActive;
#if HAS_SMART_DRIVERS
		driverAtIdleCurrent[driver] = false;
		UpdateMotorCurrent(driver);
#endif
	}

	return driverStates[driver] == DriverStateControl::driverActive;
#else
	return false;
#endif
}

#endif

#if SUPPORT_PHASE_STEPPING

void Move::ConfigurePhaseStepping(size_t axisOrExtruder, float value, PhaseStepConfig config)
{
	switch (config)
	{
	default:
		break;
	case PhaseStepConfig::kv:
		dms[axisOrExtruder].phaseStepControl.SetKv(value);
		break;
	case PhaseStepConfig::ka:
		dms[axisOrExtruder].phaseStepControl.SetKa(value);
		break;
	}
}

PhaseStepParams Move::GetPhaseStepParams(size_t axisOrExtruder) const noexcept
{
	PhaseStepParams params;
	params.Kv = dms[axisOrExtruder].phaseStepControl.GetKv();
	params.Ka = dms[axisOrExtruder].phaseStepControl.GetKa();
	return params;
}

// Get the motor position in the current move so far, also speed and acceleration. Units are full steps and step clocks.
// segments might be updated
bool Move::UpdateCurrentMotion(size_t driver, uint32_t when, MotionParameters& mParams) noexcept
{
	const bool ret = dms[driver].UpdateCurrentMotion(when, mParams);
	mParams.Scale(phaseStepMultiplier[driver]);
	return ret;
}

bool Move::SetStepMode(size_t axisOrExtruder, StepMode mode, const StringRef& reply) noexcept
{
	bool hasRemoteDrivers = false;
	IterateRemoteDrivers(axisOrExtruder, [&hasRemoteDrivers](DriverId driver) noexcept -> void { hasRemoteDrivers = true; });

	// Phase stepping does not support remote drivers
	if (hasRemoteDrivers && mode == StepMode::phase)
	{
		return false;
	}

	bool ret = true;
	DriveMovement* dm = &dms[axisOrExtruder];
	const uint32_t now = StepTimer::GetTimerTicks();

	bool interpolation;
	unsigned int microsteps = GetMicrostepping(axisOrExtruder, interpolation);
	UpdateCurrentMotion(axisOrExtruder, now, dm->phaseStepControl.mParams); // Update position variable

	IterateLocalDrivers(axisOrExtruder, [this, dm, &ret, &mode, axisOrExtruder, microsteps](uint8_t driver) noexcept -> void {
		// If we are going from step dir to phase step, we need to update the phase offset so the calculated phase matches MSCNT
		if (!SmartDrivers::IsPhaseSteppingEnabled(driver) && mode == StepMode::phase)
		{
			dm->phaseStepControl.SetPhaseOffset(driver, 0);												// Reset offset
			const uint16_t initialPhase = SmartDrivers::GetMicrostepPosition(driver) * 4;				// Get MSCNT
			const uint16_t calculatedPhase = dm->phaseStepControl.CalculateStepPhase(driver);			// Get the phase based on current machine position

			dm->phaseStepControl.SetPhaseOffset(driver, (initialPhase - calculatedPhase) % 4096u);		// Update the offset so calculated phase equals MSCNT
			dm->phaseStepControl.SetMotorPhase(driver, initialPhase, 1.0);								// Update XDIRECT register with new phase values
		}
		// If we are going from phase step to step dir, we need to send some fake steps to the driver to update MSCNT to avoid a jitter when disabling direct_mode
		// This is suboptimal but it is a configuration command that is unlikely to be run so a few ms delay is unlikely to cause much harm.
		// If the delay is an issue then all the drivers for the axis could be stepped together and each loop check if each drivers MSCNT has reached the target.
		else if (SmartDrivers::IsPhaseSteppingEnabled(driver) && mode == StepMode::stepDir)
		{
			const uint16_t targetPhase = dm->phaseStepControl.CalculateStepPhase(driver) / 4;
			uint16_t mscnt = SmartDrivers::GetMicrostepPosition(driver);
			int16_t steps = ((int16_t)mscnt - (int16_t)targetPhase) / (int)(256 / microsteps);
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PhaseStep))
			{
				debugPrintf("dms[%u]: mscnt=%u, targetPhase=%u, steps=%d", axisOrExtruder, mscnt, targetPhase, steps);
			}

			bool d = digitalRead(DIRECTION_PINS[driver]);
			if (steps < 0)
			{
				digitalWrite(DIRECTION_PINS[driver], false);
			}
			else
			{
				digitalWrite(DIRECTION_PINS[driver], true);
			}

			steps = (int16_t)abs((int)steps);

			while (steps > 0)
			{
				StepPins::StepDriversHigh(StepPins::CalcDriverBitmap(driver));	// step drivers high
				delayMicroseconds(20);
# if SAME70
				__DSB();														// without this the step pulse can be far too short
# endif
				StepPins::StepDriversLow(StepPins::CalcDriverBitmap(driver));	// step drivers low
				delayMicroseconds(20);
				steps--;
			}

			digitalWrite(DIRECTION_PINS[driver], d);

			delay(10);															// Give enough time for MSCNT to be read
			if (reprap.GetDebugFlags(Module::Move).IsBitSet(MoveDebugFlags::PhaseStep))
			{
				debugPrintf(", new mscnt=%u\n", SmartDrivers::GetMicrostepPosition(driver));
			}
		}

		if (!SmartDrivers::EnablePhaseStepping(driver, mode == StepMode::phase))
		{
			ret = false;
		}
	});
	dms[axisOrExtruder].SetStepMode(mode);

	ResetPhaseStepMonitoringVariables();
	return ret;
}

StepMode Move::GetStepMode(size_t axisOrExtruder) const noexcept
{
	if (axisOrExtruder >= MaxAxesPlusExtruders)
	{
		return StepMode::unknown;
	}
	return dms[axisOrExtruder].GetStepMode();
}

void Move::PhaseStepControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevPSControlLoopCallTime;
	prevPSControlLoopCallTime = loopCallTime;
	if (timeElapsed < minPSControlLoopCallInterval) { minPSControlLoopCallInterval = timeElapsed; }
	if (timeElapsed > maxPSControlLoopCallInterval) { maxPSControlLoopCallInterval = timeElapsed; }

	const uint32_t now = StepTimer::ConvertLocalToMovementTime(loopCallTime);
	MovementFlags flags;
	flags.Clear();

	{
		const DriveMovement *_ecv_null dm = phaseStepDMs;
		while (dm != nullptr)
		{
			if (dm->state > DMState::starting)
			{
				flags |= dm->segmentFlags;
			}
			dm = dm->nextDM;
		}
	}

	if (flags.checkEndstops)
	{
		CheckEndstops(true);												// call out to a separate function because this may help cache locality in the more common and time-critical case where we don't call it
	}

	bool inserted = false;
	DriveMovement *_ecv_null *dmp = &phaseStepDMs;
	while (*dmp != nullptr)
	{
		DriveMovement * const dm = _ecv_not_null(*dmp);
		UpdateCurrentMotion(dm->drive, now, dm->phaseStepControl.mParams);

		if (dm->state != DMState::phaseStepping)
		{
			*dmp = dm->nextDM;
			if (dm->state >= DMState::firstMotionState)
			{
				InsertDM(dm);
				if (activeDMs == dm)
				{
					inserted = true;										// we have scheduled a new segment which isn't ready to start, so we need an interrupt
				}
			}
		}
		else
		{
			dm->phaseStepControl.CalculateCurrentFraction();

			IterateLocalDrivers(dm->drive, [dm](uint8_t driver) noexcept -> void {
				if ((dm->driversCurrentlyUsed & StepPins::CalcDriverBitmap(driver)) == 0)
				{
					if (likely(dm->state > DMState::starting))
					{
						// Driver has been stopped (probably by Move::CheckEndstops() so we don't need to update it)
						dm->phaseStepControl.UpdatePhaseOffset(driver);
					}
					return;
				}
				dm->phaseStepControl.InstanceControlLoop(driver);
			});
			dmp = &(dm->nextDM);
		}
	}

	if (inserted)
	{
		BasePriorityBooster booster(NvicPriorityStep);					// shut out the step interrupt
		if (!ScheduleNextStepInterrupt())
		{
			Interrupt();
		}
	}

	// Record how long this has taken to run
	const StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	if (loopRuntime < minPSControlLoopRuntime) { minPSControlLoopRuntime = loopRuntime; }
	if (loopRuntime > maxPSControlLoopRuntime) { maxPSControlLoopRuntime = loopRuntime; }
}


// Helper function to reset the 'monitoring variables' as defined above
void Move::ResetPhaseStepMonitoringVariables() noexcept
{
	minPSControlLoopRuntime = std::numeric_limits<StepTimer::Ticks>::max();
	maxPSControlLoopRuntime = 1;
	minPSControlLoopCallInterval = std::numeric_limits<StepTimer::Ticks>::max();
	maxPSControlLoopCallInterval = 1;
}

#endif

// ISR for the step interrupt
void Move::Interrupt() noexcept
{
	if (activeDMs != nullptr)
	{
		uint32_t now = StepTimer::GetMovementTimerTicks();
		const uint32_t isrStartTime = now;
#if 0	// TEMP DEBUG - see later
		for (unsigned int iterationCount = 0; ; )
#else
		for (;;)
#endif
		{
			// Generate steps for the current move segments
			StepDrivers(now);									// check endstops if necessary and step the drivers

			if (activeDMs == nullptr || hadStepError)
			{
				WakeMoveTask();					// we may have just completed a special move, so wake up the Move task so that it can notice that
				break;
			}

			// Schedule a callback at the time when the next step is due, and quit unless it is due immediately
			if (!ScheduleNextStepInterrupt())
			{
				break;
			}

			// The next step is due immediately. Check whether we have been in this ISR for too long already and need to take a break
			now = StepTimer::GetMovementTimerTicks();
			const int32_t clocksTaken = (int32_t)(now - isrStartTime);
			if (clocksTaken >= (int32_t)MoveTiming::MaxStepInterruptTime)
			{
				// Force a break by updating the move start time.
				++numInterruptHiccups;
#if SUPPORT_CAN_EXPANSION
				uint32_t hiccupTimeInserted = 0;
#endif
				for (uint32_t hiccupTime = MoveTiming::HiccupTime; ; hiccupTime += MoveTiming::HiccupIncrement)
				{
#if SUPPORT_CAN_EXPANSION
					hiccupTimeInserted += hiccupTime;
#endif
					StepTimer::IncreaseMovementDelay(hiccupTime);

					// Reschedule the next step interrupt. This time it should succeed if the hiccup time was long enough.
					if (!ScheduleNextStepInterrupt())
					{
#if 0	//TEMP DEBUG
# if SUPPORT_CAN_EXPANSION
						debugPrintf("Add hiccup %" PRIu32 ", ic=%u, now=%" PRIu32 "\n", hiccupTimeInserted, iterationCount, now);
# else
						debugPrintf("Add hiccup, ic=%u, now=%" PRIu32 "\n", iterationCount, now);
# endif
						activeDMs->DebugPrint();
						MoveSegment::DebugPrintList(activeDMs->segments);
						if (activeDMs->nextDM != nullptr)
						{
							activeDMs->nextDM ->DebugPrint();
							MoveSegment::DebugPrintList(activeDMs->nextDM->segments);

						}
						//END DEBUG
#endif
						return;
					}
					// The hiccup wasn't long enough, so go round the loop again
				}
			}
		}
	}
}

// Move timer callback function
/*static*/ void Move::TimerCallback(CallbackParameter p) noexcept
{
	static_cast<Move*>(p.vp)->Interrupt();
}

// Remove this drive from the list of drives with steps due and put it in the completed list
// Called from the step ISR only.
void Move::DeactivateDM(DriveMovement *dmToRemove) noexcept
{
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	DriveMovement *_ecv_null * dmp = dmToRemove->state == DMState::phaseStepping ? &phaseStepDMs : &activeDMs;
#else
	DriveMovement *_ecv_null * dmp = &activeDMs;
#endif
	while (*dmp != nullptr)
	{
		DriveMovement * const dm = _ecv_not_null(*dmp);
		if (dm == dmToRemove)
		{
			(*dmp) = dm->nextDM;
			dm->state = DMState::idle;
			break;
		}
		dmp = &(dm->nextDM);
	}
}

// Check the endstops, given that we know that this move checks endstops.
// This may be called both from the step ISR and from the CanReceive task.
// If executingMove is set then the move is already being executed; otherwise we are preparing to commit the move.
#if SUPPORT_CAN_EXPANSION
// Returns true if the caller needs to wake the async sender task because CAN-connected drivers need to be stopped
bool Move::CheckEndstops(bool executingMove) noexcept
#else
void Move::CheckEndstops(bool executingMove) noexcept
#endif
{
#if SUPPORT_CAN_EXPANSION
	bool wakeAsyncSender = false;
#endif
	EndstopsManager& emgr = reprap.GetPlatform().GetEndstops();
	while (true)
	{
		const EndstopHitDetails hitDetails = emgr.CheckEndstops();

		switch (hitDetails.GetAction())
		{
		case EndstopHitAction::stopAll:
#if SUPPORT_CAN_EXPANSION
			if (StopAllDrivers(executingMove, hitDetails.whenTriggered)) { wakeAsyncSender = true; }
#else
			StopAllDrivers(executingMove);
#endif
			if (hitDetails.isZProbe)
			{
				reprap.GetGCodes().MoveStoppedByZProbe();
			}
			else
			{
				reprap.GetGCodes().RecordEndstopTriggered(hitDetails.axis, kinematics->GetHomingMode());
			}

			if (executingMove)
			{
				WakeMoveTask();			// wake move task so that it sets the move as finished promptly
			}
#if SUPPORT_CAN_EXPANSION
			return wakeAsyncSender;
#else
			return;
#endif

		case EndstopHitAction::stopAxis:
			// We must stop the drive before we mess with its coordinates
#if SUPPORT_CAN_EXPANSION
			if (StopAxisOrExtruder(executingMove, hitDetails.axis, hitDetails.whenTriggered)) { wakeAsyncSender = true; }
#else
			StopAxisOrExtruder(executingMove, hitDetails.axis);
#endif
			reprap.GetGCodes().RecordEndstopTriggered(hitDetails.axis, kinematics->GetHomingMode());

			if (executingMove && !emgr.AnyEndstopsActive())
			{
				WakeMoveTask();			// wake move task so that it sets the move as finished promptly
			}
			break;

		case EndstopHitAction::stopDriver:
#if SUPPORT_CAN_EXPANSION
			if (hitDetails.driver.IsRemote())
			{
				if (executingMove)
				{
					DriveMovement& dm = dms[hitDetails.axis];
					if (dm.state >= DMState::firstMotionState)
					{
						if (CanMotion::StopDriverWhenExecuting(hitDetails.driver, dm.GetNetStepsTakenThisMove(hitDetails.whenTriggered)))
						{
							wakeAsyncSender = true;
						}
					}
				}
				else
				{
					CanMotion::StopDriverWhenProvisional(hitDetails.driver);
				}
			}
			else
#endif
			{
				const size_t localDriver = hitDetails.driver.localDriver;
				dms[hitDetails.axis].driversCurrentlyUsed &= ~StepPins::CalcDriverBitmap(localDriver);
				if (!executingMove)
				{
					dms[hitDetails.axis].driverEndstopsTriggeredAtStart |= StepPins::CalcDriverBitmap(localDriver);
				}
			}
			break;

		default:
#if SUPPORT_CAN_EXPANSION
			return wakeAsyncSender;
#else
			return;
#endif
		}
	}
}

// Generate the step pulses of internal drivers used by this DDA
// Note, we use the movement timer ticks to decide when to generate step pulses, but we must use the raw step timer to enforce delays between pulses.
// 'now'is the movement timer ticks
void Move::StepDrivers(uint32_t now) noexcept
{
	uint32_t driversStepping = 0;
	MovementFlags flags;
	flags.Clear();
	DriveMovement *_ecv_null dm = activeDMs;
	while (dm != nullptr && (int32_t)(dm->nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)		// if the next step is due
	{
		driversStepping |= dm->driversCurrentlyUsed;
		flags |= dm->segmentFlags;
		dm = dm->nextDM;
	}

	if (flags.checkEndstops)
	{
#if SUPPORT_CAN_EXPANSION
		if (CheckEndstops(true)) { CanInterface::WakeAsyncSender(); }
#else
		CheckEndstops(true);												// call out to a separate function because this may help cache locality in the more common and time-critical case where we don't call it
#endif

		// Calling CheckEndstops may have removed DMs from the active list, also it takes time; so re-check which drives need steps
		driversStepping = 0;
		dm = activeDMs;
		now = StepTimer::GetMovementTimerTicks();
		while (dm != nullptr && (int32_t)(dm->nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)	// if the next step is due
		{
			driversStepping |= dm->driversCurrentlyUsed;
			dm = dm->nextDM;
		}
	}

#ifdef DUET3_MB6XD
	if (driversStepping != 0)
	{
		// Wait until step low and direction setup time have elapsed
		const uint32_t locLastStepPulseTime = lastStepHighTime;
		const uint32_t locLastDirChangeTime = lastDirChangeTime;
		uint32_t rawNow;
		do
		{
			rawNow = StepTimer::GetTimerTicks();
		} while (rawNow - locLastStepPulseTime < GetSlowDriverStepPeriodClocks() || rawNow - locLastDirChangeTime < GetSlowDriverDirSetupClocks());

		StepPins::StepDriversLow(StepPins::AllDriversBitmap & (~driversStepping));		// disable the step pins of the drivers we don't want to step
		StepPins::StepDriversHigh(driversStepping);										// set up the drivers that we do want to step

		// Trigger the TC so that it generates a step pulse
		STEP_GATE_TC->TC_CHANNEL[STEP_GATE_TC_CHAN].TC_CCR = TC_CCR_SWTRG;
		lastStepHighTime = StepTimer::GetTimerTicks();
	}

	// Calculate the next step times. We must do this even if no local drivers are stepping in case endstops or Z probes are active.
	PrepareForNextSteps(dm, flags, now);
#else
# if SUPPORT_SLOW_DRIVERS											// if supporting slow drivers
	if ((driversStepping & slowDriversBitmap) != 0)					// if using some slow drivers
	{
		// Wait until step low and direction setup time have elapsed
		uint32_t lastStepPulseTime = lastStepLowTime;
		uint32_t rawNow;
		do
		{
			rawNow = StepTimer::GetTimerTicks();
		} while (rawNow - lastStepPulseTime < GetSlowDriverStepLowClocks() || rawNow - lastDirChangeTime < GetSlowDriverDirSetupClocks());

		StepPins::StepDriversHigh(driversStepping);					// step drivers high
		lastStepPulseTime = StepTimer::GetTimerTicks();

		PrepareForNextSteps(dm, flags, now);

		while (StepTimer::GetTimerTicks() - lastStepPulseTime < GetSlowDriverStepHighClocks()) {}
		StepPins::StepDriversLow(driversStepping);					// step drivers low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}
	else
# endif
	{
		StepPins::StepDriversHigh(driversStepping);					// step drivers high
# if SAME70
		__DSB();													// without this the step pulse can be far too short
# endif
		PrepareForNextSteps(dm, flags, now);
		StepPins::StepDriversLow(driversStepping);					// step drivers low
	}
#endif

	// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
	DriveMovement *_ecv_null dmToInsert = activeDMs;				// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		DriveMovement *_ecv_null const nextToInsert = dmToInsert->nextDM;
		if (dmToInsert->state >= DMState::firstMotionState)
		{
			if (dmToInsert->directionChanged)
			{
				dmToInsert->directionChanged = false;
				SetDirection(dmToInsert->drive, dmToInsert->direction);
			}
			InsertDM(dmToInsert);
		}
		dmToInsert = nextToInsert;
	}
}

// Prepare each DM that we generated a step for for the next step
// This is called only by the step ISR
void Move::PrepareForNextSteps(DriveMovement *stopDm, MovementFlags flags, uint32_t now) noexcept
{
	for (DriveMovement *_ecv_null dm2 = activeDMs; dm2 != stopDm; dm2 = dm2->nextDM)
	{
		if (unlikely(dm2->state == DMState::starting))
		{
			if (dm2->NewSegment(now) != nullptr && dm2->state != DMState::starting)
			{
				dm2->driversCurrentlyUsed = dm2->driversNormallyUsed & ~dm2->driverEndstopsTriggeredAtStart;	// we previously set driversCurrentlyUsed to 0 to avoid generating a step, so restore it now
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
				if (dm2->state != DMState::phaseStepping)				// if we are phase stepping, skip the rest and proceed to the next DM
#endif
				{
# if SUPPORT_CAN_EXPANSION
					flags |= dm2->segmentFlags;
					if (unlikely(!flags.checkEndstops && dm2->driversNormallyUsed == 0))
					{
						dm2->TakeStepsAndCalcStepTimeRarely(now);
					}
					else
# endif
					{
						(void)dm2->CalcNextStepTimeFull(now); 			// calculate next step time
						dm2->directionChanged = true;					// force the direction to be set up
					}
				}
			}
		}
# if SUPPORT_CAN_EXPANSION
		else if (unlikely(!flags.checkEndstops && dm2->driversNormallyUsed == 0))
		{
			dm2->TakeStepsAndCalcStepTimeRarely(now);
		}
# endif
		else
		{
			(void)dm2->CalcNextStepTime(now);						// calculate next step time, which may change the required direction
		}
	}
}

void Move::SetDirection(size_t axisOrExtruder, bool direction) noexcept
{
#ifdef DUET3_MB6XD
	while (StepTimer::GetTimerTicks() - lastStepHighTime < GetSlowDriverDirHoldClocksFromLeadingEdge()) { }
#else
	const bool isSlowDriver = (dms[axisOrExtruder].driversNormallyUsed & slowDriversBitmap) != 0;
	if (isSlowDriver)
	{
		while (StepTimer::GetTimerTicks() - lastStepLowTime < GetSlowDriverDirHoldClocksFromTrailingEdge()) { }
	}
#endif

	SetDriversDirection(axisOrExtruder, direction);

#ifndef DUET3_MB6XD
	if (isSlowDriver)
#endif
	{
		lastDirChangeTime = StepTimer::GetTimerTicks();
	}
}
// Simulate stepping the drivers, for debugging.
// This is basically a copy of StepDrivers except that instead of being called from the timer ISR and generating steps,
// it is called from the Move task and outputs info on the step timings. It ignores endstops.
void Move::SimulateSteppingDrivers(Platform& p) noexcept
{
	static uint32_t lastStepTime;
	static bool checkTiming = false;
	static uint8_t lastDrive = 0;

	DriveMovement *_ecv_null dm = activeDMs;
	if (dm != nullptr)
	{
		// Generating and sending the debug output can take a lot of time, so to avoid shutting out high priority tasks, reduce our priority
		const unsigned int oldPriority = TaskBase::GetCurrentTaskPriority();
		TaskBase::SetCurrentTaskPriority(TaskPriority::SpinPriority);
		const uint32_t dueTime = dm->nextStepTime;
		while (dm != nullptr && (int32_t)(dueTime >= dm->nextStepTime) >= 0)			// if the next step is due
		{
			uint32_t timeDiff;
			const bool badTiming = checkTiming && dm->drive == lastDrive && ((timeDiff = dm->nextStepTime - lastStepTime) < 10 || timeDiff > 100000000);
			if (dm->nextStep == 1)
			{
				dm->DebugPrint();
				MoveSegment::DebugPrintList(dm->segments);
			}
#if 1
			if (badTiming || ((uint32_t)dm->nextStep & 255u) == 1u || dm->nextStep + 1 == dm->segmentStepLimit)
#endif
			{
				debugPrintf("%10" PRIu32 " D%u %c ns=%" PRIi32 "%s", dm->nextStepTime, dm->drive, (dm->direction) ? 'F' : 'B', dm->nextStep, (badTiming) ? " *\n" : "\n");
			}
			lastDrive = dm->drive;
			dm = dm->nextDM;
		}
		lastStepTime = dueTime;
		checkTiming = true;

		for (DriveMovement *_ecv_null dm2 = activeDMs; dm2 != dm; dm2 = dm2->nextDM)
		{
			if (unlikely(dm2->state == DMState::starting))
			{
				if (dm2->NewSegment(dueTime) != nullptr && dm2->state != DMState::starting)
				{
					(void)dm2->CalcNextStepTime(dueTime);				// calculate next step time
				}
			}
			else
			{
				(void)dm2->CalcNextStepTime(dueTime);					// calculate next step time
			}
		}

		// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
		DriveMovement *_ecv_null dmToInsert = activeDMs;							// head of the chain we need to re-insert
		activeDMs = dm;													// remove the chain from the list
		while (dmToInsert != dm)										// note that both of these may be nullptr
		{
			DriveMovement *_ecv_null const nextToInsert = dmToInsert->nextDM;
			if (dmToInsert->state >= DMState::firstMotionState)
			{
				dmToInsert->directionChanged = false;
				InsertDM(dmToInsert);
			}
			dmToInsert = nextToInsert;
		}
		TaskBase::SetCurrentTaskPriority(oldPriority);
	}

	if (activeDMs == nullptr)
	{
		checkTiming = false;		// don't check the timing of the first step in the next move
	}
}

// This is called when we abort a move because we have hit an endstop.
// It stops all drives and adjusts the end points of the current move to account for how far through the move we got.
bool Move::StopAllDrivers(bool executingMove, uint32_t when) noexcept
{
	bool wakeAsyncSender = false;
	for (size_t drive = 0; drive < MaxAxesPlusExtruders; ++drive)
	{
		if (StopAxisOrExtruder(executingMove, drive, when)) { wakeAsyncSender = true; }
	}
	return wakeAsyncSender;
}

// Stop a drive and re-calculate the end position. Return true if any remote drivers were scheduled to be stopped.
bool Move::StopAxisOrExtruder(bool executingMove, size_t logicalDrive, uint32_t when) noexcept
{
	DriveMovement& dm = dms[logicalDrive];
	int32_t netStepsTaken;
	const bool wasMoving = dm.StopLogicalDrive(netStepsTaken, when);
	bool wakeAsyncSender = false;
#if SUPPORT_CAN_EXPANSION
	if (wasMoving)
	{
		IterateDrivers(logicalDrive,
						[](uint8_t) noexcept -> void { },						// no action if the driver is local
						[executingMove, netStepsTaken, &wakeAsyncSender](DriverId did) noexcept -> void
							{
								if (executingMove)
								{
									if (CanMotion::StopDriverWhenExecuting(did, netStepsTaken)) { wakeAsyncSender = true; }
								}
								else
								{
									CanMotion::StopDriverWhenProvisional(did);
								}
							}
					  );
	}
#else
	(void)wasMoving;
#endif

	return wakeAsyncSender;
}

#if SUPPORT_CAN_EXPANSION

// Function to identify and iterate through all drivers attached to an axis or extruder
void Move::IterateDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> localFunc, function_ref_noexcept<void(DriverId) noexcept> remoteFunc) noexcept
{
	if (axisOrExtruder < reprap.GetGCodes().GetTotalAxes())
	{
		for (size_t i = 0; i < axisDrivers[axisOrExtruder].numDrivers; ++i)
		{
			const DriverId id = axisDrivers[axisOrExtruder].driverNumbers[i];
			if (id.IsLocal())
			{
				localFunc(id.localDriver);
			}
			else
			{
				remoteFunc(id);
			}
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		if (LogicalDriveToExtruder(axisOrExtruder) < reprap.GetGCodes().GetNumExtruders())
		{
			const DriverId id = extruderDrivers[LogicalDriveToExtruder(axisOrExtruder)];
			if (id.IsLocal())
			{
				localFunc(id.localDriver);
			}
			else
			{
				remoteFunc(id);
			}
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders + NumDirectDrivers)
	{
		localFunc(axisOrExtruder - MaxAxesPlusExtruders);
	}
}

#else

// Function to identify and iterate through all drivers attached to an axis or extruder
void Move::IterateDrivers(size_t axisOrExtruder, function_ref_noexcept<void(uint8_t) noexcept> localFunc) noexcept
{
	if (axisOrExtruder < reprap.GetGCodes().GetTotalAxes())
	{
		for (size_t i = 0; i < axisDrivers[axisOrExtruder].numDrivers; ++i)
		{
			const DriverId id = axisDrivers[axisOrExtruder].driverNumbers[i];
			localFunc(id.localDriver);
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders)
	{
		if (LogicalDriveToExtruder(axisOrExtruder) < reprap.GetGCodes().GetNumExtruders())
		{
			const DriverId id = extruderDrivers[LogicalDriveToExtruder(axisOrExtruder)];
			localFunc(id.localDriver);
		}
	}
	else if (axisOrExtruder < MaxAxesPlusExtruders + NumDirectDrivers)
	{
		localFunc(axisOrExtruder - MaxAxesPlusExtruders);
	}
}

#endif

#ifdef DUET3_MB6XD

bool Move::HasDriverError(size_t driver) const noexcept
{
	if (driver < NumDirectDrivers)
	{
		const bool b = digitalRead(DRIVER_ERR_PINS[driver]);
		return (driverErrPinsActiveLow) ? !b : b;
	}
	return false;
}

#endif

void Move::SetAxisDriversConfig(size_t axis, size_t numValues, const DriverId driverNumbers[]) noexcept
{
	AxisDriversConfig& cfg = axisDrivers[axis];
	cfg.numDrivers = numValues;
	uint32_t bitmap = 0;
	for (size_t i = 0; i < numValues; ++i)
	{
		const DriverId id = driverNumbers[i];
		cfg.driverNumbers[i] = id;
		if (id.IsLocal())
		{
			bitmap |= StepPins::CalcDriverBitmap(id.localDriver);
#if HAS_SMART_DRIVERS
			SmartDrivers::SetAxisNumber(id.localDriver, axis);
#endif
		}
	}
	dms[axis].driversNormallyUsed = bitmap;
	dms[axis].isExtruder = false;
}

// Set the characteristics of an axis
void Move::SetAxisType(size_t axis, AxisWrapType wrapType, bool isNistRotational) noexcept
{
	if (isNistRotational)
	{
		rotationalAxes.SetBit(axis);
	}
	else
	{
		linearAxes.SetBit(axis);
	}

	switch (wrapType)
	{
#if 0	// shortcut axes not implemented yet
	case AxisWrapType::wrapWithShortcut:
		shortcutAxes.SetBit(axis);
		[[fallthrough]];
#endif
	case AxisWrapType::wrapAt360:
		continuousAxes.SetBit(axis);
		break;

	default:
		break;
	}
}

// Map an extruder to a driver
void Move::SetExtruderDriver(size_t extruder, DriverId driver) noexcept
{
	extruderDrivers[extruder] = driver;
	const size_t logicalDriveNumber = ExtruderToLogicalDrive(extruder);
	if (driver.IsLocal())
	{
#if HAS_SMART_DRIVERS
		SmartDrivers::SetAxisNumber(driver.localDriver, logicalDriveNumber);
#endif
		dms[logicalDriveNumber].driversNormallyUsed = StepPins::CalcDriverBitmap(driver.localDriver);
	}
	else
	{
		dms[logicalDriveNumber].driversNormallyUsed = 0;
	}
	dms[logicalDriveNumber].isExtruder = true;
}

void Move::SetDriverStepTiming(size_t driver, const float microseconds[4]) noexcept
{
#ifdef DUET3_MB6XD
	memcpyf(driverTimingMicroseconds[driver], microseconds, 4);
	UpdateDriverTimings();
#else
	const uint32_t bitmap = StepPins::CalcDriverBitmap(driver);
	slowDriversBitmap &= ~bitmap;								// start by assuming this drive does not need extended timing
	if (slowDriversBitmap == 0)
	{
		for (uint32_t& entry : slowDriverStepTimingClocks)
		{
			entry = 0;											// reset all to zero if we have no known slow drivers
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(slowDriverStepTimingClocks); ++i)
	{
		if (microseconds[i] > MinStepPulseTiming)
		{
			slowDriversBitmap |= StepPins::CalcDriverBitmap(driver);			// this drive does need extended timing
			const uint32_t clocks = MicrosecondsToStepClocks(microseconds[i]);	// convert microseconds to step clocks, rounding up
			if (clocks > slowDriverStepTimingClocks[i])
			{
				slowDriverStepTimingClocks[i] = clocks;
			}
		}
	}
#endif
}

// Get the driver step timing, returning true if we are using slower timing than standard
bool Move::GetDriverStepTiming(size_t driver, float microseconds[4]) const noexcept
{
#ifdef DUET3_MB6XD
	memcpyf(microseconds, driverTimingMicroseconds[driver], 4);
	return true;
#else
	const bool isSlowDriver = ((slowDriversBitmap & StepPins::CalcDriverBitmap(driver)) != 0);
	for (size_t i = 0; i < 4; ++i)
	{
		microseconds[i] = (isSlowDriver)
							? (float)slowDriverStepTimingClocks[i] * (1000000.0/(float)StepClockRate)
								: 0.0;
	}
	return isSlowDriver;
#endif
}

#if HAS_STALL_DETECT || SUPPORT_CAN_EXPANSION

// Configure the motor stall detection, returning true if an error was encountered
GCodeResult Move::ConfigureStallDetection(GCodeBuffer& gb, const StringRef& reply, OutputBuffer *_ecv_null & buf) THROWS(GCodeException)
{
	// Build a bitmap of all the drivers referenced
	// First looks for explicit driver numbers
	LocalDriversBitmap drivers;
# if SUPPORT_CAN_EXPANSION
	CanDriversList canDrivers;
# endif
	if (gb.Seen('P'))
	{
		DriverId drives[NumDirectDrivers];
		size_t dCount = NumDirectDrivers;
		gb.GetDriverIdArray(drives, dCount);
		for (size_t i = 0; i < dCount; i++)
		{
			if (drives[i].IsLocal())
			{
# if HAS_SMART_DRIVERS
				if (drives[i].localDriver >= numSmartDrivers)
				{
					reply.printf("Invalid local drive number '%u'", drives[i].localDriver);
					return GCodeResult::error;
				}
# endif
				drivers.SetBit(drives[i].localDriver);
			}
# if SUPPORT_CAN_EXPANSION
			else
			{
				canDrivers.AddEntry(drives[i]);
			}
# endif
		}
	}

	// Now look for axes
	for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); ++axis)
	{
		if (gb.Seen(reprap.GetGCodes().GetAxisLetters()[axis]))
		{
			IterateDrivers(axis,
							[&drivers](uint8_t localDriver) noexcept -> void { drivers.SetBit(localDriver); }
# if SUPPORT_CAN_EXPANSION
						  , [&canDrivers](DriverId driver) noexcept -> void { canDrivers.AddEntry(driver); }
# endif
						  );
		}
	}

	// Look for extruders
	if (gb.Seen('E'))
	{
		uint32_t extruderNumbers[MaxExtruders];
		size_t numSeen = MaxExtruders;
		gb.GetUnsignedArray(extruderNumbers, numSeen, false);
		for (size_t i = 0; i < numSeen; ++i)
		{
			if (extruderNumbers[i] < MaxExtruders)
			{
				const DriverId driver = GetExtruderDriver(extruderNumbers[i]);
				if (driver.IsLocal())
				{
					drivers.SetBit(driver.localDriver);
				}
# if SUPPORT_CAN_EXPANSION
				else
				{
					canDrivers.AddEntry(driver);
				}
# endif
			}
		}
	}

# if HAS_STALL_DETECT
	// Now check for values to change
	bool seen = false;

	// Check for stall threshold parameter
	{
		int32_t sgThreshold;
		if (gb.TryGetLimitedIValue('S', sgThreshold, seen,
#ifdef DUET_NG
				-64, 63						// TMC2660 accepts values in this range
#else
				-128, 127					// TMC2209 accepts 0..255 which we map to -128..127. TMC2160 accepts -64..63. We may have a mixture in a CAN bus system.
#endif
		   ))
		{
			drivers.Iterate([sgThreshold](unsigned int drive, unsigned int) noexcept -> void { SmartDrivers::SetStallThreshold(drive, sgThreshold); });
		}
	}

	// Check for stall detect filter parameter
	{
		bool sgFilter;
		if (gb.TryGetBValue('F', sgFilter, seen))
		{
			drivers.Iterate([sgFilter](unsigned int drive, unsigned int) noexcept -> void { SmartDrivers::SetStallFilter(drive, sgFilter); });
		}
	}

	// Check for stall minimum fullsteps/sec parameter
	if (gb.Seen('H'))
	{
		seen = true;
		const unsigned int stepsPerSecond = gb.GetUIValue();
		drivers.Iterate([stepsPerSecond](unsigned int drive, unsigned int) noexcept -> void { SmartDrivers::SetStallMinimumStepsPerSecond(drive, stepsPerSecond); });
	}

	// Check for coolconf parameter
	{
		uint32_t coolStepConfig;
		if (gb.TryGetLimitedUIValue('T', coolStepConfig, seen, 1u << 16))
		{
			drivers.Iterate([coolStepConfig](unsigned int drive, unsigned int) noexcept -> void { SmartDrivers::SetRegister(drive, SmartDriverRegister::coolStep, coolStepConfig); } );
		}
	}

	// Check for action-on-stall parameter
	{
		uint32_t action;
		if (gb.TryGetLimitedUIValue('R', action, seen, 4))
		{
			switch (action)
			{
			case 0:
			default:
				logOnStallDrivers &= ~drivers;
				eventOnStallDrivers &= ~drivers;
				break;

			case 1:
				eventOnStallDrivers &= ~drivers;
				logOnStallDrivers |= drivers;
				break;

			case 2:
			case 3:
				logOnStallDrivers &= ~drivers;
				eventOnStallDrivers |= drivers;
				break;
			}
		}
	}

#else
	// Board does not have any local drivers with stall detection but may have CAN-connected drivers
	const bool seen = gb.SeenAny("SFHTR");
#endif

	if (seen)
	{
# if SUPPORT_CAN_EXPANSION
		const GCodeResult rslt = CanInterface::GetSetRemoteDriverStallParameters(canDrivers, gb, reply, buf);
#  if !HAS_SMART_DRIVERS
		if (drivers.IsNonEmpty())
		{
			reply.lcatf("Stall detection not available for external drivers");
			return max(rslt, GCodeResult::warning);
		}
#  endif
		return rslt;
# else
		return GCodeResult::ok;
# endif
	}

	// Print the stall status
# if HAS_SMART_DRIVERS
	if (drivers.IsEmpty()
#  if SUPPORT_CAN_EXPANSION
		&& canDrivers.IsEmpty()
#  endif
	   )
	{
		drivers = LocalDriversBitmap::MakeLowestNBits(numSmartDrivers);
	}

	if (!OutputBuffer::Allocate(buf, false))
	{
		return GCodeResult::notFinished;
	}

	drivers.Iterate
		([buf, this, &reply](unsigned int drive, unsigned int) noexcept -> void
			{
#  if SUPPORT_CAN_EXPANSION
				buf->lcatf("Driver 0.%u: ", drive);
#  else
				buf->lcatf("Driver %u: ", drive);
#  endif
				reply.Clear();										// we use 'reply' as a temporary buffer
				SmartDrivers::AppendStallConfig(drive, reply);
				buf->cat(reply.c_str());
				buf->catf(", action on stall: %s",
							(eventOnStallDrivers.IsBitSet(drive)) ? "raise event"
								: (logOnStallDrivers.IsBitSet(drive)) ? "log"
									: "none"
						  );
			}
		);
# else
	if (canDrivers.IsEmpty())
	{
		reply.copy("No local drivers have stall detection");
		return GCodeResult::ok;
	}

	if (!OutputBuffer::Allocate(buf))
	{
		return GCodeResult::notFinished;
	}
# endif
# if SUPPORT_CAN_EXPANSION
	return CanInterface::GetSetRemoteDriverStallParameters(canDrivers, gb, reply, buf);
# else
	return GCodeResult::ok;
# endif
}

#endif

#if SUPPORT_REMOTE_COMMANDS

// Stop a drive and re-calculate the end position
void Move::StopDriveFromRemote(size_t drive) noexcept
{
	dms[drive].StopDriverFromRemote();
}

// Get the number of steps taken by the last move, if it was an isolated move
int32_t Move::GetLastMoveStepsTaken(size_t drive) const noexcept
{
	const DriveMovement& dm = dms[drive];
	return dm.currentMotorPosition - dm.positionAtMoveStart;
}

#endif

// Reset all extruder positions to zero. Called when we start a print. All motion must be stopped before we call this, otherwise we will get Code 6 movement system errors.
void Move::ResetExtruderPositions() noexcept
{
	for (size_t drive = MaxAxesPlusExtruders - reprap.GetGCodes().GetNumExtruders(); drive < MaxAxesPlusExtruders; ++drive)
	{
		dms[drive].SetMotorPosition(0);
	}
}

// Update the backlash correction in steps. Called when the configured backlash distance or steps/mm is changed.
void Move::UpdateBacklashSteps() noexcept
{
	for (size_t i = 0; i < reprap.GetGCodes().GetTotalAxes(); ++i)
	{
		backlashSteps[i] = (unsigned int)(backlashMm[i] * DriveStepsPerMm(i));
	}
}

// Given the number of microsteps that an axis has been asked to move, return the number that it should actually move
int32_t Move::ApplyBacklashCompensation(size_t drive, int32_t delta) noexcept
{
	// If this drive has changed direction, update the backlash correction steps due
	const bool backwards = (delta < 0);
	int32_t& targetSteps = targetBacklashSteps[drive];
	if (backwards != lastDirections.IsBitSet(drive))
	{
		lastDirections.InvertBit(drive);		// Direction has reversed
		int32_t temp = (int32_t)backlashSteps[drive];
		if (backwards)
		{
			temp = -temp;
		}
		targetSteps += temp;
	}

	int32_t& currentSteps = currentBacklashSteps[drive];
	const int32_t stepsDue = targetSteps - currentSteps;

	// Apply some or all of the compensation steps due
	if (stepsDue != 0)
	{
		if ((unsigned long)labs(stepsDue) * backlashCorrectionDistanceFactor <= (unsigned long)labs(delta))		// avoid a division if we can
		{
			delta += stepsDue;
			currentSteps = targetSteps;
		}
		else
		{
			const int32_t maxAllowedSteps = (int32_t)max<uint32_t>((uint32_t)labs(delta)/backlashCorrectionDistanceFactor, 1u);
			const int32_t stepsToDo = (stepsDue < 0) ? max<int32_t>(stepsDue, -maxAllowedSteps) : min<int32_t>(stepsDue, maxAllowedSteps);
			currentSteps += stepsToDo;
			delta += stepsToDo;
		}
	}
	return delta;
}

void Move::PollOneDriver(size_t driver) noexcept
{
	if (enableValues[driver] >= 0)	// don't poll driver if it is flagged "no poll"
	{

#if defined(DUET3_MB6XD)
		// Don't raise driver error events while we are being tested by ATE
		const bool reportError = !CanInterface::InTestMode() && HasDriverError(driver);
		StandardDriverStatus stat((reportError) ? ((uint32_t)1u << StandardDriverStatus::ExternDriverErrorBitPos) : 0);
#else
		StandardDriverStatus stat = SmartDrivers::GetStatus(driver, true, true);
#endif
#if HAS_SMART_DRIVERS
		const LocalDriversBitmap mask = LocalDriversBitmap::MakeFromBits(driver);
		if (stat.ot)
		{
			temperatureShutdownDrivers |= mask;
		}
		else
		{
			temperatureShutdownDrivers &= ~mask;
			if (stat.otpw)
			{
				temperatureWarningDrivers |= mask;
			}
			else
			{
				temperatureWarningDrivers &= ~mask;
			}
		}

		if (stat.s2ga || stat.s2gb || stat.s2vsa || stat.s2vsb)
		{
			shortToGroundDrivers |= mask;
		}
		else
		{
			shortToGroundDrivers &= ~mask;
		}

		// Deal with the open load bits
		// The driver often produces a transient open-load error, especially in stealthchop mode, so we require the condition to persist before we report it.
		// So clear them unless they have been active for the minimum time.
		MillisTimer& timer = openLoadTimers[driver];
		if (stat.IsAnyOpenLoadBitSet())
		{
			if (timer.IsRunning())
			{
				if (!timer.CheckNoStop(OpenLoadTimeout))
				{
					stat.ClearOpenLoadBits();
				}
			}
			else
			{
				timer.Start();
				stat.ClearOpenLoadBits();
			}
		}
		else
		{
			timer.Stop();
		}
#endif	// HAS_SMART_DRIVERS

#ifdef DUET3_MB6XD
		// Suppress spurious GCC 12.2 warning
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Warray-bounds"
#endif
		const StandardDriverStatus oldStatus = lastEventStatus[driver];
		lastEventStatus[driver] = stat;
#ifdef DUET3_MB6XD
# pragma GCC diagnostic pop
#endif
		if (stat.HasNewErrorSince(oldStatus))
		{
			// It's a new error
# if SUPPORT_REMOTE_COMMANDS
			if (inExpansionMode)
			{
				CanInterface::RaiseEvent(EventType::driver_error, stat.AsU16(), driver, "", va_list());
			}
			else
#endif
			{
				Event::AddEvent(EventType::driver_error, stat.AsU16(), CanInterface::GetCanAddress(), driver, "");
			}
		}
		else if (stat.HasNewWarningSince(oldStatus))
		{
			// It's a new warning
# if SUPPORT_REMOTE_COMMANDS
			if (inExpansionMode)
			{
				CanInterface::RaiseEvent(EventType::driver_warning, stat.AsU16(), driver, "", va_list());
			}
			else
#endif
			{
				Event::AddEvent(EventType::driver_warning, stat.AsU16(), CanInterface::GetCanAddress(), driver, "");
			}
		}

# if HAS_STALL_DETECT
		if (stat.HasNewStallSince(oldStatus))
		{
			// This stall is new so check whether we need to perform some action in response to the stall
#  if SUPPORT_REMOTE_COMMANDS
			if (inExpansionMode)
			{
				if (eventOnStallDrivers.Intersects(mask))
				{
					CanInterface::RaiseEvent(EventType::driver_stall, 0, driver, "", va_list());
				}
			}
			else
#  endif
			{
				if (eventOnStallDrivers.Intersects(mask))
				{
					Event::AddEvent(EventType::driver_stall, 0, CanInterface::GetCanAddress(), driver, "");
				}
				else if (logOnStallDrivers.Intersects(mask))
				{
					reprap.GetPlatform().MessageF(WarningMessage, "Driver %u stalled at Z height %.2f\n", driver, (double)reprap.GetGCodes().GetPrimaryMovementState().LiveMachineCoordinate(Z_AXIS));
				}
			}
		}
# endif
	}

	// Brake control
	if (brakeOffTimers[driver].CheckAndStop(brakeOffDelays[driver]))
	{
		DisengageBrake(driver);
	}
	if (motorOffTimers[driver].CheckAndStop(motorOffDelays[driver]))
	{
		InternalDisableDriver(driver);
	}

# if SUPPORT_BRAKE_PWM
	// If the brake solenoid is activated, adjust the PWM if necessary
	if (currentBrakePwm[driver] != 0.0 && reprap.GetPlatform().GetVinVoltage() > 10.0)
	{
		const float newBrakePwm = min<float>(brakeVoltages[driver]/reprap.GetPlatform().GetVinVoltage(), 1.0);
		if (fabsf(newBrakePwm - currentBrakePwm[driver]) >= 0.05)
		{
			brakePorts[driver].WriteAnalog(newBrakePwm);
			currentBrakePwm[driver] = newBrakePwm;
		}
	}
# endif
}

#if HAS_SMART_DRIVERS

// TMC driver temperatures
float Move::GetTmcDriversTemperature(unsigned int boardNumber) const noexcept
{
#if defined(DUET3MINI)
	const LocalDriversBitmap mask = LocalDriversBitmap::MakeLowestNBits(7);						// report the 2-driver addon along with the main board
#elif defined(DUET3)
	const LocalDriversBitmap mask = LocalDriversBitmap::MakeLowestNBits(6);						// there are 6 drivers, only one board
#elif defined(DUET_NG)
	const LocalDriversBitmap mask = LocalDriversBitmap::MakeLowestNBits(5).ShiftUp(5 * boardNumber);	// there are 5 drivers on each board
#elif defined(DUET_M)
	const LocalDriversBitmap mask = LocalDriversBitmap::MakeLowestNBits(7);						// report the 2-driver addon along with the main board
#elif defined(PCCB_10)
	const LocalDriversBitmap mask = (boardNumber == 0)
							? LocalDriversBitmap::MakeLowestNBits(2)							// drivers 0,1 are on-board
								: LocalDriversBitmap::MakeLowestNBits(5).ShiftUp(2);			// drivers 2-7 are on the DueX5
#else
# error Undefined board
#endif
	return (temperatureShutdownDrivers.Intersects(mask)) ? 150.0
			: (temperatureWarningDrivers.Intersects(mask)) ? 100.0
				: 0.0;
}

/*static*/ void Move::SpinSmartDrivers(bool driversPowered) noexcept
{
	SmartDrivers::Spin(driversPowered);
}

/*static*/ StandardDriverStatus Move::GetSmartDriverStatus(size_t driver, bool accumulated, bool clearAccumulated) noexcept
{
	return SmartDrivers::GetStatus(driver, accumulated, clearAccumulated);
}

void Move::DriversJustPoweredUp() noexcept
{
	for (size_t i= 0; i < MaxSmartDrivers; ++i)
	{
		openLoadTimers[i].Stop();
	}
	temperatureShutdownDrivers.Clear();
	temperatureWarningDrivers.Clear();
	shortToGroundDrivers.Clear();
}

void Move::TurnSmartDriversOff() noexcept
{
	SmartDrivers::TurnDriversOff();
}

#endif

#if SUPPORT_CAN_EXPANSION

// This is called when we update endstop states because of a message from a remote board.
// In time we may use it to help implement interrupt-driven local endstops too, but for now those are checked in the step ISR by a direct call to CheckEndstops().
void Move::OnEndstopOrZProbeStatesChanged() noexcept
{
	bool wakeAsyncSender;
	{
		BasePriorityBooster booster(NvicPriorityStep);		// shut out the step interrupt
		wakeAsyncSender = CheckEndstops(true);
	}
	if (wakeAsyncSender) { CanInterface::WakeAsyncSender(); }
}

GCodeResult Move::UpdateRemoteStepsPerMmAndMicrostepping(AxesBitmap axesAndExtruders, const StringRef& reply) noexcept
{
	CanDriversData<StepsPerUnitAndMicrostepping> data;
	axesAndExtruders.Iterate([this, &data](unsigned int axisOrExtruder, unsigned int count) noexcept -> void
								{
									const StepsPerUnitAndMicrostepping driverData(DriveStepsPerMm(axisOrExtruder), GetRawMicrostepping(axisOrExtruder));
									this->IterateRemoteDrivers(axisOrExtruder,
																[&data, &driverData](DriverId driver) noexcept -> void
																{
																	data.AddEntry(driver, driverData);
																}
															  );
								}
							);
	return CanInterface::SetRemoteDriverStepsPerMmAndMicrostepping(data, reply);
}

#endif

#if SUPPORT_ASYNC_MOVES

// Get and lock the aux move buffer. If successful, return a pointer to the buffer.
// The caller must not attempt to lock the aux buffer more than once, and must call ReleaseAuxMove to release the buffer.
AsyncMove *_ecv_null Move::LockAuxMove() noexcept
{
	AtomicCriticalSectionLocker lock;
	if (!auxMoveLocked && !auxMoveAvailable)
	{
		auxMoveLocked = true;
		return &auxMove;
	}
	return nullptr;
}

// Release the aux move buffer and optionally signal that it contains a move
// The caller must have locked the buffer before calling this. If it calls this with hasNewMove true, it must have populated the move buffer with the move details
void Move::ReleaseAuxMove(bool hasNewMove) noexcept
{
	auxMoveAvailable = hasNewMove;
	auxMoveLocked = false;
	MoveAvailable();
}

// Configure height following
GCodeResult Move::ConfigureHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (heightController == nullptr)
	{
		heightController = new HeightController;
	}
	return heightController->Configure(gb, reply);
}

// Start/stop height following
GCodeResult Move::StartHeightFollowing(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	if (heightController == nullptr)
	{
		reply.copy("Height following has not been configured");
		return GCodeResult::error;
	}
	return heightController->StartHeightFollowing(gb, reply);
}

#endif

// End
