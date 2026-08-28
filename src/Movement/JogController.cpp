/*
 * JogController.cpp
 *
 *  See JogController.h for what this does.
 */

#include "JogController.h"

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <Platform/RepRap.h>

// Below this the chunk is not worth queueing; it is well under one microstep on any sane machine.
constexpr float MinChunkDistance = 0.001;

JogController::JogController() noexcept
	: jogAxes(), chunkMillis(DefaultChunkMillis), chunkClocks((DefaultChunkMillis * StepClockRate)/1000),
	  timeoutMillis(DefaultTimeoutMillis), whenLastCommanded(0), maxQueuedMoves(DefaultMaxQueuedMoves), active(false)
{
	for (float& s : requestedSpeeds)
	{
		s = 0.0;
	}
}

// The highest speed we are prepared to run this axis at: its configured maximum, and nothing else.
//
// This used to also clamp to 2.a.P. That came from every chunk having to be stoppable within itself,
// because DDA::InitStandardMove sets endSpeed = 0 (DDA.cpp:624) until a following move exists, and a
// singly-generated chunk never had one. The machine never needed to stop within one chunk; it needs to
// be able to decelerate from its current speed, which takes v/a however the motion was commanded.
// Keeping enough chunks queued that each has a successor lets lookahead blend them, which is what an
// ordinary G-code stream already relies on.
float JogController::MaxSpeedForAxis(size_t axis) const noexcept
{
	return reprap.GetMove().MaxFeedrate(axis);
}

void JogController::ClampSpeeds() noexcept
{
	const size_t numVisibleAxes = reprap.GetGCodes().GetVisibleAxes();
	jogAxes.Clear();
	clampedAxes.Clear();
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		const float limit = MaxSpeedForAxis(axis);
		if (fabsf(requestedSpeeds[axis]) > limit)
		{
			clampedAxes.SetBit(axis);					// the host asked for more than M203 allows; say so rather than silently obeying something else
		}
		requestedSpeeds[axis] = constrain<float>(requestedSpeeds[axis], -limit, limit);
		// A speed below one chunk's minimum distance cannot be expressed at all, so treat it as zero rather
		// than as a jog that generates nothing. Otherwise the axis counts as jogging, keeps the machine out
		// of idle, and produces a chunk per pass that is only thrown away.
		if (fabsf(requestedSpeeds[axis]) * (float)chunkClocks < MinChunkDistance)
		{
			requestedSpeeds[axis] = 0.0;
		}
		if (requestedSpeeds[axis] != 0.0)
		{
			jogAxes.SetBit(axis);
		}
	}
	for (size_t axis = numVisibleAxes; axis < MaxAxes; ++axis)
	{
		requestedSpeeds[axis] = 0.0;
	}
}

void JogController::Stop() noexcept
{
	for (float& s : requestedSpeeds)
	{
		s = 0.0;
	}
	jogAxes.Clear();
	active = false;
}

void JogController::ReportStatus(const StringRef& reply) const noexcept
{
	const GCodes& gcodes = reprap.GetGCodes();
	const char *_ecv_array const axisLetters = gcodes.GetAxisLetters();
	reply.printf("Jogging %s, chunk %" PRIu32 "ms, timeout %" PRIu32 "ms, queue %u",
					(active) ? "active" : "inactive", chunkMillis, timeoutMillis, maxQueuedMoves);
	if (clampedAxes.IsNonEmpty())
	{
		reply.cat(", clamped to axis maximum:");
		for (size_t axis = 0; axis < gcodes.GetVisibleAxes(); ++axis)
		{
			if (clampedAxes.IsBitSet(axis))
			{
				reply.catf(" %c%.1f", axisLetters[axis], (double)InverseConvertSpeedToMmPerSec(requestedSpeeds[axis]));
			}
		}
	}
	if (active)
	{
		reply.cat(", speeds");
		for (size_t axis = 0; axis < gcodes.GetVisibleAxes(); ++axis)
		{
			if (jogAxes.IsBitSet(axis))
			{
				reply.catf(" %c%.1f", axisLetters[axis], (double)InverseConvertSpeedToMmPerSec(requestedSpeeds[axis]));
			}
		}
	}
}

// M700: set the jog velocity of each axis, in mm (or degrees) per second.
// The axis letters that are present define the whole velocity vector: any axis not mentioned is set to zero, so that a
// truncated or lost command can never leave an axis running.
GCodeResult JogController::ProcessM700(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodes& gcodes = reprap.GetGCodes();
	const char *_ecv_array const axisLetters = gcodes.GetAxisLetters();
	const size_t numVisibleAxes = gcodes.GetVisibleAxes();

	// Tuning parameters. These take effect on the next chunk.
	bool seenParam = false;
	gb.TryGetLimitedUIValue('P', chunkMillis, seenParam, MinChunkMillis, MaxChunkMillis + 1);
	gb.TryGetLimitedUIValue('R', timeoutMillis, seenParam, 1, MaxTimeoutMillis + 1);
	uint32_t queueDepth = maxQueuedMoves;
	gb.TryGetLimitedUIValue('D', queueDepth, seenParam, MinMaxQueuedMoves, MaxMaxQueuedMoves + 1);
	chunkClocks = (chunkMillis * StepClockRate)/1000;
	maxQueuedMoves = queueDepth;

	// S0 is an explicit stop.
	if (gb.Seen('S') && gb.GetUIValue() == 0)
	{
		Stop();
		return GCodeResult::ok;
	}

	float newSpeeds[MaxAxes] = { 0.0 };
	bool seenAxis = false;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (gb.Seen(axisLetters[axis]))
		{
			// Speeds are always in mm (or degrees) per second. G20 deliberately does not rescale them: the sender of a
			// velocity command should not have its meaning changed by modal state it may know nothing about.
			newSpeeds[axis] = ConvertSpeedFromMmPerSec(gb.GetFValue());
			seenAxis = true;
		}
		else
		{
			newSpeeds[axis] = 0.0;
		}
	}

	if (!seenAxis)
	{
		if (!seenParam)
		{
			ReportStatus(reply);
		}
		return GCodeResult::ok;
	}

	AxesBitmap newJogAxes;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (newSpeeds[axis] != 0.0)
		{
			newJogAxes.SetBit(axis);
		}
	}

	if (newJogAxes.IsNonEmpty() && gcodes.CheckEnoughAxesHomed(newJogAxes))
	{
		reply.copy("Insufficient axes homed");
		return GCodeResult::error;
	}

	if (!active && newJogAxes.IsNonEmpty())
	{
		// Starting up. We take over the axis positions of movement system 0, so it must be at a standstill and not printing.
		if (gcodes.IsReallyPrintingOrResuming())
		{
			reply.copy("Cannot jog while a print is running");
			return GCodeResult::error;
		}
		if (!gcodes.LockMovementSystemAndWaitForStandstill(gb, 0))
		{
			return GCodeResult::notFinished;
		}
	}

#if SUPPORT_ASYNC_MOVES
	if ((newJogAxes & ~jogAxes).IsNonEmpty()					// an axis we are not already moving has been added, so it may not be ours yet
		&& gcodes.moveStates[0].AllocateAxes(newJogAxes, ParameterLettersBitmap()).IsNonEmpty())
	{
		reply.copy("Cannot jog: axes are in use by another movement system");
		return GCodeResult::error;
	}
#endif

	memcpyf(requestedSpeeds, newSpeeds, numVisibleAxes);
	ClampSpeeds();
	whenLastCommanded = millis();
	active = jogAxes.IsNonEmpty();
	return GCodeResult::ok;
}

// Top the movement queue up. Called regularly from GCodes::Spin.
void JogController::Spin() noexcept
{
	if (!active)
	{
		return;
	}

	GCodes& gcodes = reprap.GetGCodes();
	Move& move = reprap.GetMove();

	// Watchdog: an input that stops sending must not leave the machine moving.
	if (millis() - whenLastCommanded > timeoutMillis)
	{
		Stop();
		return;
	}

	// Something else has taken over movement, so get out of the way. A macro or a tool change moves axes
	// on its own account, and jogging underneath it would fight it for the same movement system.
	// DoingFileMacro deliberately excludes daemon.g (GCodes.cpp:374), so a daemon running on its usual
	// cycle does not chop the jog stream up.
	// Deliberately NOT included: WaitingForAcknowledgement. "Jog to the workpiece corner, then press OK"
	// is a standard CNC setup pattern, and the machine is stationary with the operator at the controls,
	// so blocking it would remove a genuinely useful workflow for no safety gain.
	if (gcodes.IsReallyPrintingOrResuming() || gcodes.DoingFileMacro() || gcodes.IsDoingToolChange())
	{
		Stop();
		return;
	}

	MovementState& ms = gcodes.moveStates[0];
	if (ms.segmentsLeft != 0)
	{
		return;																// the previous chunk has not been picked up yet
	}
	if (move.GetScheduledMoves() - move.GetCompletedMoves() >= maxQueuedMoves)
	{
		return;																// far enough ahead already; queueing more would only add latency
	}

	(void)GenerateChunk(ms);
}

// Build one constant-velocity chunk and hand it to the Move subsystem. Return true if we queued anything.
bool JogController::GenerateChunk(MovementState& ms) noexcept
{
	GCodes& gcodes = reprap.GetGCodes();
	Move& move = reprap.GetMove();
	const size_t numVisibleAxes = gcodes.GetVisibleAxes();

	gcodes.SetMoveBufferDefaults(ms);										// this also copies the previous target into ms.initialCoords

	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		if (requestedSpeeds[axis] != 0.0)
		{
			ms.currentUserPosition[axis] += requestedSpeeds[axis] * (float)chunkClocks;
		}
	}

	ms.raw.movementTool = ms.currentTool;
	gcodes.ToolOffsetTransform(ms, jogAxes);

	// Limit the whole line rather than just its end point, so that kinematics with a non-rectangular envelope stay inside it.
	const LimitPositionResult lp = move.GetKinematics().LimitPosition(ms.raw.coords, ms.initialCoords, numVisibleAxes,
																		gcodes.axesVirtuallyHomed & jogAxes, true, gcodes.limitAxes);
	if (lp == LimitPositionResult::intermediateUnreachable || lp == LimitPositionResult::adjustedAndIntermediateUnreachable)
	{
		Stop();
		return false;
	}
	if (lp == LimitPositionResult::adjusted)
	{
		gcodes.ToolOffsetInverseTransform(ms, ms.raw.coords, ms.currentUserPosition);	// the target was clipped, so put the user position back in step with it
	}

	// Axes that have run into their limit contribute nothing to the distance, which is exactly what keeps the remaining
	// axes at their commanded speed: every axis still covers its own delta in one chunk time.
	float distanceSquared = 0.0;
	for (size_t axis = 0; axis < numVisibleAxes; ++axis)
	{
		const float d = ms.raw.coords[axis] - ms.initialCoords[axis];
		if (d != 0.0)
		{
			distanceSquared += fsquare(d);
			if (move.IsAxisRotational(axis))
			{
				ms.raw.rotationalAxesMentioned = true;
			}
			else
			{
				ms.raw.linearAxesMentioned = true;
			}
		}
	}

	if (distanceSquared < fsquare(MinChunkDistance))
	{
		// Nothing worth moving. The target has to be put back, not just abandoned: currentUserPosition was
		// already advanced above, and SetMoveBufferDefaults seeds initialCoords from raw.coords, so a
		// rejected chunk would otherwise become the next chunk's baseline and the reported position would
		// climb at the commanded speed while the machine stood still, with nothing ever resyncing it.
		memcpyf(ms.raw.coords, ms.initialCoords, numVisibleAxes);
		gcodes.ToolOffsetInverseTransform(ms, ms.raw.coords, ms.currentUserPosition);
		return false;
	}

	ms.raw.isCoordinated = true;
	ms.raw.canPauseAfter = true;
	ms.raw.feedRate = fastSqrtf(distanceSquared)/(float)chunkClocks;
	ms.raw.originalFeedRate = (float16_t)(InverseConvertSpeedToMmPerSec(ms.raw.feedRate) * MinutesToSeconds);	// this field is in mm/min
	ms.raw.moveStartVirtualExtruderPosition = ms.latestVirtualExtruderPosition;

	gcodes.NewSegmentableMoveAvailable(ms);
	return true;
}

// End
