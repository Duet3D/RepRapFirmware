/*
 * AxisFollower.cpp
 *
 *  See AxisFollower.h for what this is for.
 */

#include "AxisFollower.h"

#include <GCodes/GCodes.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>
#include <Movement/Move.h>
#include <Platform/RepRap.h>
#include <ObjectModel/ObjectModel.h>

// Object model. AxisControl and DWC would otherwise have to parse the text of an M604 report to know
// whether following is engaged, which is not something a UI should have to do.

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...)		OBJECT_MODEL_FUNC_BODY(AxisFollower, __VA_ARGS__)

constexpr ObjectModelTableEntry AxisFollower::objectModelTable[] =
{
	// Within each group, these entries must be in alphabetical order
	// 0. AxisFollower members
	{ "engaged",	OBJECT_MODEL_FUNC(self->engaged),										ObjectModelEntryFlags::live },
	{ "follower",	OBJECT_MODEL_FUNC(self->GetAxisLetterOrNull(self->followerAxis)),		ObjectModelEntryFlags::none },
	{ "leader",		OBJECT_MODEL_FUNC(self->GetAxisLetterOrNull(self->leaderAxis)),			ObjectModelEntryFlags::none },
	{ "offset",		OBJECT_MODEL_FUNC(self->offset, 3),										ObjectModelEntryFlags::none },
	{ "scale",		OBJECT_MODEL_FUNC(self->scale, 3),										ObjectModelEntryFlags::none },
};

constexpr uint8_t AxisFollower::objectModelTableDescriptor[] = { 1, 5 };

DEFINE_GET_OBJECT_MODEL_TABLE(AxisFollower)

// Returns null when unconfigured, so the field reads as null rather than as a bogus axis letter.
const char *_ecv_array AxisFollower::GetAxisLetterOrNull(int32_t axis) const noexcept
{
	if (axis < 0)
	{
		return nullptr;
	}
	static char letter[2] = { 0, 0 };
	letter[0] = reprap.GetGCodes().GetAxisLetters()[axis];
	return letter;
}

AxisFollower::AxisFollower() noexcept
	: followerAxis(-1), leaderAxis(-1), scale(-1.0), offset(0.0), engaged(false)
{
}

void AxisFollower::Apply(float coords[MaxAxes]) const noexcept
{
	if (!engaged || followerAxis < 0 || leaderAxis < 0)
	{
		return;
	}

	const Move& move = reprap.GetMove();
	const float wanted = (scale * coords[leaderAxis]) + offset;

	// Clamped here rather than left to Kinematics::LimitPosition, which only limits axes the command
	// actually mentioned - and the whole point is that nothing mentions the follower. Clamping is also
	// the useful behaviour rather than an error case: a dust shoe tracks the tool down until it reaches
	// its lower limit and then stays resting there while Z carries on into the work.
	coords[followerAxis] = constrain<float>(wanted,
											move.AxisMinimum((size_t)followerAxis),
											move.AxisMaximum((size_t)followerAxis));
}

void AxisFollower::SyncUserPosition(float userCoords[MaxAxes], const float machineCoords[MaxAxes], const float scaleFactors[MaxAxes]) const noexcept
{
	if (engaged && followerAxis >= 0)
	{
		const float sf = scaleFactors[followerAxis];
		userCoords[followerAxis] = (sf != 0.0) ? machineCoords[followerAxis] / sf : machineCoords[followerAxis];
	}
}

// M604: configure one axis to follow another
GCodeResult AxisFollower::Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException)
{
	GCodes& gcodes = reprap.GetGCodes();
	const char *_ecv_array const axisLetters = gcodes.GetAxisLetters();
	const size_t numVisibleAxes = gcodes.GetVisibleAxes();

	bool seen = false;

	if (gb.Seen('A'))
	{
		String<StringLength20> letter;
		gb.GetQuotedString(letter.GetRef());
		const char *_ecv_array const found = strchr(axisLetters, toupper(letter.c_str()[0]));
		if (found == nullptr || (size_t)(found - axisLetters) >= numVisibleAxes)
		{
			reply.printf("Unknown axis '%s'", letter.c_str());
			return GCodeResult::error;
		}
		followerAxis = found - axisLetters;
		seen = true;
	}

	if (gb.Seen('B'))
	{
		String<StringLength20> letter;
		gb.GetQuotedString(letter.GetRef());
		const char *_ecv_array const found = strchr(axisLetters, toupper(letter.c_str()[0]));
		if (found == nullptr || (size_t)(found - axisLetters) >= numVisibleAxes)
		{
			reply.printf("Unknown axis '%s'", letter.c_str());
			return GCodeResult::error;
		}
		leaderAxis = found - axisLetters;
		seen = true;
	}

	gb.TryGetFValue('S', scale, seen);
	gb.TryGetFValue('O', offset, seen);

	if (gb.Seen('E'))
	{
		const bool wanted = gb.GetUIValue() != 0;
		if (wanted && (followerAxis < 0 || leaderAxis < 0))
		{
			reply.copy("Configure the follower and leader axes before engaging");
			return GCodeResult::error;
		}
		if (wanted && followerAxis == leaderAxis)
		{
			reply.copy("An axis cannot follow itself");
			return GCodeResult::error;
		}
		if (wanted && !gcodes.IsAxisHomed((unsigned int)followerAxis))
		{
			reply.copy("Cannot engage: the follower axis is not homed");
			return GCodeResult::error;
		}

		// Capture the relationship at the moment of engaging unless an explicit offset was given. This
		// is what the G-code implementation used dustShoePrevZ for: engaging means "hold the current
		// separation from here on", not "jump to some absolute relationship".
		if (wanted && !gb.Seen('O'))
		{
			MovementState& ms = gcodes.GetMovementStateForFollower();
			const float leaderNow = ms.LiveMachineCoordinate((unsigned int)leaderAxis);
			const float followerNow = ms.LiveMachineCoordinate((unsigned int)followerAxis);
			offset = followerNow - (scale * leaderNow);
		}

		engaged = wanted;
		seen = true;
#if SUPPORT_ASYNC_MOVES
		if (engaged)
		{
			// The follower has to be owned by the movement system or its motion is silently dropped:
			// the coordinate updates and M114 reports the new machine position, but no steps come out,
			// because nothing ever mentioned the axis in a command.
			MovementState& owner = gcodes.GetMovementStateForFollower();
			if (owner.AllocateAxes(AxesBitmap::MakeFromBits((unsigned int)followerAxis), ParameterLettersBitmap()).IsNonEmpty())
			{
				engaged = false;
				reply.copy("Cannot engage: the follower axis is in use by another movement system");
				return GCodeResult::error;
			}
		}
#endif
	}

	if (!seen)
	{
		Report(reply);
	}
	return GCodeResult::ok;
}

void AxisFollower::Report(const StringRef& reply) const noexcept
{
	if (followerAxis < 0 || leaderAxis < 0)
	{
		reply.copy("No axis following configured");
		return;
	}
	const char *_ecv_array const axisLetters = reprap.GetGCodes().GetAxisLetters();
	reply.printf("%c follows %c as %.3f * %c %c %.3f, %s",
					axisLetters[followerAxis], axisLetters[leaderAxis], (double)scale,
					axisLetters[leaderAxis], (offset < 0.0) ? '-' : '+', (double)fabsf(offset),
					(engaged) ? "engaged" : "disengaged");
}

// End
