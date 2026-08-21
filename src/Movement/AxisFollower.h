/*
 * AxisFollower.h
 *
 * Makes one axis track another as part of the same coordinated move, rather than reacting to it.
 *
 * The motivating case is a Z-independent dust shoe on a U axis. Doing that in G-code needs a trigger
 * that watches Z and then issues a U move, so the shoe can only start moving after Z already has, and
 * the correcting move queues behind whatever motion is already planned. Deriving U at the point a
 * move's target is computed instead makes the two axes part of one move: they start, accelerate and
 * stop together because the planner never sees them as separate.
 */

#ifndef SRC_MOVEMENT_AXISFOLLOWER_H_
#define SRC_MOVEMENT_AXISFOLLOWER_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>

class AxisFollower INHERIT_OBJECT_MODEL
{
public:
	AxisFollower() noexcept;

	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);

	// Derive the follower's machine coordinate from the leader's. Called for every move, so it must be
	// cheap and must not depend on anything that changes between segments of the same move.
	void Apply(float coords[MaxAxes]) const noexcept;

	// Bring the follower's user coordinate back in step with the machine coordinate we derived, so that
	// reporting matches reality.
	void SyncUserPosition(float userCoords[MaxAxes], const float machineCoords[MaxAxes], const float scaleFactors[MaxAxes]) const noexcept;

	bool IsEngaged() const noexcept { return engaged; }
	int32_t GetFollowerAxis() const noexcept { return followerAxis; }
	void Disengage() noexcept { engaged = false; }

protected:
	DECLARE_OBJECT_MODEL

private:
	void Report(const StringRef& reply) const noexcept;
	const char *_ecv_array GetAxisLetterOrNull(int32_t axis) const noexcept;

	int32_t followerAxis;					// -1 when unconfigured
	int32_t leaderAxis;
	float scale;							// defaults to -1: a follower carried on the leader must move the opposite way to stay put
	float offset;							// follower = scale * leader + offset, then clamped to its axis limits
	bool engaged;
};

#endif /* SRC_MOVEMENT_AXISFOLLOWER_H_ */
