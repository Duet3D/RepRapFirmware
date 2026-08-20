/*
 * JogController.h
 *
 * Velocity-mode movement: axes are commanded by signed speed rather than by destination, so that a
 * joystick or similar analogue input can drive them directly.
 *
 * The commanded velocity vector is turned into a stream of short constant-velocity moves that are fed
 * into movement system 0. Lookahead blends consecutive chunks, and because the last move in the ring is
 * always planned to end at zero speed, a stream that stops arriving decelerates the machine normally
 * instead of stopping it dead.
 */

#ifndef SRC_MOVEMENT_JOGCONTROLLER_H_
#define SRC_MOVEMENT_JOGCONTROLLER_H_

#include <RepRapFirmware.h>

class MovementState;

class JogController
{
public:
	JogController() noexcept;

	GCodeResult ProcessM700(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	void Spin() noexcept;								// keep the movement queue topped up; called from GCodes::Spin
	void Stop() noexcept;								// stop jogging; queued motion decelerates to a halt
	bool IsActive() const noexcept { return active; }

private:
	bool GenerateChunk(MovementState& ms) noexcept;
	float MaxSpeedForAxis(size_t axis) const noexcept;
	void ClampSpeeds() noexcept;
	void ReportStatus(const StringRef& reply) const noexcept;

	static constexpr uint32_t DefaultChunkMillis = 50;
	static constexpr uint32_t MinChunkMillis = 10;
	static constexpr uint32_t MaxChunkMillis = 200;
	static constexpr uint32_t DefaultTimeoutMillis = 250;
	static constexpr uint32_t MaxTimeoutMillis = 10000;
	static constexpr unsigned int DefaultMaxQueuedMoves = 3;
	static constexpr unsigned int MinMaxQueuedMoves = 2;
	static constexpr unsigned int MaxMaxQueuedMoves = 8;

	float requestedSpeeds[MaxAxes];						// signed commanded speed per axis, in mm (or degrees) per step clock
	AxesBitmap jogAxes;									// the axes with a non-zero commanded speed
	uint32_t chunkMillis;								// how much travel time one chunk represents
	uint32_t chunkClocks;								// the same, in step clocks
	uint32_t timeoutMillis;								// speeds are zeroed if no fresh command arrives within this time
	uint32_t whenLastCommanded;
	unsigned int maxQueuedMoves;						// bounds both the response latency and the distance available to stop in
	volatile bool active;								// written by the GCode task, read by the same task only, but kept volatile for clarity
};

#endif /* SRC_MOVEMENT_JOGCONTROLLER_H_ */
