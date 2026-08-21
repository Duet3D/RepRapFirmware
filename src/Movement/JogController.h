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

	// Latency is dominated by the chunk time already queued ahead of the change: the chunks are a FIFO
	// and a new speed takes effect only once the queued ones have run. Measured on the emulator, command
	// injection to the step pins changing rate, D=2: P=15 -> 44.6ms, P=20 -> 38.5ms, P=25 -> 67.2ms,
	// P=30 -> 90.2ms, and P=10 cannot sustain 15mm/s at all (see MaxSpeedForAxis). So P=20/D=2 is a real
	// optimum: shortening P makes it worse, not better. Sizing the chunk adaptively to the requested
	// speed - short chunk for a slow jog - was tried and is much worse (3->15mm/s: 39.5 -> 75.4ms;
	// 1->3mm/s: 79 -> 245ms). Whatever sets the floor below ~40ms of queued motion, it is not the
	// queue arithmetic, and two plausible culprits were measured and cleared: MoveTiming's preparation
	// window (halving UsualMinimumPreparedTime to 25ms moved 50.3 -> 50.2ms) and the lookahead grace
	// period (M595 R0 is worth about 2ms). Doubling the host command rate changed nothing either.
	static constexpr uint32_t DefaultChunkMillis = 20;
	static constexpr uint32_t MinChunkMillis = 10;
	static constexpr uint32_t MaxChunkMillis = 200;
	static constexpr uint32_t DefaultTimeoutMillis = 250;
	static constexpr uint32_t MaxTimeoutMillis = 10000;
	// 2 with a 20ms chunk measured clean - no stutter over a 20Hz stream - and is what gets latency to
	// 50ms. The earlier stutter at depth 3 was with 50ms chunks, where the ring holds far more time and
	// the producer has correspondingly longer to fall behind.
	static constexpr unsigned int DefaultMaxQueuedMoves = 2;
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
