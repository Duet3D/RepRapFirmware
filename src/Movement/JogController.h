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

	// Measured on the emulator, timing from command injection to the step pins changing rate:
	//   D=5 P=50 -> 257ms    D=3 P=20 -> 126ms    D=2 P=20 -> 50ms
	//   D=2 P=15 ->  62ms    D=2 P=10 -> 127ms
	// Latency stops following D*P below about 40ms of queued motion and then gets worse, because Move
	// wants roughly MoveTiming::UsualMinimumPreparedTime queued before it will run moves. So P=20/D=2
	// is an optimum rather than a compromise - shortening either makes it slower, not faster.
	// Doubling the command rate changed nothing (50.3 -> 50.0ms), so this is the firmware, not the host.
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
