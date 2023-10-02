/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#include <RepRapFirmware.h>
#include <General/NamedEnum.h>
#include <ObjectModel/ObjectModel.h>

// These names must be in alphabetical order and lowercase
NamedEnum(InputShaperType, uint8_t,
	custom,
	ei2,
	ei3,
	mzv,
	none,
	zvd,
	zvdd,
	zvddd,
);

namespace InputShapingDebugFlags
{
	// Bit numbers in the input shaping debug bitmap
	constexpr unsigned int Errors = 0;
	constexpr unsigned int Retries = 1;
	constexpr unsigned int All = 2;
}

class DDA;
class PrepParams;
class MoveSegment;
struct AccelOrDecelPlan;

#if SUPPORT_REMOTE_COMMANDS
struct CanMessageSetInputShaping;
struct CanMessageMovementLinear;
#endif

class AxisShaper INHERIT_OBJECT_MODEL
{
public:
	AxisShaper() noexcept;

	// Configure input shaping
	GCodeResult Configure(GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);	// process M593

#if SUPPORT_REMOTE_COMMANDS
	// Handle a request from the master board to set input shaping parameters
	GCodeResult EutSetInputShaping(const CanMessageSetInputShaping& msg, size_t dataLength, const StringRef& reply) noexcept;
#endif

	// Calculate the move segments when input shaping is not used
	static MoveSegment *GetUnshapedSegments(DDA& dda, const PrepParams& params) noexcept;

	void Diagnostics(MessageType mtype) noexcept;

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	static constexpr unsigned int MaxExtraImpulses = 4;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.05;

	// Input shaping parameters input by the user
	InputShaperType type;								// the type of the input shaper, from which we can find its name
	float frequency;									// the undamped frequency in Hz
	float zeta;											// the damping ratio, see https://en.wikipedia.org/wiki/Damping. 0 = undamped, 1 = critically damped.

	// Parameters that fully define the shaping
	unsigned int numExtraImpulses;						// the number of extra impulses
	float coefficients[MaxExtraImpulses];				// the coefficients of all the impulses
	float interval;										// the interval in step clocks between impulses

};

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
