/*
 * Kinematics.h
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_H_

#include "RepRapFirmware.h"
#include "Math/Matrix.h"

inline floatc_t fcsquare(floatc_t a)
{
	return a * a;
}

// Different types of kinematics we support. Each of these has a class to represent it.
// These must have the same numeric assignments as the K parameter of the M669 command, as documented in the GCodes wiki page
enum class KinematicsType : uint8_t
{
	cartesian = 0,
	coreXY,
	coreXZ,
	linearDelta,
	scara,
	coreXYU,
	hangprinter,
	polar,
	coreXYUV,
	reserved,			// reserved for @sga, see https://forum.duet3d.com/topic/5775/aditional-carterian-z-axis-on-delta-printer
	rotaryDelta,
	markForged,
	collinearTriperon,	// reserved for @oliof, see https://forum.duet3d.com/topic/11646/kinematics-type-number-allocation-for-colinear-tripteron

	unknown				// this one must be last!
};

// Different types of low-level motion we support
enum class MotionType : uint8_t
{
	linear,
	segmentFreeDelta
};

// Class used to define homing mode
enum class HomingMode : uint8_t
{
	homeCartesianAxes,
	homeIndividualMotors,
	homeSharedMotors
};

// Return value from limitPosition
enum class LimitPositionResult : uint8_t
{
	ok,									// the final position is reachable, so are all the intermediate positions
	adjusted,							// the final position was unreachable so it has been limited, the intermediate positions are now reachable
	intermediateUnreachable,			// the final position is reachable but intermediate positions are not
	adjustedAndIntermediateUnreachable	// we adjusted the final position to make it reachable, but intermediate positions are still urreachable
};

class Kinematics
{
public:
	// Functions that must be defined in each derived class that implements a kinematics

	// Return the name of the current kinematics.
	// If 'forStatusReport' is true then the string must be the one for that kinematics expected by DuetWebControl and PanelDue.
	// Otherwise it should be in a format suitable for printing.
	// For any new kinematics, the same string can be returned regardless of the parameter.
	virtual const char *GetName(bool forStatusReport = false) const = 0;

	// Set or report the parameters from a M665, M666 or M669 command
	// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 665, 666, 667 or 669)
	// then search for parameters used to configure the current kinematics. If any are found, perform appropriate actions,
	// and return true if the changes affect the geometry.
	// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
	// If no relevant parameters are found, print the existing ones to 'reply' and return false.
	// If 'mCode' does not apply to this kinematics, call the base class version of this function, which will print a suitable error message.
	virtual bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error);

	// Convert Cartesian coordinates to motor positions measured in steps from reference position
	// 'machinePos' is a set of axis and extruder positions to convert
	// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
	// 'numAxes' is the number of machine axes to convert, which will always be at least 3
	// 'motorPos' is the output vector of motor positions
	// Return true if successful, false if we were unable to convert
	virtual bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const = 0;

	// Convert motor positions (measured in steps from reference position) to Cartesian coordinates
	// 'motorPos' is the input vector of motor positions
	// 'stepsPerMm' is as configured in M92. On a Scara or polar machine this would actually be steps per degree.
	// 'numDrives' is the number of machine drives to convert, which will always be at least 3
	// 'machinePos' is the output set of converted axis and extruder positions
	virtual void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const = 0;

	// Return true if the kinematics supports auto calibration based on bed probing.
	// Normally returns false, but overridden for delta kinematics and kinematics with multiple independently-drive Z leadscrews.
	virtual bool SupportsAutoCalibration() const { return false; }

	// Perform auto calibration. Override this implementation in kinematics that support it. Caller already owns the movement lock.
	// Return true if an error occurred.
	virtual bool DoAutoCalibration(size_t numFactors, const RandomProbePointSet& probePoints, const StringRef& reply)
	pre(SupportsAutoCalibration()) { return false; }

	// Set the default parameters that are changed by auto calibration back to their defaults.
	// Do nothing if auto calibration is not supported.
	virtual void SetCalibrationDefaults() { }

	// Write the parameters that are set by auto calibration to the config-override.g file, returning true if success
	// Just return true if auto calibration is not supported.
	virtual bool WriteCalibrationParameters(FileStore *f) const { return true; }

	// Get the bed tilt fraction for the specified axis.
	// Usually this is only relevant if we are auto calibrating the bed tilt, however you can also specify bed tilt manually if you wanted to.
	virtual float GetTiltCorrection(size_t axis) const { return 0.0; }

	// Return true if the specified XY position is reachable by the print head reference point.
	// The default implementation assumes a rectangular reachable area, so it just uses the bed dimensions give in the M208 commands.
	virtual bool IsReachable(float x, float y, bool isCoordinated) const;

	// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
	// The default implementation just applies the rectangular limits set up by M208 to those axes that have been homed.
	// applyM208Limits determines whether the m208 limits are applied, otherwise just the geometric limitations of the architecture are applied.
	// If initialCoords is null, just limit the final coordinates; else limit all points on a straight line between the two.
	virtual LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated, bool applyM208Limits) const;

	// Return the set of axes that must have been homed before bed probing is allowed
	// The default implementation requires just X and Y, but some kinematics require additional axes to be homed (e.g. delta, CoreXZ)
	virtual AxesBitmap AxesToHomeBeforeProbing() const { return MakeBitmap<AxesBitmap>(X_AXIS) | MakeBitmap<AxesBitmap>(Y_AXIS); }

	// Return the initial Cartesian coordinates we assume after switching to this kinematics
	virtual void GetAssumedInitialPosition(size_t numAxes, float positions[]) const;

	// Override this one if any axes do not use the linear motion code (e.g. for segmentation-free delta motion)
	virtual MotionType GetMotionType(size_t axis) const { return MotionType::linear; }

	// Override this if the number of homing buttons (excluding the home all button) is not the same as the number of visible axes (e.g. on a delta printer)
	virtual size_t NumHomingButtons(size_t numVisibleAxes) const { return numVisibleAxes; }

	// Override this if the homing buttons are not named after the axes (e.g. SCARA printer)
	virtual const char* HomingButtonNames() const { return "XYZUVWABC"; }

	// This function is called when a request is made to home the axes in 'toBeHomed' and the axes in 'alreadyHomed' have already been homed.
	// If we can't proceed because other axes need to be homed first, return those axes.
	// If we can proceed with homing some axes, set 'filename' to the name of the homing file to be called and return 0. Optionally, update 'alreadyHomed' to indicate
	// that some additional axes should be considered not homed.
	virtual AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const;

	// This function is called from the step ISR when an endstop switch is triggered during homing.
	// Return true if the entire homing move should be terminated, false if only the motor associated with the endstop switch should be stopped.
	virtual bool QueryTerminateHomingMove(size_t axis) const = 0;

	// This function is called from the step ISR when an endstop switch is triggered during homing after stopping just one motor or all motors.
	// Take the action needed to define the current position, normally by calling dda.SetDriveCoordinate() and return false.
	virtual void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const = 0;

	// Return the type of homing we do
	virtual HomingMode GetHomingMode() const = 0;

	// Return the axes that we can assume are homed after executing a G92 command to set the specified axis coordinates
	// This default is good for Cartesian and Core printers, but not deltas or SCARA
	virtual AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const { return g92Axes; }

	// Return the set of axes that must be homed prior to regular movement of the specified axes
	// This default is good for Cartesian and Core printers, but not deltas or SCARA
	virtual AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const { return (disallowMovesBeforeHoming) ? axesMoving : 0; }

	// Write any calibration data that we need to resume a print after power fail, returning true if successful. Override where necessary.
	virtual bool WriteResumeSettings(FileStore *f) const { return true; }

	// Limit the speed and acceleration of a move to values that the mechanics can handle.
	// The speeds along individual Cartesian axes have already been limited before this is called.
	virtual void LimitSpeedAndAcceleration(DDA& dda, const float *normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const = 0;

	// Return true if the specified axis is a continuous rotation axis
	virtual bool IsContinuousRotationAxis(size_t axis) const { return false; }

	// Return a bitmap of the motors that cause movement of a particular axis or tower.
	// This is used to determine which motors we need to enable to move a particular axis, and which motors to monitor for stall detect homing.
	// For example, the first XY move made by a CoreXY machine may be a diagonal move, and it's important to enable the non-moving motor too.
	virtual AxesBitmap GetConnectedAxes(size_t axis) const;

	// Return a bitmap of axes that move linearly in response to the correct combination of linear motor movements.
	// This is called to determine whether we can babystep the specified axis independently of regular motion.
	virtual AxesBitmap GetLinearAxes() const = 0;

	// Override this virtual destructor if your constructor allocates any dynamic memory
	virtual ~Kinematics() { }

	// Factory function to create a particular kinematics object and return a pointer to it.
	// When adding new kinematics, you will need to extend this function to handle your new kinematics type.
	static Kinematics *Create(KinematicsType k);

	// Functions that return information held in this base class
	KinematicsType GetKinematicsType() const { return type; }

	bool UseSegmentation() const { return useSegmentation; }
	bool UseRawG0() const { return useRawG0; }
	float GetSegmentsPerSecond() const pre(UseSegmentation()) { return segmentsPerSecond; }
	float GetMinSegmentLength() const pre(UseSegmentation()) { return minSegmentLength; }

protected:
	// Constructor. Pass segsPerSecond <= 0.0 to get non-segmented motion.
	Kinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0);

	// Apply the M208 limits to the Cartesian position that the user wants to move to for all axes from the specified one upwards
	// Return true if any coordinates were changed
	bool LimitPositionFromAxis(float coords[], size_t firstAxis, size_t numVisibleAxes, AxesBitmap axesHomed) const;

	// Debugging functions
	static void PrintMatrix(const char* s, const MathMatrix<float>& m, size_t numRows = 0, size_t maxCols = 0);
	static void PrintMatrix(const char* s, const MathMatrix<double>& m, size_t numRows = 0, size_t maxCols = 0);
	static void PrintVector(const char *s, const float *v, size_t numElems);
	static void PrintVector(const char *s, const double *v, size_t numElems);

	float segmentsPerSecond;				// if we are using segmentation, the target number of segments/second
	float minSegmentLength;					// if we are using segmentation, the minimum segment size

	static const char * const HomeAllFileName;

private:
	bool useSegmentation;					// true if we have to approximate linear movement using segmentation
	bool useRawG0;							// true if we normally use segmentation but we do not need to segment travel moves
	KinematicsType type;
};

#endif /* SRC_MOVEMENT_KINEMATICS_H_ */
