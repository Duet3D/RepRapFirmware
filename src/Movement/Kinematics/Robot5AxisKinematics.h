/*
 * Robot5AxisKinematics.h
 *
 *  Created on: 7 May 2026
 */

#ifndef SRC_MOVEMENT_KINEMATICS_ROBOT5AXISKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_ROBOT5AXISKINEMATICS_H_

#include "ZLeadscrewKinematics.h"
#include <Math/Matrix.h>

// K13 generalized kinematics supporting both matrix-only mapping and screw-based FK/IK.
class Robot5AxisKinematics : public ZLeadscrewKinematics
{
public:
	// Construct default K13 state with identity matrix map and default IK settings.
	Robot5AxisKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	// Return short kinematics name used in status and reporting.
	const char *_ecv_array GetName(bool forStatusReport) const noexcept override;
	// Parse and apply M669 configuration for matrix, chain, screw and IK parameters.
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	// Convert machine-space endpoint to motor steps, using matrix mode or screw IK based on U parameter.
	MovementError CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	// Reconstruct machine-space coordinates from motor step positions.
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	// Return homing behavior selected by H parameter.
	HomingMode GetHomingMode() const noexcept override { return homingMode; }
	// Return endstop coordinate for a drive, including per-axis A assignments when used.
	float GetEndstopPosition(size_t drive, bool highEnd) noexcept override;
	// Report which axes can be considered homed after G92 for this kinematics.
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	// Report which axes must be homed before allowing motion.
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	// Clamp coordinates to configured limits and check screw-solver reachability.
	LimitPositionResult LimitPosition(float finalCoords[], const float *_ecv_array _ecv_null initialCoords,
													 size_t numVisibleAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	// Apply coupling-aware speed and acceleration limits so no mapped drive exceeds limits.
	void LimitSpeedAndAcceleration(DDA& dda, const float *_ecv_array normalisedDirectionVector, size_t numVisibleAxes, bool continuousRotationShortcut) const noexcept override;
	// Return whether axis should be treated as continuous rotation (R parameter semantics).
	bool IsContinuousRotationAxis(size_t axis) const noexcept override;
	// Return bitset of drives that influence a given axis.
	LogicalDrivesBitmap GetControllingDrives(size_t axis, bool forHoming) const noexcept override;

protected:
	DECLARE_OBJECT_MODEL

private:

	// Compact rigid transform representation: 3x4 row-major matrix.
	struct ChainTransform
	{
		float m[12];
	};

	// Recompute forward matrix inverse and controlling-drive lookup tables.
	void Recalc() noexcept;
	// Return true when an axis maps to more than one logical motor.
	bool HasSharedMotor(size_t axis) const noexcept;
	// Map a visible axis letter (e.g. X/A/B) to visible-axis index, or -1 if missing.
	int FindVisibleAxisByLetter(char c, size_t numVisibleAxes) const noexcept;
	// Map a chain letter in B parameter to chain-joint index, or -1 if missing.
	int FindChainJointByLetter(char c) const noexcept;
	// Validate and rebuild chain letter -> visible axis mapping.
	bool UpdateChainMapping(size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Return true if letter is one of pose axes used in pose-target construction.
	bool IsPoseLetter(char c) const noexcept;
	// Build initial joint vector from machine coordinates/home references.
	void SeedJointVectorFromMachine(const float machinePos[], size_t numVisibleAxes, float jointsOut[]) const noexcept;
	// Build target end-effector pose from machine X/Y/Z and A/B/C values.
	void BuildTargetPose(const float machinePos[], size_t numVisibleAxes, ChainTransform& target) const noexcept;
	// Evaluate forward kinematics for a joint vector.
	void ForwardKinematics(const float joints[], ChainTransform& out) const noexcept;
	// Iteratively solve IK for target machine pose using damped least squares.
	bool SolveInverseKinematics(const float machinePos[], size_t numVisibleAxes, float jointsInOut[]) const noexcept;
	// Apply inverse matrix map from axis-space values to motor steps.
	void ApplyMatrixMap(const float axisPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], MovementError& rslt) const noexcept;
	// Apply forward matrix map from motor steps to axis-space values.
	void ApplyInverseMatrixMap(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float axisPos[]) const noexcept;
	// Extract X/Y/Z and A/B/C values from FK pose matrix.
	void ExtractPoseToAxes(const ChainTransform& pose, float axisPos[], size_t numVisibleAxes) const noexcept;
	// Ensure screw axes are valid and normalize omega vectors.
	bool ValidateAndNormaliseScrews(size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;

	// Parse P string and update per-joint R/P type assignments.
	bool ParseJointTypes(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse R string and update continuous-rotation axis flags.
	bool ParseContinuousAxes(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse A assignments and update per-axis homing coordinates.
	bool ParseHomeAssignments(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse B chain-order string and update joint order.
	bool ParseChain(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse C screw definitions (omega and q) for each chain joint.
	bool ParseScrewDefinitions(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse L limit/home tuples for each chain joint.
	bool ParseJointLimits(const char* s, size_t numVisibleAxes, const StringRef& reply, bool& error) noexcept;
	// Parse M tool transform 3x4 matrix.
	bool ParseToolTransform(const char* s, const StringRef& reply, bool& error) noexcept;

	FixedMatrix<float, MaxAxes, MaxAxes> inverseMatrix;
	FixedMatrix<float, MaxAxes, MaxAxes> forwardMatrix;
	LogicalDrivesBitmap controllingDrivers[MaxAxes];

	float screwOmega[MaxAxes][3];
	float screwQ[MaxAxes][3];
	float jointMin[MaxAxes];
	float jointMax[MaxAxes];
	float jointHome[MaxAxes];
	mutable float lastJointSolution[MaxAxes];
	int8_t chainAxisMap[MaxAxes];
	char chainLetters[MaxAxes + 1];
	float homePositions[MaxAxes];
	char jointTypes[MaxAxes + 1];
	char chainDescription[80];
	ChainTransform toolTransform;

	bool hasScrew[MaxAxes];
	bool hasJointLimits[MaxAxes];
	bool hasHomePositions[MaxAxes];
	bool continuousRotation[MaxAxes];
	bool useScrewSolver;
	mutable bool hasLastJointSolution;
	bool modified;
	uint8_t firstMotor[MaxAxes], lastMotor[MaxAxes];
	uint8_t firstAxis[MaxAxes], lastAxis[MaxAxes];
	uint16_t ikIterationsLimit;
	float ikDamping;
	float ikTolerance;
	float ikRotationWeight;
	mutable uint32_t ikSolveCount;
	mutable uint32_t ikFailCount;
	mutable float lastIkResidual;
	HomingMode homingMode;
};

#endif /* SRC_MOVEMENT_KINEMATICS_ROBOT5AXISKINEMATICS_H_ */
