/*
 * HangprinterKinematics.h
 *
 *  Created on: 24 Nov 2017
 *      Author: David
 */

#ifndef SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_
#define SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_

#include "RoundBedKinematics.h"

#if SUPPORT_HANGPRINTER

// Different modes can be configured for different tradeoffs in terms of printing volumes and speeds
enum class HangprinterAnchorMode {
	None, // All is reacheable in None anchor mode as printing volume
	LastOnTop, // (Default) Results in a pyramid plus a prism below if the lower anchors are above the printing bed
	AllOnTop, // Result in a prism (speeds get limited, specially going down in Z)
};

class HangprinterKinematics : public RoundBedKinematics
{
public:
	// Constructors
	HangprinterKinematics() noexcept;

	// Overridden base class functions. See Kinematics.h for descriptions.
	const char *GetName(bool forStatusReport) const noexcept override;
	bool Configure(unsigned int mCode, GCodeBuffer& gb, const StringRef& reply, bool& error) THROWS(GCodeException) override;
	bool CartesianToMotorSteps(const float machinePos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, int32_t motorPos[], bool isCoordinated) const noexcept override;
	void MotorStepsToCartesian(const int32_t motorPos[], const float stepsPerMm[], size_t numVisibleAxes, size_t numTotalAxes, float machinePos[]) const noexcept override;
	bool SupportsAutoCalibration() const noexcept override { return true; }
	bool IsReachable(float axesCoords[MaxAxes], AxesBitmap axes) const noexcept override;
	void SetCalibrationDefaults() noexcept override { Init(); }
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteCalibrationParameters(FileStore *f) const noexcept override;
#endif
	LimitPositionResult LimitPosition(float finalCoords[], const float * null initialCoords, size_t numAxes, AxesBitmap axesToLimit, bool isCoordinated, bool applyM208Limits) const noexcept override;
	void GetAssumedInitialPosition(size_t numAxes, float positions[]) const noexcept override;
	HomingMode GetHomingMode() const noexcept override { return HomingMode::homeIndividualMotors; }
	AxesBitmap AxesAssumedHomed(AxesBitmap g92Axes) const noexcept override;
	AxesBitmap MustBeHomedAxes(AxesBitmap axesMoving, bool disallowMovesBeforeHoming) const noexcept override;
	AxesBitmap GetHomingFileName(AxesBitmap toBeHomed, AxesBitmap alreadyHomed, size_t numVisibleAxes, const StringRef& filename) const noexcept override;
	bool QueryTerminateHomingMove(size_t axis) const noexcept override;
	void OnHomingSwitchTriggered(size_t axis, bool highEnd, const float stepsPerMm[], DDA& dda) const noexcept override;
#if HAS_MASS_STORAGE || HAS_SBC_INTERFACE
	bool WriteResumeSettings(FileStore *f) const noexcept override;
#endif
#if DUAL_CAN
	GCodeResult ReadODrive3AxisForce(DriverId driver, const StringRef& reply,
																					float setTorqueConstants[] = nullptr, uint32_t setMechanicalAdvantage[] = nullptr,
																					uint32_t setSpoolGearTeeth[] = nullptr, uint32_t setMotorGearTeeth[] = nullptr,
																					float setSpoolRadii[] = nullptr) THROWS(GCodeException);
	GCodeResult ReadODrive3Encoder(DriverId driver, GCodeBuffer& gb, const StringRef& reply) THROWS(GCodeException);
	GCodeResult SetODrive3TorqueMode(DriverId driver, float force_Newton, const StringRef& reply,
																					uint32_t setMechanicalAdvantage[] = nullptr,
																					uint32_t setSpoolGearTeeth[] = nullptr, uint32_t setMotorGearTeeth[] = nullptr,
																					float setSpoolRadii[] = nullptr) noexcept;
#endif

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

	bool IsInsidePyramidSides(float const coords[3]) const noexcept;
	bool IsInsidePrismSides(float const coords[3], unsigned const discount_last) const noexcept;

private:
	// Basic facts about movement system
	static constexpr const char* ANCHOR_CHARS = "ABCDIJKLO";
	static constexpr size_t HANGPRINTER_MAX_ANCHORS = 5;
	static constexpr size_t DefaultNumAnchors = 4;

	void Init() noexcept;
	void Recalc() noexcept;
	void ForwardTransform(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept;
	void ForwardTransformTetrahedron(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept;
	void ForwardTransformQuadrilateralPyramid(float const distances[HANGPRINTER_MAX_ANCHORS], float machinePos[3]) const noexcept;
	float MotorPosToLinePos(const int32_t motorPos, size_t axis) const noexcept;

	void PrintParameters(const StringRef& reply) const noexcept;			// Print all the parameters for debugging

	// The real defaults are in the cpp file
	HangprinterAnchorMode anchorMode = HangprinterAnchorMode::LastOnTop;
	size_t numAnchors = DefaultNumAnchors;
	float printRadius = 0.0F;
	float anchors[HANGPRINTER_MAX_ANCHORS][3];

	// Line buildup compensation configurables

	/* The real defaults are in the Init() function, since we sometimes need to reset
	 * defaults during runtime */
	float spoolBuildupFactor = 0.0F;
	float spoolRadii[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	uint32_t mechanicalAdvantage[HANGPRINTER_MAX_ANCHORS] = { 0 };
	uint32_t linesPerSpool[HANGPRINTER_MAX_ANCHORS] = { 0 };
	uint32_t motorGearTeeth[HANGPRINTER_MAX_ANCHORS] = { 0 };
	uint32_t spoolGearTeeth[HANGPRINTER_MAX_ANCHORS] = { 0 };
	uint32_t fullStepsPerMotorRev[HANGPRINTER_MAX_ANCHORS] = { 0 };

	// Flex compensation configurables
	float moverWeight_kg = 0.0F;
	float springKPerUnitLength = 0.0F;
	float minPlannedForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float maxPlannedForce_Newton[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float guyWireLengths[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float targetForce_Newton = 0.0F;
	float torqueConstants[HANGPRINTER_MAX_ANCHORS] = { 0.0F };

	// Derived parameters
	float k0[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float spoolRadiiSq[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float k2[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float distancesOrigin[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float springKsOrigin[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float relaxedSpringLengthsOrigin[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float fOrigin[HANGPRINTER_MAX_ANCHORS] = { 0.0F };
	float printRadiusSquared = 0.0F;

	float SpringK(float const springLength) const noexcept;
	void StaticForces(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;
	void StaticForcesTetrahedron(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;
	void StaticForcesQuadrilateralPyramid(float const machinePos[3], float F[HANGPRINTER_MAX_ANCHORS]) const noexcept;
	void flexDistances(float const machinePos[3], float const distances[HANGPRINTER_MAX_ANCHORS],
	                   float flex[HANGPRINTER_MAX_ANCHORS]) const noexcept;

#if DUAL_CAN
	// Some CAN helpers
	struct ODriveAnswer {
		bool valid = false;
		float value = 0.0;
	};
	static ODriveAnswer GetODrive3MotorCurrent(DriverId driver, const StringRef& reply) THROWS(GCodeException);
	ODriveAnswer GetODrive3EncoderEstimate(DriverId driver, bool makeReference, const StringRef& reply, bool subtractReference) THROWS(GCodeException);
	static GCodeResult SetODrive3TorqueModeInner(DriverId driver, float torque_Nm, const StringRef& reply) noexcept;
	GCodeResult SetODrive3PosMode(DriverId driver, const StringRef& reply) noexcept;
#endif
};

// Protocol copied from ODrive Firmware source:
// https://github.com/odriverobotics/ODrive/blob/0256229b229255551c183afc0df390111ae1fa52/Firmware/communication/can/can_simple.hpp
// and
// https://github.com/odriverobotics/ODrive/blob/0256229b229255551c183afc0df390111ae1fa52/GUI/src/assets/odriveEnums.json
class CANSimple {
	public:
	enum {
		MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
		MSG_ODRIVE_HEARTBEAT,
		MSG_ODRIVE_ESTOP,
		MSG_GET_MOTOR_ERROR,  // Errors
		MSG_GET_ENCODER_ERROR,
		MSG_GET_SENSORLESS_ERROR,
		MSG_SET_AXIS_NODE_ID,
		MSG_SET_AXIS_REQUESTED_STATE,
		MSG_SET_AXIS_STARTUP_CONFIG,
		MSG_GET_ENCODER_ESTIMATES,
		MSG_GET_ENCODER_COUNT,
		MSG_SET_CONTROLLER_MODES,
		MSG_SET_INPUT_POS,
		MSG_SET_INPUT_VEL,
		MSG_SET_INPUT_TORQUE,
		MSG_SET_VEL_LIMIT,
		MSG_START_ANTICOGGING,
		MSG_SET_TRAJ_VEL_LIMIT,
		MSG_SET_TRAJ_ACCEL_LIMITS,
		MSG_SET_TRAJ_INERTIA,
		MSG_GET_IQ,
		MSG_GET_SENSORLESS_ESTIMATES,
		MSG_RESET_ODRIVE,
		MSG_GET_VBUS_VOLTAGE,
		MSG_CLEAR_ERRORS,
		MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
	};
	static const int32_t CONTROL_MODE_VOLTAGE_CONTROL = 0;
	static const int32_t CONTROL_MODE_TORQUE_CONTROL = 1;
	static const int32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
	static const int32_t CONTROL_MODE_POSITION_CONTROL = 3;
	static const int32_t INPUT_MODE_INACTIVE = 0;
	static const int32_t INPUT_MODE_PASSTHROUGH = 1;
	static const int32_t INPUT_MODE_VEL_RAMP = 2;
	static const int32_t INPUT_MODE_POS_FILTER = 3;
	static const int32_t INPUT_MODE_MIX_CHANNELS = 4;
	static const int32_t INPUT_MODE_TRAP_TRAJ = 5;
	static const int32_t INPUT_MODE_TORQUE_RAMP = 6;
	static const int32_t INPUT_MODE_MIRROR = 7;
	static const int32_t INPUT_MODE_TUNING = 8;
};

#endif // SUPPORT_HANGPRINTER

#endif /* SRC_MOVEMENT_KINEMATICS_HANGPRINTERKINEMATICS_H_ */
