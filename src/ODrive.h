#ifndef SRC_ODRIVEUART_H
#define SRC_ODRIVEUART_H

#include "RepRapFirmware.h"


enum ODriveAxis { M0=0, M1=1, NO_AXIS=-1 };

class ODrive {
public:
	ODrive();
	ODrive(Stream*);
	ODrive(size_t, size_t);
	ODrive(size_t, size_t, Stream*);

	enum AxisState_t {
		AXIS_STATE_UNDEFINED = 0, //<! will fall through to idle
		AXIS_STATE_IDLE = 1, //<! disable PWM and do nothing
		AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
		AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3, //<! run all calibration procedures, then idle
		AXIS_STATE_MOTOR_CALIBRATION = 4, //<! run motor calibration
		AXIS_STATE_SENSORLESS_CONTROL = 5, //<! run sensorless control
		AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
		AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
		AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
	};

	enum ControlState_t {
		CTRL_MODE_VOLTAGE_CONTROL = 0,
		CTRL_MODE_CURRENT_CONTROL = 1,
		CTRL_MODE_VELOCITY_CONTROL = 2,
		CTRL_MODE_POSITION_CONTROL = 3,
		CTRL_MODE_TRAJECTORY_CONTROL = 4
	};

	// Commands
	void SetSerial(Stream* serial_ptr_in);
	void SetPosition(ODriveAxis motorNumber, float position) const; // TODO: this doesn't seem to work...

	// Hangprinter needs
	void SetCtrlMode(ODriveAxis axis, int ctrlMode) const;
	void EnableCurrentControlMode(ODriveAxis motorNumber) const; // set <axis>.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
	void EnablePositionControlMode(ODriveAxis motorNumber) const; // set <axis>.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	void SetCurrent(ODriveAxis motorNumber, float current) const; // set <axis>.controller.current_setpoint = <current_in_A>
	void SetPosSetpoint(ODriveAxis axis, float position) const;
	void flush() const;
	float AskForEncoderPosEstimate(ODriveAxis axis) const; // read <axis>.encoder.pos_estimate
	int32_t AskForEncoderConfigCountsPerRev(ODriveAxis axis) const; // read <axis>.encoder.config.cpr

	// ODrive Firmware can't currently store reference points so this is stored locally in the RRF ODrive object
	float GetEncoderPosReference(ODriveAxis axis) const { return encoderPosReference[axis]; }
	void StoreEncoderPosReference(ODriveAxis axis);
	float GetCountsPerRev(ODriveAxis axis) const { return countsPerRev[axis]; };
	void StoreCountsPerRev(ODriveAxis axis);

	// General params
	float readFloat() const;
	int32_t readInt() const;

	// State helper
	bool run_state(ODriveAxis axis, int requested_state, bool wait) const;

	ODriveAxis AxisToODriveAxis(size_t axis) const; // arg is RRF axis
	void SetRRFToODriveAxis(ODriveAxis odrvAxis, size_t axis);
	size_t GetAxisMap(ODriveAxis odrvAxis) { return axes[odrvAxis]; };

private:
	float encoderPosReference[2]; // One ODrive board can control two axes
	float countsPerRev[2];
	Stream* serial_ptr;
	size_t axes[2];

	size_t readString(char*, size_t) const;
};

#endif
