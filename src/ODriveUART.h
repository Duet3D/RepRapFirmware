#ifndef SRC_ODRIVEUART_H
#define SRC_ODRIVEUART_H

#include "RepRapFirmware.h"

/* The policy is: we don't save run state information about the ODrive unless you have to.
 * We only save information about how to ask for the things we need,
 * and never-changing info about how to interpret them.
 */
class ODriveUART {
public:
	ODriveUART();
	ODriveUART(Stream& serial);

	enum AxisState_t {
		AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
		AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
		AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
		AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
		AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
		AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
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
	void SetSerial(Stream& serial);
	void SetPosition(int motor_number, float position);
	void SetPosition(int motor_number, float position, float velocity_feedforward);
	void SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward);
	void SetVelocity(int motor_number, float velocity);
	void SetVelocity(int motor_number, float velocity, float current_feedforward);

	// Hangprinter needs
	void SetCtrlMode(int axis, int ctrlMode);
	void EnableCurrentControlMode(int motor_number); // set <axis>.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
	void EnablePositionControlMode(int motor_number); // set <axis>.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
	void SetCurrent(int motor_number, float current); // set <axis>.controller.current_setpoint = <current_in_A>
	void SetPosSetpoint(int axis, float position);
	void flush();
	float AskForEncoderPosEstimate(int axis); // read <axis>.encoder.pos_estimate
	int32_t AskForEncoderConfigCpr(int axis); // read <axis>.encoder.config.cpr

	// ODrive Firmware can't currently store reference points so this is stored locally in the RRF ODrive object
	float GetEncoderPosReference() const { return encoderPosReference; }
	float SetEncoderPosReference(const int axis);

	// General params
	float readFloat();
	int32_t readInt();

	// State helper
	bool run_state(int axis, int requested_state, bool wait);
private:
	float encoderPosReference; // One ODrive board can control two axes
	size_t readString(char*, size_t);

	Stream& serial_;
	bool serialAttached;
};

#endif
