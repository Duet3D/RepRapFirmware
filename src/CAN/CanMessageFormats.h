/*
 * CanMessageFormats.h
 *
 *  Created on: 16 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEFORMATS_H_
#define SRC_CAN_CANMESSAGEFORMATS_H_

constexpr unsigned int MaxDriversPerCanSlave = 3;
constexpr unsigned int MaxHeatersPerCanSlave = 6;

// CAN message formats
// We have maximum 64 bytes in a CAN-FD packet.
// The first 4 bytes are the time to start the command, in clock ticks. This field doubles up as a sequence number.
// So the CAN packets sent by the master must be in strict increasing order. If 2 commands in separate packets need to be executed at the same time,
// the second must be delayed by 1 clock tick.
// Some commands are always executed immediately on receipt. For these, the first 4 bytes are still present and serve as a sequence number.

// The next 4 bytes are split as follows:
//  10 bit command number. For many types of command (but not movement commands), this is the corresponding M-code number.
//   6 bit flags. The meaning depends on the command. Unused bits must be set to zero for future compatibility.
//  16 bit map of which of the following command parameters are valid.

// The remaining up to 56 bytes are organised as up to 14 parameters. Each one is either a float, an int32_t or a uint32_t.

// Movement messages may be a little different because of the need to carry a lot of data, however the first dword will still be the start time and it will
// be possible to distinguish a movement command by looking at the next 16 bits.

// This is the layout of the second dword
struct CanMessageTypeWord
{
	uint16_t code : 10,
			 flags : 6;
	uint16_t validParams;
};

// Time sync message, excluding the first word (which is used only as a sequence counter in time sync messages)
struct CanMessageTimeSync
{
	CanMessageTypeWord type;
	uint32_t timeSent;								// when this message was sent
	uint32_t timePreviousMessageAcked;				// when the previous message was acknowledged
};

// Movement message
struct CanMovementMessageNew
{
	union MovementFlags
	{
		uint32_t u32;
		struct
		{
			uint32_t	deltaDrives : 4,				// which drivers are doing delta movement
						pressureAdvanceDrives : 4,		// which drivers have pressure advance applied
						endStopsToCheck : 4,			// which drivers have endstop checks applied
						stopAllDrivesOnEndstopHit : 1;	// whether to stop all drivers when one endstop is hit
		};
	};

	CanMessageTypeWord type;
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;
	float initialSpeedFraction;
	float finalSpeedFraction;

	MovementFlags flags;				// miscellaneous flags and small stuff

	float initialX;						// needed only for delta movement
	float initialY;						// needed only for delta movement
	float finalX;						// needed only for delta movement
	float finalY;						// needed only for delta movement
	float zMovement;					// needed only for delta movement

	struct
	{
		int32_t steps;					// net steps moved
	} perDrive[MaxDriversPerCanSlave];

	void DebugPrint();
};


// Here are the layouts for standard message types, excluding the first dword.

struct CanMessageM140								// also does M104, M141
{
	CanMessageTypeWord type;
	float setPoint[MaxHeatersPerCanSlave];
};

struct CanMessageM303
{
	CanMessageTypeWord type;
	uint32_t heaterNumber;
};

struct CanMessageM305
{
	CanMessageTypeWord type;
	uint32_t heaterNumber;
	uint32_t channel;
	float paramB;
	float paramC;
	int32_t paramD;
	int32_t paramF;
	float paramL;
	float paramH;
	float paramR;
	float paramT;
	uint32_t paramW;

	bool GotParamB() const { return IsBitSet(type.validParams, 0); }
	bool GotParamC() const { return IsBitSet(type.validParams, 1); }
	bool GotParamD() const { return IsBitSet(type.validParams, 2); }
	bool GotParamF() const { return IsBitSet(type.validParams, 3); }
	bool GotParamL() const { return IsBitSet(type.validParams, 4); }
	bool GotParamH() const { return IsBitSet(type.validParams, 5); }
	bool GotParamR() const { return IsBitSet(type.validParams, 6); }
	bool GotParamT() const { return IsBitSet(type.validParams, 7); }
	bool GotParamW() const { return IsBitSet(type.validParams, 8); }
};

#define M305_SET_IF_PRESENT(_val, _msg, _letter) if (_msg.GotParam ## _letter ()) { _val = _msg.param ## _letter; }

struct CanMessageM307
{
	CanMessageTypeWord type;

};

struct CanMessageM350
{
	CanMessageTypeWord type;
	struct MicrostepSetting
	{
		uint32_t microstepping : 16,
				 interpolated : 1;
	};
	MicrostepSetting motorMicrostepping[MaxDriversPerCanSlave];
};

struct CanMessageM569
{
	CanMessageTypeWord type;
	struct DriverModeSetting
	{
		uint32_t driverMode : 3,
				 direction : 1,
				 enablePolarity : 1,
				 stepPulseLength : 5,
				 stepPulseInterval : 5,
				 dirSetupTime : 5,
				 dirHoldTime : 5,
				 blankingTime,
				 offTime,
				 hstart,
				 hend,
				 hdec,
				 tpwmthrs,
				 thigh;
	};
	DriverModeSetting driverModeSettings[MaxDriversPerCanSlave];
};

struct CanMessageM906
{
	CanMessageTypeWord type;
	struct MotorSetting
	{
		uint16_t currentMilliamps;
		uint8_t standstillCurrentPercent;
		uint8_t idleCurrentPercent;
	};
	MotorSetting motorCurrents[MaxDriversPerCanSlave];
};

struct CanMessageM913
{
	CanMessageTypeWord type;
	uint32_t motorCurrentPercentages[MaxDriversPerCanSlave];
};

struct CanMessageM915
{
	CanMessageTypeWord type;
	struct DriverStallSetting
	{

	};
	DriverStallSetting stallSettings[MaxDriversPerCanSlave];
};

// Standard responses

struct CanResponseHeaterStatus
{
	CanMessageTypeWord type;
	uint32_t status;						// 2 bits per heater: off, on, or fault
	struct HeaterStatus
	{
		int16_t actualTemp;
		float setPoint;
	};
	HeaterStatus heaterStatus[MaxHeatersPerCanSlave];
};

// Old style CAN movement message

struct CanMovementMessage
{
	union MovementFlags
	{
		uint32_t u32;
		struct
		{
			uint32_t	deltaDrives : 4,
						pressureAdvanceDrives : 4,
						endStopsToCheck : 4,
						stopAllDrivesOnEndstopHit : 1;
		};
	};

	// Timing information
	uint32_t timeNow;					// the time now, temporary until we have separate clock sync messages
	uint32_t moveStartTime;				// when this move should start
	uint32_t accelerationClocks;
	uint32_t steadyClocks;
	uint32_t decelClocks;
	float initialSpeedFraction;
	float finalSpeedFraction;

	MovementFlags flags;				// miscellaneous flags and small stuff

	float initialX;						// needed only for delta movement
	float initialY;						// needed only for delta movement
	float finalX;						// needed only for delta movement
	float finalY;						// needed only for delta movement
	float zMovement;					// needed only for delta movement

	struct
	{
		int32_t steps;					// net steps moved
	} perDrive[MaxDriversPerCanSlave];

	void DebugPrint();
};

#endif /* SRC_CAN_CANMESSAGEFORMATS_H_ */
