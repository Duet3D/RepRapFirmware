/*
 * InputMonitor.h
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_INPUTMONITOR_H_
#define SRC_ENDSTOPS_INPUTMONITOR_H_

#include <RepRapFirmware.h>

#if SUPPORT_REMOTE_COMMANDS

#include <Hardware/IoPorts.h>
#include <RTOSIface/RTOSIface.h>

struct CanMessageCreateInputMonitor;
struct CanMessageChangeInputMonitor;
struct CanMessageInputChanged;
class CanMessageBuffer;

class InputMonitor
{
public:
	InputMonitor() noexcept { }

	static void Init() noexcept;

	static GCodeResult Create(const CanMessageCreateInputMonitor& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept;
	static GCodeResult Change(const CanMessageChangeInputMonitor& msg, const StringRef& reply, uint8_t& extra) noexcept;

	static uint32_t AddStateChanges(CanMessageInputChanged *msg) noexcept;
	static void ReadInputs(CanMessageBuffer *buf) noexcept;

	static void CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept;
	static void CommonAnalogPortInterrupt(CallbackParameter cbp, uint16_t reading) noexcept;

private:
	bool Activate(bool useInterrupt) noexcept;
	void Deactivate() noexcept;
	void DigitalInterrupt() noexcept;
	void AnalogInterrupt(uint16_t reading) noexcept;
	uint16_t GetAnalogValue() const noexcept;

	static bool Delete(uint16_t hndl) noexcept;
	static ReadLockedPointer<InputMonitor> Find(uint16_t hndl) noexcept;

	InputMonitor *next;
	IoPort port;
	uint32_t whenLastSent;
	uint16_t handle;
	uint16_t minInterval;
	uint16_t threshold;
	bool active;
	volatile bool state;
	volatile bool sendDue;

	static InputMonitor * volatile monitorsList;
	static InputMonitor * volatile freeList;

	static ReadWriteLock listLock;
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
