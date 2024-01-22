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
#include <General/FreelistManager.h>

struct CanMessageCreateInputMonitorNew;
struct CanMessageChangeInputMonitorNew;
struct CanMessageInputChangedNew;
class CanMessageBuffer;

class InputMonitor
{
public:
	DECLARE_FREELIST_NEW_DELETE(InputMonitor)

	InputMonitor() noexcept { }

	static void Init() noexcept;

	static GCodeResult Create(const CanMessageCreateInputMonitorNew& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept;
	static GCodeResult Change(const CanMessageChangeInputMonitorNew& msg, const StringRef& reply, uint8_t& extra) noexcept;

	static uint32_t AddStateChanges(CanMessageInputChangedNew *msg) noexcept;
	static void ReadInputs(CanMessageBuffer *buf) noexcept;

	static void CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept;
	static void CommonAnalogPortInterrupt(CallbackParameter cbp, uint32_t reading) noexcept;

private:
	bool Activate(bool useInterrupt) noexcept;
	void Deactivate() noexcept;
	void DigitalInterrupt() noexcept;
	void AnalogInterrupt(uint32_t reading) noexcept;
	uint16_t GetAnalogValue() const noexcept;

	static bool Delete(uint16_t hndl) noexcept;
	static ReadLockedPointer<InputMonitor> Find(uint16_t hndl) noexcept;

	InputMonitor *next;
	uint32_t whenLastSent;
	IoPort port;
	uint16_t handle;
	uint16_t minInterval;
	uint32_t threshold;
	bool active;
	volatile bool state;
	volatile bool sendDue;

	static InputMonitor * volatile monitorsList;

	static ReadWriteLock listLock;
};

#endif	// SUPPORT_CAN_EXPANSION

#endif /* SRC_ENDSTOPS_INPUTMONITOR_H_ */
