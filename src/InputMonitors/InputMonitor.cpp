/*
 * InputMonitor.cpp
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#include "InputMonitor.h"

#if SUPPORT_REMOTE_COMMANDS

#include <CanMessageFormats.h>
#include <Hardware/IoPorts.h>
#include <CAN/CanInterface.h>
#include <CanMessageBuffer.h>

InputMonitor * volatile InputMonitor::monitorsList = nullptr;
ReadWriteLock InputMonitor::listLock;

bool InputMonitor::Activate(bool useInterrupt) noexcept
{
	bool ok = true;
	if (!active)
	{
		if (threshold == 0)
		{
			// Digital input
			const irqflags_t flags = IrqSave();
			ok = !useInterrupt || port.AttachInterrupt(CommonDigitalPortInterrupt, InterruptMode::change, CallbackParameter(this));
			state = port.ReadDigital();
			IrqRestore(flags);
		}
		else
		{
			// Analog port
			state = port.ReadAnalog() >= threshold;
			ok =
#if SAME5x
				!useInterrupt || port.SetAnalogCallback(CommonAnalogPortInterrupt, CallbackParameter(this), 1);
#else
				false;			// SAME70 doesn't support SetAnalogCallback yet
#endif
		}
		active = true;
		whenLastSent = millis();
	}

	return ok;
}

void InputMonitor::Deactivate() noexcept
{
	if (active)
	{
		if (threshold == 0)
		{
			port.DetachInterrupt();
		}
		else
		{
#if SAME5x
			port.ClearAnalogCallback();
#endif
		}
	}
	active = false;
}

// Return the analog value of this input
uint16_t InputMonitor::GetAnalogValue() const noexcept
{
	return (threshold != 0) ? port.ReadAnalog()
			: port.ReadDigital() ? 0xFFFF
				: 0;
}

void InputMonitor::DigitalInterrupt() noexcept
{
	const bool newState = port.ReadDigital();
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSender();
		}
	}
}

void InputMonitor::AnalogInterrupt(uint32_t reading) noexcept
{
	const bool newState = reading >= threshold;
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSender();
		}
	}
}

/*static*/ void InputMonitor::Init() noexcept
{
	// Nothing needed here yet
}

/*static*/ void InputMonitor::CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept
{
	static_cast<InputMonitor*>(cbp.vp)->DigitalInterrupt();
}

/*static*/ void InputMonitor::CommonAnalogPortInterrupt(CallbackParameter cbp, uint32_t reading) noexcept
{
	static_cast<InputMonitor*>(cbp.vp)->AnalogInterrupt(reading);
}

/*static*/ ReadLockedPointer<InputMonitor> InputMonitor::Find(uint16_t hndl) noexcept
{
	ReadLocker lock(listLock);
	InputMonitor *current = monitorsList;
	while (current != nullptr && current->handle != hndl)
	{
		current = current->next;
	}
	return ReadLockedPointer<InputMonitor>(lock, current);
}

// Delete a monitor. Must own the write lock before calling this.
/*static*/ bool InputMonitor::Delete(uint16_t hndl) noexcept
{
	InputMonitor *prev = nullptr, *current = monitorsList;
	while (current != nullptr)
	{
		if (current->handle == hndl)
		{
			current->Deactivate();

			// Remove the monitor from the list of monitors
			if (prev == nullptr)
			{
				monitorsList = current->next;
			}
			else
			{
				prev->next = current->next;
			}

			delete current;
			return true;
		}

		// Move on to next monitor in list
		prev = current;
		current = current->next;
	}

	return false;
}

/*static*/ GCodeResult InputMonitor::Create(const CanMessageCreateInputMonitorNew& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept
{
	WriteLocker lock(listLock);

	Delete(msg.handle.u.all);						// delete any existing monitor with the same handle

	// Allocate a new one
	InputMonitor *newMonitor = new InputMonitor;

	newMonitor->handle = msg.handle.u.all;
	newMonitor->active = false;
	newMonitor->state = false;
	newMonitor->minInterval = msg.minInterval;
	newMonitor->threshold = msg.threshold;
	newMonitor->sendDue = false;
	String<StringLength50> pinName;
	pinName.copy(msg.pinName, msg.GetMaxPinNameLength(dataLength));
	if (newMonitor->port.AssignPort(pinName.c_str(), reply, PinUsedBy::endstop, (msg.threshold == 0) ? PinAccess::read : PinAccess::readAnalog))
	{
		newMonitor->next = monitorsList;
		monitorsList = newMonitor;
		// Pins used only by the ATE may not have interrupts, and the ATE doesn't need interrupts from them
		const bool ok = newMonitor->Activate(msg.handle.u.parts.type != RemoteInputHandle::typeAte);
		extra = (newMonitor->state) ? 1 : 0;
		if (!ok)
		{
			reply.copy("Failed to set pin change interrupt");
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}

	delete newMonitor;
	return GCodeResult::error;
}

/*static*/ GCodeResult InputMonitor::Change(const CanMessageChangeInputMonitorNew& msg, const StringRef& reply, uint8_t& extra) noexcept
{
	if (msg.action == CanMessageChangeInputMonitorNew::actionDelete)
	{
		WriteLocker lock(listLock);

		if (Delete(msg.handle.u.all))
		{
			return GCodeResult::ok;
		}

		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::warning;					// only a warning when deleting a non-existent handle
	}

	auto m = Find(msg.handle.u.all);
	if (m.IsNull())
	{
		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::error;
	}

	GCodeResult rslt;
	switch (msg.action)
	{
	case CanMessageChangeInputMonitorNew::actionDoMonitor:
		rslt = (m->Activate(true)) ? GCodeResult::ok : GCodeResult::error;
		break;

	case CanMessageChangeInputMonitorNew::actionDontMonitor:
		m->Deactivate();
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionReturnPinName:
		m->port.AppendPinName(reply);
		reply.catf(", min interval %ums", m->minInterval);
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionChangeThreshold:
		m->threshold = msg.param;
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionChangeMinInterval:
		m->minInterval = msg.param;
		rslt = GCodeResult::ok;
		break;

	default:
		reply.printf("ChangeInputMonitor action #%u not implemented", msg.action);
		rslt = GCodeResult::error;
		break;
	}

	extra = (m->state) ? 1 : 0;
	return rslt;
}

// Check the input monitors and add any pending ones to the message
// Return the number of ticks before we should be woken again, or TaskBase::TimeoutUnlimited if we shouldn't be work until an input changes state
/*static*/ uint32_t InputMonitor::AddStateChanges(CanMessageInputChangedNew *msg) noexcept
{
	uint32_t timeToWait = TaskBase::TimeoutUnlimited;
	ReadLocker lock(listLock);

	const uint32_t now = millis();
	for (InputMonitor *p = monitorsList; p != nullptr; p = p->next)
	{
		if (p->sendDue)
		{
			const uint32_t age = now - p->whenLastSent;
			if (age >= p->minInterval)
			{
				bool monitorState;
				{
					InterruptCriticalSectionLocker ilock;
					p->sendDue = false;
					monitorState = p->state;
				}

				if (msg->AddEntry(p->handle, p->GetAnalogValue(), monitorState))
				{
					p->whenLastSent = now;
				}
				else
				{
					p->sendDue = true;
					return 1;
				}
			}
			else
			{
				// The state has changed but we've recently sent a state change for this input
				const uint32_t timeLeft = p->minInterval - age;
				if (timeLeft < timeToWait)
				{
					timeToWait = timeLeft;
				}
			}
		}
	}
	return timeToWait;
}

// Read the specified inputs. The incoming message is a CanMessageReadInputsRequest. We return a CanMessageReadInputsReply in the same buffer.
// Return error if we didn't have any of the requested inputs.
/*static*/ void InputMonitor::ReadInputs(CanMessageBuffer *buf) noexcept
{
	// Extract data before we overwrite the message
	const CanMessageReadInputsRequest& req = buf->msg.readInputsRequest;
	const CanAddress srcAddress = buf->id.Src();
	const uint16_t rid = req.requestId;
	const uint16_t mask = req.mask.u.all;
	const uint16_t pattern = req.pattern.u.all & mask;

	// Construct the new message in the same buffer
	auto reply = buf->SetupResponseMessage<CanMessageReadInputsReply>(rid, CanInterface::GetCanAddress(), srcAddress);

	unsigned int count = 0;
	ReadLocker lock(listLock);
	InputMonitor *h = monitorsList;
	while (h != nullptr && count < ARRAY_SIZE(reply->results))
	{
		if ((h->handle & mask) == pattern)
		{
			reply->results[count].handle.Set(h->handle);
			StoreLEU32(&reply->results[count].reading, h->GetAnalogValue());
			++count;
		}
		h = h->next;
	}

	reply->numReported = count;
	reply->resultCode = (uint32_t)((count == 0) ? GCodeResult::error : GCodeResult::ok);
	buf->dataLength = reply->GetActualDataLength();
}

#endif	// SUPPORT_CAN_EXPANSION

// End
