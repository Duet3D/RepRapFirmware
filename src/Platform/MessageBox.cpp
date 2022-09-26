/*
 * MessageBox.cpp
 *
 *  Created on: 26 Sept 2022
 *      Author: David
 */

#include "MessageBox.h"
#include "RepRap.h"

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(MessageBox, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(MessageBox, _condition,__VA_ARGS__)

constexpr ObjectModelTableEntry MessageBox::objectModelTable[] =
{
	{ "axisControls",			OBJECT_MODEL_FUNC((int32_t)self->controls.GetRaw()),			ObjectModelEntryFlags::important },
	{ "message",				OBJECT_MODEL_FUNC(self->message.c_str()),						ObjectModelEntryFlags::important },
	{ "mode",					OBJECT_MODEL_FUNC((int32_t)self->mode),							ObjectModelEntryFlags::important },
	{ "seq",					OBJECT_MODEL_FUNC((int32_t)self->seq),							ObjectModelEntryFlags::important },
	{ "timeout",				OBJECT_MODEL_FUNC((int32_t)self->timeout),						ObjectModelEntryFlags::important },
	{ "title",					OBJECT_MODEL_FUNC(self->title.c_str()),							ObjectModelEntryFlags::important },
};

constexpr uint8_t MessageBox::objectModelTableDescriptor[] =
{
	1,
	6
};

DEFINE_GET_OBJECT_MODEL_TABLE(MessageBox)

// Set a message box
void MessageBox::SetAlert(const char *msg, const char *p_title, int p_mode, float p_timeout, AxesBitmap p_controls) noexcept
{
	WriteLocker locker(lock);
	message.copy(msg);
	title.copy(p_title);
	mode = p_mode;
	timer = (p_timeout <= 0.0) ? 0 : millis();
	timeout = round(max<float>(p_timeout, 0.0) * 1000.0);
	controls = p_controls;
	active = true;
	++seq;
	reprap.StateUpdated();
}

// This is called periodically to handle timeouts. Return true if the data returned by the object model has changed.
void MessageBox::Spin() noexcept
{
	if (active)		// don't get the lock if we don't need to
	{
		WriteLocker locker(lock);
		if (active && timer != 0 && millis() - timer >= timeout)
		{
			active = false;
			reprap.StateUpdated();
		}
	}
}

// This is called to clear any pending message box. Return true if the data returned by the object model has changed.
void MessageBox::ClearAlert() noexcept
{
	WriteLocker locker(lock);
	if (active)
	{
		active = false;
		reprap.StateUpdated();
	}
}

float MessageBox::GetTimeLeft() const noexcept
{
	return (timer != 0) ? (float)timeout / 1000.0 - (float)(millis() - timer) / 1000.0 : 0.0;
}

// End
