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

uint32_t MessageBox::nextSeq = 0;

// Set a message box
void MessageBox::Populate(const char *msg, const char *p_title, int p_mode, float p_timeout, AxesBitmap p_controls) noexcept
{
	message.copy(msg);
	title.copy(p_title);
	mode = p_mode;
	timer = (p_timeout <= 0.0) ? 0 : millis();
	timeout = roundf(max<float>(p_timeout, 0.0) * 1000.0);
	controls = p_controls;
	++seq;
}

float MessageBox::GetTimeLeft() const noexcept
{
	return (timer != 0) ? ((float)timeout - (float)(millis() - timer)) / 1000.0 : 0.0;
}

// End
