/*
 * MessageBox.cpp
 *
 *  Created on: 26 Sept 2022
 *      Author: David
 */

#include "MessageBox.h"
#include "RepRap.h"
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// Macro to build a standard lambda function that includes the necessary type conversions
#define OBJECT_MODEL_FUNC(...) OBJECT_MODEL_FUNC_BODY(MessageBox, __VA_ARGS__)
#define OBJECT_MODEL_FUNC_IF(_condition,...) OBJECT_MODEL_FUNC_IF_BODY(MessageBox, _condition,__VA_ARGS__)

constexpr ObjectModelTableEntry MessageBox::objectModelTable[] =
{
	{ "axisControls",			OBJECT_MODEL_FUNC_IF(self->mode == 2 || self->mode == 3, (int32_t)self->controls.GetRaw()),	ObjectModelEntryFlags::important },
	{ "cancelButton",			OBJECT_MODEL_FUNC(self->limits.canCancel),						ObjectModelEntryFlags::important },
	{ "choices",				OBJECT_MODEL_FUNC_IF(self->mode == 4, self->limits.choices),	ObjectModelEntryFlags::important },
	{ "default",				OBJECT_MODEL_FUNC_IF(self->mode >= 4, self->limits.defaultVal),	ObjectModelEntryFlags::important },
	{ "max",					OBJECT_MODEL_FUNC_IF(self->mode >= 5, self->limits.maxVal),		ObjectModelEntryFlags::important },
	{ "message",				OBJECT_MODEL_FUNC(self->message.c_str()),						ObjectModelEntryFlags::important },
	{ "min",					OBJECT_MODEL_FUNC_IF(self->mode >= 5, self->limits.minVal),		ObjectModelEntryFlags::important },
	{ "mode",					OBJECT_MODEL_FUNC((int32_t)self->mode),							ObjectModelEntryFlags::important },
	{ "seq",					OBJECT_MODEL_FUNC((int32_t)self->seq),							ObjectModelEntryFlags::important },
	{ "timeout",				OBJECT_MODEL_FUNC((int32_t)self->timeout),						ObjectModelEntryFlags::important },
	{ "title",					OBJECT_MODEL_FUNC(self->title.c_str()),							ObjectModelEntryFlags::important },
};

constexpr uint8_t MessageBox::objectModelTableDescriptor[] =
{
	1,
	11
};

DEFINE_GET_OBJECT_MODEL_TABLE(MessageBox)

uint32_t MessageBox::nextSeq = 0;

// Set a message box
void MessageBox::Populate(const char *msg, const char *p_title, int p_mode, float p_timeout, AxesBitmap p_controls, MessageBoxLimits *_ecv_null p_limits) noexcept
{
	message.copy(msg);
	title.copy(p_title);
	mode = p_mode;
	startTime = millis();
	timeout = lrintf(max<float>(p_timeout, 0.0) * 1000.0);
	controls = p_controls;
	if (p_limits != nullptr)
	{
		limits = *p_limits;
	}

	// Override the canCancel flag if its value is implied by the mode
	if (mode == 3)
	{
		limits.canCancel = true;
	}
	else if (mode < 3)
	{
		limits.canCancel = false;
	}
}

float MessageBox::GetTimeLeft() const noexcept
{
	return (timeout != 0) ? ((float)timeout - (float)(millis() - startTime)) / 1000.0 : 0.0;
}

void MessageBoxLimits::GetIntegerLimits(GCodeBuffer& gb, bool defaultIsString) THROWS(GCodeException)
{
	int32_t iLow, iHigh;
	bool dummy;
	const bool gotLowerLimit = gb.TryGetIValue('L', iLow, dummy);
	const bool gotUpperLimit = gb.TryGetIValue('H', iHigh, dummy);
	if (gotLowerLimit)
	{
		if (gotUpperLimit && iLow > iHigh)
		{
			gb.ThrowGCodeException("H value must not be less than L value");
		}
		minVal.SetInt(iLow);
	}
	if (gotUpperLimit)
	{
		maxVal.SetInt(iHigh);
	}
	if (gb.Seen('F'))
	{
		int32_t iDefault;
		if (defaultIsString)
		{
			String<StringLength100> sDefault;
			gb.GetQuotedString(sDefault.GetRef(), true);
			iDefault = (int32_t)sDefault.strlen();
			defaultVal.SetStringHandle(StringHandle(sDefault.c_str()));
		}
		else
		{
			iDefault = gb.GetIValue();
			defaultVal.SetInt(iDefault);
		}
		if ((gotLowerLimit && iDefault < iLow) || (gotUpperLimit && iDefault > iHigh))
		{
			gb.ThrowGCodeException("default value is outside limits");
		}
	}
}

void MessageBoxLimits::GetFloatLimits(GCodeBuffer& gb) THROWS(GCodeException)
{
	float fLow, fHigh, fDefault;
	bool dummy;
	const bool gotLowerLimit = gb.TryGetFValue('L', fLow, dummy);
	const bool gotUpperLimit = gb.TryGetFValue('H', fHigh, dummy);
	if (gotLowerLimit)
	{
		if (gotUpperLimit && fLow > fHigh)
		{
			gb.ThrowGCodeException("H value must not be less than L value");
		}
		minVal.SetFloat(fLow);
	}
	if (gotUpperLimit)
	{
		maxVal.SetFloat(fHigh);
	}
	const bool gotDefault = gb.TryGetFValue('F', fDefault, dummy);
	if (gotDefault)
	{
		if ((gotLowerLimit && fDefault < fLow) || (gotUpperLimit && fDefault > fHigh))
		{
			gb.ThrowGCodeException("default value is outside limits");
		}
		defaultVal.SetFloat(fDefault);
	}
}

// End
