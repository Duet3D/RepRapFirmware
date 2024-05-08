/*
 * MessageBox.cpp
 *
 *  Created on: 26 Sept 2022
 *      Author: David
 */

#include "MessageBox.h"
#include "RepRap.h"
#include <GCodes/GCodes.h>
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
MessageBox *MessageBox::mboxList = nullptr;
ReadWriteLock MessageBox::mboxLock;
uint32_t MessageBox::startTime;
unsigned int MessageBox::numMessages = 0;
unsigned int MessageBox::numAutoCancelledMessages = 0;

// Create a message box. Caller must have a write lock on the message box lock first.
/*static*/ uint32_t MessageBox::Create(const char *msg, const char *p_title, int p_mode, float p_timeout, AxesBitmap p_controls, MessageBoxLimits *_ecv_null p_limits) noexcept
{
	// Find the end of the message box list. On the way, if we already have the maximum allowed number of messages in the list, time out the oldest non-blocking one.
	// Also reduce the timeout of any remaining non-blocking message boxes to 1 second.
	MessageBox **mbp = &mboxList;
	while (*mbp != nullptr)
	{
		MessageBox *mb = *mbp;
		if (!mb->IsBlocking() && numMessages >= MaxMessageBoxes)
		{
			// Time out this nonblocking message now to make room in the queue
			*mbp = mb->next;
			mb->TimeOut();
			++numAutoCancelledMessages;
		}
		else
		{
			if (!mb->IsBlocking() && mb->timeout > 1000)
			{
				// Time out this non-blocking message after 1 second
				mb->timeout = 1000;
			}
			mbp = &((*mbp)->next);
		}
	}

	if (numMessages >= MaxMessageBoxes)
	{
		// Message box queue is still full, so time out the oldest message even though it is blocking
		MessageBox *const mb = mboxList;
		mboxList = mb->next;
		mb->TimeOut();
		++numAutoCancelledMessages;
	}

	MessageBox *const mbox = new MessageBox(nullptr);
	mbox->message.copy(msg);
	mbox->title.copy(p_title);
	mbox->mode = p_mode;
	mbox->startTime = millis();
	mbox->controls = p_controls;
	if (p_limits != nullptr)
	{
		mbox->limits = *p_limits;
	}

	// Override the canCancel flag if its value is implied by the mode
	if (p_mode == 3)
	{
		mbox->limits.canCancel = true;
	}
	else if (p_mode < 3)
	{
		mbox->limits.canCancel = false;
	}
	mbox->timeout = (p_mode <= 1 || mbox->limits.canCancel) ? lrintf(max<float>(p_timeout, 0.0) * 1000.0) : 0;

	if (mbp == &mboxList)
	{
		startTime = millis();
	}
	*mbp = mbox;
	++numMessages;
	return mbox->seq;
}

float MessageBox::GetTimeLeft() const noexcept
{
	return (timeout != 0) ? ((float)timeout - (float)(millis() - startTime)) / 1000.0 : 0.0;
}

// If we have an active message box with the specified sequence number, close it and tell the caller whether it was blocking or not, and return true
/*static*/ bool MessageBox::Acknowledge(uint32_t ackSeq, bool& wasBlocking) noexcept
{
	if (mboxList != nullptr)
	{
		WriteLocker lock(mboxLock);

		MessageBox **mbp = &mboxList;
		while (*mbp != nullptr)
		{
			MessageBox *mb = *mbp;
			if (ackSeq == 0 || mb->GetSeq() == ackSeq)
			{
				wasBlocking = mb->IsBlocking();
				if (mbp == &mboxList)
				{
					startTime = millis();
				}
				*mbp = mb->next;
				delete mb;
				--numMessages;
				reprap.StateUpdated();
				return true;
			}
			mbp = &((*mbp)->next);
		}
	}

	return false;
}

// Check if the head message box should be timed out, returning true if we timed it out
/*static*/ bool MessageBox::CheckTimeout() noexcept
{
	if (mboxList != nullptr)
	{
		WriteLocker locker(mboxLock);

		MessageBox *mb = mboxList;
		if (mb != nullptr && mb->timeout != 0 && millis() - startTime >= mb->timeout)
		{
			mboxList = mb->next;
			mb->TimeOut();
			startTime = millis();				// restart the timeout for the next message box (if any) in the list
			return true;
		}
	}

	return false;
}

// Time out and delete this message box
void MessageBox::TimeOut() noexcept
{
	if (IsBlocking())
	{
		reprap.GetGCodes().MessageBoxClosed(CanCancel(), false, seq, GetDefaultValue());
	}
	--numMessages;
	delete this;
}

/*static*/ ReadLockedPointer<const MessageBox> MessageBox::GetLockedCurrent() noexcept
{
	ReadLockedPointer<const MessageBox> p(mboxLock, mboxList);
	if (p.IsNull())
	{
		p.Release();
	}
	return p;
}

// MessageBoxLimits members
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
