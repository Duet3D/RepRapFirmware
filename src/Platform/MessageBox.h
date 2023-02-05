/*
 * MessageBox.h
 *
 *  Created on: 26 Sept 2022
 *      Author: David
 */

#ifndef SRC_PLATFORM_MESSAGEBOX_H_
#define SRC_PLATFORM_MESSAGEBOX_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <General/FreelistManager.h>

// Struct to hold minimum, maximum and default values
struct MessageBoxLimits
{
	MessageBoxLimits() noexcept { }
	void GetIntegerLimits(GCodeBuffer& gb, bool defaultIsString) THROWS(GCodeException);
	void GetFloatLimits(GCodeBuffer& gb) THROWS(GCodeException);

	ExpressionValue minVal, maxVal, defaultVal, choices;
	bool canCancel = false;
};

// Message box data
class MessageBox INHERIT_OBJECT_MODEL
{
public:
	DECLARE_FREELIST_NEW_DELETE(MessageBox)

	// Create a message box, returning the sequence number
	static uint32_t Create(const char *_ecv_array msg, const char *_ecv_array p_title, int p_mode, float p_timeout, AxesBitmap p_controls, MessageBoxLimits *_ecv_null p_limits) noexcept;
	static bool HaveCurrent() noexcept { return mboxList != nullptr; }
	static MessageBox *GetCurrent() noexcept { return mboxList; }
	static ReadLockedPointer<const MessageBox> GetLockedCurrent() noexcept;
	static bool Acknowledge(uint32_t ackSeq, bool& wasBlocking) noexcept;
	static bool CheckTimeout() noexcept;

	// Return true if the mode of this message box is one that the legacy status calls can handle
	bool IsLegacyType() const noexcept { return mode <= 3; }	// legacy M407 and rr_status etc. don't handle higher message box modes

	// Return the length of time before this box should time out
	float GetTimeLeft() const noexcept;

	// Return true if this message box blocks the input that created it
	bool IsBlocking() const noexcept { return mode >= 2; }

	// Return various data about the message box
	AxesBitmap GetControls() const noexcept { return controls; }
	int GetMode() const noexcept { return mode; }
	uint32_t GetSeq() const noexcept { return seq; }
	const char *_ecv_array GetMessage() const noexcept { return message.c_str(); }
	const char *_ecv_array GetTitle() const noexcept { return title.c_str(); }
	ExpressionValue GetDefaultValue() const noexcept { return limits.defaultVal; }
	bool CanCancel() const noexcept { return limits.canCancel; }

	static ReadWriteLock mboxLock;

protected:
	DECLARE_OBJECT_MODEL

private:
	explicit MessageBox(MessageBox *_ecv_null p_next) noexcept : next(p_next) { seq = ++nextSeq; }
	void TimeOut() noexcept;

	MessageBox *_ecv_null next;
	String<MaxMessageLength> message;
	String<MaxTitleLength> title;
	MessageBoxLimits limits;
	int mode;
	uint32_t seq;
	uint32_t timeout;
	AxesBitmap controls;

	static uint32_t nextSeq;
	static MessageBox *_ecv_null mboxList;
	static uint32_t startTime;								// when the message came to the head of the list
	static unsigned int numMessages;
	static unsigned int numAutoCancelledMessages;
};

#endif /* SRC_PLATFORM_MESSAGEBOX_H_ */
