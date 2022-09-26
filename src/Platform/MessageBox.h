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

// Message box data
class MessageBox INHERIT_OBJECT_MODEL
{
public:
	MessageBox() noexcept : seq(0), active(false) { }

	// Set a message box
	void SetAlert(const char *msg, const char *p_title, int p_mode, float p_timeout, AxesBitmap p_controls) noexcept;

	// This is called periodically to handle timeouts
	void Spin() noexcept;

	// This is called to clear any pending message box
	void ClearAlert() noexcept;

	ReadWriteLock& GetLock() const noexcept { return lock; }
	bool IsActive() const noexcept { return active; }
	bool IsActiveLegacy() const noexcept { return active && mode <= 3; }	// legacy M407 and rr_status etc. don't handle higher message box modes

	ReadWriteLock* GetObjectLock(unsigned int tableNumber) const noexcept override { return &lock; }

	AxesBitmap GetControls() const noexcept { return controls; }
	int GetMode() const noexcept { return mode; }
	uint32_t GetSeq() const noexcept { return seq; }
	const char *GetMessage() const noexcept { return message.c_str(); }
	const char *GetTitle() const noexcept { return title.c_str(); }

	float GetTimeLeft() const noexcept;

protected:
	DECLARE_OBJECT_MODEL

private:
	mutable ReadWriteLock lock;
	String<MaxMessageLength> message;
	String<MaxTitleLength> title;
	int mode;
	uint32_t seq;
	uint32_t timer;
	uint32_t timeout;
	AxesBitmap controls;
	bool active;
};

#endif /* SRC_PLATFORM_MESSAGEBOX_H_ */
