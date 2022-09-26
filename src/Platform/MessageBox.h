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
	bool active;
	String<MaxMessageLength> message;
	String<MaxTitleLength> title;
	int mode;
	uint32_t seq;
	uint32_t timer, timeout;
	AxesBitmap controls;

	MessageBox() noexcept : active(false), seq(0) { }

protected:
	DECLARE_OBJECT_MODEL
};

#endif /* SRC_PLATFORM_MESSAGEBOX_H_ */
