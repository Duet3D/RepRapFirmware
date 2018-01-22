/*
 * Display.h
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#ifndef SRC_DISPLAY_DISPLAY_H_
#define SRC_DISPLAY_DISPLAY_H_

#include "RotaryEncoder.h"
#include "ST7920/lcd7920.h"
#include "Menu.h"

class Display
{
public:
	Display();

	void Init();
	void Spin(bool full);
	void Exit();

private:
	Lcd7920 lcd;
	RotaryEncoder encoder;
	Menu *mainMenu;
};

#endif /* SRC_DISPLAY_DISPLAY_H_ */
