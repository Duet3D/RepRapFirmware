/*
 * Display.cpp
 *
 *  Created on: 22 Jan 2018
 *      Author: David
 */

#include "Display.h"

Display::Display()
	: lcd(LcdCSPin), encoder(EncoderPinA, EncoderPinB, EncoderPinSw, 4)
{
	//TODO init menus here
}

void Display::Init()
{
	lcd.begin();
}

void Display::Spin(bool full)
{
	encoder.Poll();
	if (full)
	{
		lcd.Update();
	}
}

void Display::Exit()
{
	// Nothing needed here except perhaps display a "shutdown" message
}

// End
