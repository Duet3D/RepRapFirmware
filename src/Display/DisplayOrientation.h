/*
 * DisplayOrientation.h
 *
 * Created: 04/11/2014 17:34:21
 *  Author: David
 */


#ifndef SRC_DISPLAY_DISPLAYORIENTATION_H_
#define SRC_DISPLAY_DISPLAYORIENTATION_H_

#include <cstdint>

// Enumeration to define the orientation of the display.
// To keep the code small and fast, we use individual bits to say what needs to be done on the display.
// Then we define the supported orientations in terms of those bits.
enum DisplayOrientation : uint8_t {
	Default = 0x00,
	SwapXY = 0x01,
	ReverseX = 0x02,
	ReverseY = 0x04,
	InvertText = ReverseY,
	InvertBitmap = ReverseX
};

#endif /* SRC_DISPLAY_DISPLAYORIENTATION_H_ */
