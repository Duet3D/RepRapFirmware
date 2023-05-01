/*
 * LedStripBase.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LEDSTRIPBASE_H_
#define SRC_LEDSTRIPS_LEDSTRIPBASE_H_

#include <ObjectModel/ObjectModel.h>

#if SUPPORT_LED_STRIPS

class LedStripBase INHERIT_OBJECT_MODEL
{
public:
	LedStripBase();

protected:
	DECLARE_OBJECT_MODEL
};

#endif

#endif /* SRC_LEDSTRIPS_LEDSTRIPBASE_H_ */
