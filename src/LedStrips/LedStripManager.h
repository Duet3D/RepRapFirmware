/*
 * LedStripManager.h
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_
#define SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_

#include <ObjectModel/ObjectModel.h>

class LedStripBase;

class LedStripManager INHERIT_OBJECT_MODEL
{
public:
	LedStripManager();

	GCodeResult CreateStrip(GCodeBuffer& gb, const StringRef& rslt) THROWS(GCodeException);
	GCodeResult ExecM150(GCodeBuffer& gb, const StringRef& rslt) THROWS(GCodeException);

protected:
	DECLARE_OBJECT_MODEL_WITH_ARRAYS

private:
	size_t GetNumLedStrips() const noexcept;

	LedStripBase *strips[MaxLedStrips];
};

#endif /* SRC_LEDSTRIPS_LEDSTRIPMANAGER_H_ */
