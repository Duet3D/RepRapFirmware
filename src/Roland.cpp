// This class allows the RepRap firmware to transmit commands to a Roland mill
// See: http://www.rolanddg.com/product/3d/3d/mdx-20_15/mdx-20_15.html
//      http://altlab.org/d/content/m/pangelo/ideas/rml_command_guide_en_v100.pdf

#if SUPPORT_ROLAND

#include "RepRapFirmware.h"

Roland::Roland(Platform& p) : platform(p)
{
}

void Roland::Init()
{
	pinMode(ROLAND_RTS_PIN, OUTPUT);
	pinMode(ROLAND_CTS_PIN, INPUT);
	digitalWrite(ROLAND_RTS_PIN, HIGH);

	sBuffer = new StringRef(buffer, ARRAY_SIZE(buffer));
	sBuffer->Clear();

	bufferPointer = 0;
	Zero(true);
	longWait = platform.Time();
	active = false;
}

void Roland::Spin()
{
	if (!Active())
	{
		platform.ClassReport(longWait);
		return;
	}

	// 'U' is 01010101 in binary (nice for an oscilloscope...)

	//SERIAL_AUX2_DEVICE.write('U');
	//SERIAL_AUX2_DEVICE.flush();
	//return;

	// Are we sending something to the Roland?

	if (Busy())	// Busy means we are sending something
	{
		if (digitalRead(ROLAND_CTS_PIN))
		{
			platform.ClassReport(longWait);
			return;
		}

		SERIAL_AUX2_DEVICE.write(buffer[bufferPointer]);
		SERIAL_AUX2_DEVICE.flush();
		bufferPointer++;
	}
	else		// Not sending.
	{	
		// Finished talking to the Roland

		sBuffer->Clear();
		bufferPointer = 0;

		// Anything new to do?

		EndstopChecks endStopsToCheck;
		uint8_t moveType;
		FilePosition filePos;
		if (reprap.GetGCodes()->ReadMove(move, endStopsToCheck, moveType, filePos))
		{
			move[AXES] = move[DRIVES]; // Roland doesn't have extruders etc.
			ProcessMove();
		}
	}

	platform.ClassReport(longWait);
}

void Roland::Zero(bool feed)
{
	size_t lim = feed ? AXES + 1 : AXES;
	for(size_t axis = 0; axis < lim; axis++)
	{
		move[axis] = 0.0;
		coordinates[axis] = 0.0;
		oldCoordinates[axis] = 0.0;
		offset[axis] = 0.0;
	}

	if (reprap.Debug(moduleGcodes))
	{
		platform.Message(HOST_MESSAGE, "Roland zero\n");
	}
}

bool Roland::Busy()
{
	return buffer[bufferPointer] != 0;
}

bool Roland::ProcessHome()
{
	if (Busy())
	{
		return false;
	}

	sBuffer->copy("H;\n");
	Zero(false);
	if (reprap.Debug(moduleGcodes))
	{
		platform.MessageF(HOST_MESSAGE, "Roland home: %s", buffer);
	}
	return true;
}

bool Roland::ProcessDwell(long milliseconds)
{
	if (Busy())
	{
		return false;
	}

	sBuffer->printf("W%ld;", milliseconds);
	sBuffer->catf("Z %.4f,%.4f,%.4f;", oldCoordinates[0], oldCoordinates[1], oldCoordinates[2]);
	sBuffer->cat("W0;\n");
	if (reprap.Debug(moduleGcodes))
	{
		platform.MessageF(HOST_MESSAGE, "Roland dwell: %s", buffer);
	}
	return true;
}

bool Roland::ProcessG92(float v, size_t axis)
{
	if (Busy())
	{
		return false;
	}

	move[axis] = v;
	coordinates[axis] = move[axis]*ROLAND_FACTOR + offset[axis];
	offset[axis] = oldCoordinates[axis];
	oldCoordinates[axis] = coordinates[axis];
	if (reprap.Debug(moduleGcodes))
	{
		platform.Message(HOST_MESSAGE, "Roland G92\n");
	}
	return true;
}

bool Roland::ProcessSpindle(float rpm)
{
	if (Busy())
	{
		return false;
	}

	if (rpm < 0.5)	// Stop
	{
		sBuffer->printf("!MC 0;\n");
	}
	else			// Go
	{
		sBuffer->printf("!RC%ld;!MC 1;\n", (long)(rpm + 100.0));
	}

	if (reprap.Debug(moduleGcodes))
	{
		platform.MessageF(HOST_MESSAGE, "Roland spindle: %s", buffer);
	}
	return true;

}

void Roland::GetCurrentRolandPosition(float moveBuffer[])
{
	for(size_t axis = 0; axis < AXES; axis++)
	{
		moveBuffer[axis] = move[axis];
	}

	for(size_t axis = AXES; axis < DRIVES; axis++)
	{
		moveBuffer[axis] = 0.0;
	}

	moveBuffer[DRIVES] = move[AXES];
}

void Roland::ProcessMove()
{
	for(size_t axis = 0; axis < AXES; axis++)
	{
		coordinates[axis] = move[axis] * ROLAND_FACTOR + offset[axis];
	}
	coordinates[AXES] = move[AXES];

	// Start with feedrate; For some reason the Roland won't accept more than 4 d.p.

	sBuffer->printf("V %.4f;", coordinates[AXES]);

	// Now the move

	sBuffer->catf("Z %.4f,%.4f,%.4f;\n", coordinates[0], coordinates[1], coordinates[2]);

	for(size_t axis = 0; axis <= AXES; axis++)
	{
		oldCoordinates[axis] = coordinates[axis];
	}

	if (reprap.Debug(moduleGcodes))
	{
		platform.MessageF(HOST_MESSAGE, "Roland move: %s", buffer);
	}
}


bool Roland::RawWrite(const char* s)
{
	if (Busy())
	{
		return false;
	}

	sBuffer->copy(s);
	sBuffer->cat("\n");

	if (reprap.Debug(moduleGcodes))
	{
		platform.MessageF(HOST_MESSAGE, "Roland rawwrite: %s", buffer);
	}
	return true;
}

bool Roland::Active()
{
	return active;
}

void Roland::Activate()
{
	digitalWrite(ROLAND_RTS_PIN, LOW);
	active = true;

	if (reprap.Debug(moduleGcodes))
	{
		platform.Message(HOST_MESSAGE, "Roland started\n");
	}
}

bool Roland::Deactivate()
{
	if (Busy())
	{
		return false;
	}

	digitalWrite(ROLAND_RTS_PIN, HIGH);
	active = false;
	if (reprap.Debug(moduleGcodes))
	{
		platform.Message(HOST_MESSAGE, "Roland stopped\n");
	}
	return true;
}

#endif
