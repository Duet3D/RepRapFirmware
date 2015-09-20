#include "Arduino.h"
#include "Line.h"

//***************************************************************************************************

// Serial/USB class

Line::Line(Stream& p_iface) : iface(p_iface)
{
}

uint8_t Line::Status() const
{
	return inputNumChars == 0 ? (uint8_t)IOStatus::nothing : (uint8_t)IOStatus::byteAvailable;
}

// This is only ever called on initialisation, so we know the buffer won't overflow
void Line::InjectString(char* string)
{
	int i = 0;
	while(string[i])
	{
		inBuffer[(inputGetIndex + inputNumChars) % lineInBufsize] = string[i];
		inputNumChars++;
		i++;
	}
}

int Line::Read(char& b)
{
	if (inputNumChars == 0)
		return 0;
	b = inBuffer[inputGetIndex];
	inputGetIndex = (inputGetIndex + 1) % lineInBufsize;
	--inputNumChars;
	return 1;
}

void Line::Init()
{
	inputGetIndex = 0;
	inputNumChars = 0;
	outputGetIndex = 0;
	outputNumChars = 0;
	ignoringOutputLine = false;
	inWrite = 0;
	outputColumn = 0;
	timeLastCharWritten = 0;
}

void Line::Spin()
{
	// Read the serial data in blocks to avoid excessive flow control
	if (inputNumChars <= lineInBufsize / 2)
	{
		int16_t target = iface.available() + (int16_t) inputNumChars;
		if (target > lineInBufsize)
		{
			target = lineInBufsize;
		}
		while ((int16_t) inputNumChars < target)
		{
			int incomingByte = iface.read();
			if (incomingByte < 0)
				break;
			inBuffer[(inputGetIndex + inputNumChars) % lineInBufsize] = (char) incomingByte;
			++inputNumChars;
		}
	}

	TryFlushOutput();
}

// Write a character to USB.
// If 'important' is true then we don't return until we have either written it to the USB port,
// or put it in the buffer, or we have timed out waiting for the buffer to empty. The purpose of the timeout is to
// avoid getting a software watchdog reset if we are writing important data (e.g. debug) and there is no consumer for the data.
// Otherwise, if the buffer is full then we append ".\n" to the end of it, return immediately and ignore the rest
// of the data we are asked to print until we get a new line.
void Line::Write(char b, bool important)
{
	if (b == '\n')
	{
		outputColumn = 0;
	}
	else
	{
		++outputColumn;
	}

	if (ignoringOutputLine)
	{
		// We have already failed to write some characters of this message line, so don't write any of it.
		// But try to start sending again after this line finishes.
		if (b == '\n')
		{
			ignoringOutputLine = false;
		}
	}
	else
	{
		for(;;)
		{
			TryFlushOutput();
			if (outputNumChars == 0 && iface.canWrite() != 0)
			{
				// We can write the character directly into the USB output buffer
				++inWrite;
				iface.write(b);
				--inWrite;
				timeLastCharWritten = millis();
				break;
			}
			else if (   outputNumChars + 2 < lineOutBufSize					// save 2 spaces in the output buffer
					 || (b == '\n' && outputNumChars < lineOutBufSize)		//...unless writing newline
					)
			{
				outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = b;
				++outputNumChars;
				break;
			}
			else if (!important || millis() - timeLastCharWritten >= 100)
			{
				// Output is being consumed too slowly, so throw away some data
				if (outputNumChars + 2 == lineOutBufSize)
				{
					// We still have our 2 free characters, so append ".\n" to the line to indicate it was incomplete
					outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = '.';
					++outputNumChars;
					outBuffer[(outputGetIndex + outputNumChars) % lineOutBufSize] = '\n';
					++outputNumChars;
				}
				else
				{
					// As we don't have 2 spare characters in the buffer, we can't have written any of the current line.
					// So ignore the whole line.
				}
				ignoringOutputLine = true;
				break;
			}
		}
	}
}

void Line::Write(const char* b, bool important)
{
	while (*b)
	{
		Write(*b++, important);
	}
}

void Line::TryFlushOutput()
{
	while (outputNumChars != 0 && iface.canWrite() != 0)
	{
		++inWrite;
		iface.write(outBuffer[outputGetIndex]);
		--inWrite;
		timeLastCharWritten = millis();
		outputGetIndex = (outputGetIndex + 1) % lineOutBufSize;
		--outputNumChars;
	}
}

void Line::Flush()
{
	while (outputNumChars != 0)
	{
		TryFlushOutput();
	}
}

// End
