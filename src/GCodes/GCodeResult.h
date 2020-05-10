/*
 * GCodeResult.h
 *
 *  Created on: 1 Oct 2017
 *      Author: David
 */

#ifndef SRC_GCODES_GCODERESULT_H_
#define SRC_GCODES_GCODERESULT_H_

#include <cctype>
#include <MessageType.h>

// Enumeration to specify the result of attempting to process a GCode command
// These are ordered such that errors > warnings > ok
enum class GCodeResult : uint8_t
{
	notFinished,					// we haven't finished processing this command
	ok,								// we have finished processing this code
	warning,
	warningNotSupported,
	error,
	errorNotSupported,
	notSupportedInCurrentMode,
	badOrMissingParameter,
	remoteInternalError
};

// Convert a true/false error/no-error indication to a GCodeResult
inline GCodeResult GetGCodeResultFromError(bool err) noexcept
{
	return (err) ? GCodeResult::error : GCodeResult::ok;
}

// Convert a true/false finished/not-finished indication to a GCodeResult
inline GCodeResult GetGCodeResultFromFinished(bool finished) noexcept
{
	return (finished) ? GCodeResult::ok : GCodeResult::notFinished;
}

// Convert an error or warning result into a suitable generic message type. Should only be called with GCodeResult::warning or GCodeResult::error.
inline MessageType GetGenericMessageType(GCodeResult rslt)
{
	return (rslt == GCodeResult::warning) ? WarningMessage : ErrorMessage;
}

#endif /* SRC_GCODES_GCODERESULT_H_ */
