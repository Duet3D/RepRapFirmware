/*
 * GCodeResult.h
 *
 *  Created on: 1 Oct 2017
 *      Author: David
 */

#ifndef SRC_GCODES_GCODERESULT_H_
#define SRC_GCODES_GCODERESULT_H_

#include <cctype>

// Enumeration to specify the result of attempting to process a GCode command
enum class GCodeResult : uint8_t
{
	notFinished,					// we haven't finished processing this command
	ok,								// we have finished processing this code
	error,
	warning,
	warningNotSupported,
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

#endif /* SRC_GCODES_GCODERESULT_H_ */
