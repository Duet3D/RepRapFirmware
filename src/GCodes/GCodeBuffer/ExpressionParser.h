/*
 * ExpressionParser.h
 *
 *  Created on: 6 Mar 2020
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_
#define SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_

#include <RepRapFirmware.h>
#include <General/StringBuffer.h>
#include <ObjectModel/ObjectModel.h>
#include <GCodes/GCodeException.h>

class ExpressionParser
{
public:
	ExpressionParser(const GCodeBuffer& p_gb, const char *text, const char *textLimit, int p_column = -1) noexcept;

	ExpressionValue Parse(bool evaluate = true, uint8_t priority = 0) THROWS(GCodeException);
	bool ParseBoolean() THROWS(GCodeException);
	float ParseFloat() THROWS(GCodeException);
	int32_t ParseInteger() THROWS(GCodeException);
	uint32_t ParseUnsigned() THROWS(GCodeException);

	void SkipWhiteSpace() noexcept;
	void CheckForExtraCharacters() THROWS(GCodeException);
	const char *GetEndptr() const noexcept { return currentp; }

private:
	GCodeException ConstructParseException(const char *str) const noexcept;
	GCodeException ConstructParseException(const char *str, const char *param) const noexcept;
	GCodeException ConstructParseException(const char *str, uint32_t param) const noexcept;

	ExpressionValue ParseExpectKet(bool evaluate, char expectedKet) THROWS(GCodeException);
	ExpressionValue ParseNumber() noexcept
		pre(readPointer >= 0; isdigit(gb.buffer[readPointer]));
	ExpressionValue ParseIdentifierExpression(bool evaluate, bool applyLengthOperator) THROWS(GCodeException)
		pre(readPointer >= 0; isalpha(gb.buffer[readPointer]));
	void ParseQuotedString(const StringRef& str) THROWS(GCodeException);

	void ConvertToFloat(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToBool(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToString(ExpressionValue& val, bool evaluate) THROWS(GCodeException);

	void BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException);
	void BalanceTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException);
	void EnsureNumeric(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);

	const char *GetAndFix() THROWS(GCodeException);
	int GetColumn() const noexcept;
	char CurrentCharacter() const noexcept;
	void AdvancePointer() noexcept { ++currentp; }		// could check that we havebn't reached endp but we should stop before that happens

	const char *currentp;
	const char * const startp;
	const char * const endp;
	const GCodeBuffer& gb;
	int column;
	char stringBufferStorage[StringBufferLength];
	StringBuffer stringBuffer;
};

#endif /* SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_ */
