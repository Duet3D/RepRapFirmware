/*
 * ExpressionParser.h
 *
 *  Created on: 6 Mar 2020
 *      Author: David
 */

#ifndef SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_
#define SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_

#include <RepRapFirmware.h>
#include <ObjectModel/ObjectModel.h>
#include <GCodes/GCodeException.h>

class VariableSet;

class ExpressionParser
{
public:
	ExpressionParser(const GCodeBuffer& p_gb, const char *text, const char *textLimit, int p_column = -1) noexcept;

	ExpressionValue Parse(bool evaluate = true) THROWS(GCodeException);
	bool ParseBoolean() THROWS(GCodeException);
	float ParseFloat() THROWS(GCodeException);
	int32_t ParseInteger() THROWS(GCodeException);
	uint32_t ParseUnsigned() THROWS(GCodeException);
	DriverId ParseDriverId() THROWS(GCodeException);

	// These 4 functions may be removed when we support array-valued expressions more generally
	void ParseFloatArray(float arr[], size_t& length) THROWS(GCodeException);
	void ParseIntArray(int32_t arr[], size_t& length) THROWS(GCodeException);
	void ParseUnsignedArray(uint32_t arr[], size_t& length) THROWS(GCodeException);
	void ParseDriverIdArray(DriverId arr[], size_t& length) THROWS(GCodeException);

	void SkipWhiteSpace() noexcept;
	void CheckForExtraCharacters() THROWS(GCodeException);
	const char *GetEndptr() const noexcept { return currentp; }

private:
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str) const THROWS(GCodeException);
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str, const char *param) const THROWS(GCodeException);
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str, uint32_t param) const THROWS(GCodeException);

	void ParseInternal(ExpressionValue& val, bool evaluate, uint8_t priority) THROWS(GCodeException);
	void ParseExpectKet(ExpressionValue& rslt, bool evaluate, char expectedKet) THROWS(GCodeException);
	void __attribute__((noinline)) ParseNumber(ExpressionValue& rslt) noexcept
		pre(readPointer >= 0; isdigit(gb.buffer[readPointer]));
	void __attribute__((noinline)) ParseIdentifierExpression(ExpressionValue& rslt, bool evaluate, bool applyLengthOperator, bool applyExists) THROWS(GCodeException)
		pre(readPointer >= 0; isalpha(gb.buffer[readPointer]));
	void __attribute__((noinline)) ParseQuotedString(ExpressionValue& rslt) THROWS(GCodeException);

	void ParseArray(size_t& length, function_ref<void(size_t index) THROWS(GCodeException)> processElement) THROWS(GCodeException);
	time_t ParseDateTime(const char *s) const THROWS(GCodeException);

	void GetVariableValue(ExpressionValue& rslt, const VariableSet *vars, const char *name, bool parameter, bool wantExists) THROWS(GCodeException);

	void ConvertToFloat(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToBool(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToString(ExpressionValue& val, bool evaluate) noexcept;
	void ConvertToDriverId(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);

	void CheckStack(uint32_t calledFunctionStackUsage) const THROWS(GCodeException);

	// The following must be declared 'noinline' because it allocates a large buffer on the stack and its caller is recursive
	static void __attribute__((noinline)) StringConcat(ExpressionValue &val, ExpressionValue &val2) noexcept;

	void BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException);
	void BalanceTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) THROWS(GCodeException);
	static bool TypeHasNoLiterals(TypeCode t) noexcept;

	int GetColumn() const noexcept;
	char CurrentCharacter() const noexcept;
	void AdvancePointer() noexcept { ++currentp; }		// could check that we havebn't reached endp but we should stop before that happens

	const char *currentp;
	const char * const startp;
	const char * const endp;
	const GCodeBuffer& gb;
	int column;
	String<MaxVariableNameLength> obsoleteField;
};

#endif /* SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_ */
