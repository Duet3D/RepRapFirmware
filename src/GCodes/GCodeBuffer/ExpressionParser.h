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

// Small class to read from file, checking for end of line or end of file and counting the characters read
// We could improve the efficiency by buffering a small number of characters
class LineReader
{
public:
	LineReader(FileStore *pf) noexcept : f(pf), charsRead(0), currentCharacter(0), fileFinished(false) { }

	// Read a character into currentCharacter. If we reach end of file or end of line, set the character to 0 and fileFinished to true.
	void ReadChar() noexcept;

	// While the current character is space or tab, read another character
	void SkipTabsAndSpaces() noexcept;

	// Retrieval functions
	char CurrentCharacter() const noexcept { return currentCharacter; }
	bool FileFinished() const noexcept { return fileFinished; }
	size_t CharsRead() const noexcept { return charsRead; }

	// Close the file
	void Close() noexcept { f->Close(); }

private:
	FileStore *f;
	size_t charsRead;
	char currentCharacter;
	bool fileFinished;
};

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

	void CheckForExtraCharacters() THROWS(GCodeException);
	const char *GetEndptr() const noexcept { return currentp; }

private:
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str) const THROWS(GCodeException);
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str, const char *param) const THROWS(GCodeException);
	[[noreturn]] void __attribute__((noinline)) ThrowParseException(const char *str, uint32_t param) const THROWS(GCodeException);

	void __attribute__((noinline)) ParseInternal(ExpressionValue& val, bool evaluate, uint8_t priority) THROWS(GCodeException);
	void __attribute__((noinline)) ParseExpectKet(ExpressionValue& rslt, bool evaluate, char expectedKet) THROWS(GCodeException);
	void __attribute__((noinline)) ParseNumber(ExpressionValue& rslt) noexcept
		pre(readPointer >= 0; isdigit(gb.buffer[readPointer]));
	void __attribute__((noinline)) ParseIdentifierExpression(ExpressionValue& rslt, bool evaluate, bool applyLengthOperator, bool applyExists) THROWS(GCodeException)
		pre(readPointer >= 0; isalpha(gb.buffer[readPointer]));
	void __attribute__((noinline)) ParseQuotedString(ExpressionValue& rslt) THROWS(GCodeException);

	void ParseCharacter(ExpressionValue& rslt) THROWS(GCodeException);
	void ParseGeneralArray(ExpressionValue& firstElementAndResult, bool evaluate) THROWS(GCodeException);
	void ParseArray(size_t& length, function_ref<void(ExpressionValue& ev, size_t index) THROWS(GCodeException)> processElement) THROWS(GCodeException);

	time_t __attribute__((noinline)) ParseDateTime(const char *s) const THROWS(GCodeException);

	void __attribute__((noinline)) GetVariableValue(ExpressionValue& rslt, const VariableSet *vars, const char *name, ObjectExplorationContext& context, bool isParameter, bool applyLengthOperator, bool wantExists) THROWS(GCodeException);

	void ConvertToFloat(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToInteger(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToUnsigned(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ConvertToBool(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);

	// The following must be declared 'noinline' because it allocates a large buffer on the stack and its caller is recursive
	void __attribute__((noinline)) ConvertToString(ExpressionValue& val, bool evaluate) const noexcept;

	void ConvertToDriverId(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);
	void ApplyLengthOperator(ExpressionValue& val, bool evaluate) const THROWS(GCodeException);

	void CheckStack(uint32_t calledFunctionStackUsage) const THROWS(GCodeException);

	// The following must be declared 'noinline' because it allocates a large buffer on the stack and its caller is recursive
	static void __attribute__((noinline)) StringConcat(ExpressionValue &val, ExpressionValue &val2) noexcept;

	void BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException);
	void BalanceTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException);
	void __attribute__((noinline)) EvaluateMinOrMax(ExpressionValue& v1, ExpressionValue& v2, bool evaluate, bool isMax) const THROWS(GCodeException);
	void __attribute__((noinline)) ReadArrayFromFile(ExpressionValue& rslt, unsigned int offset, unsigned int length, char delimiter) const THROWS(GCodeException);
	void ReadArrayElementFromFile(ExpressionValue& rslt, LineReader& reader, char delimiter) const THROWS(GCodeException);
	void GetNextOperand(ExpressionValue& operand, bool evaluate) THROWS(GCodeException);
	void __attribute__((noinline)) ApplyObjectModelArrayIndex(ExpressionValue& rslt, int indexCol, uint32_t indexValue, bool evaluate) THROWS(GCodeException);

	static bool TypeHasNoLiterals(TypeCode t) noexcept;

	int GetColumn() const noexcept;
	char CurrentCharacter() const noexcept;
	void AdvancePointer() noexcept;
	char SkipWhiteSpace() noexcept;

	const char *currentp;
	const char * const startp;
	const char * const endp;
	const GCodeBuffer& gb;
	int column;
	String<MaxVariableNameLength> obsoleteField;
};

#endif /* SRC_GCODES_GCODEBUFFER_EXPRESSIONPARSER_H_ */
