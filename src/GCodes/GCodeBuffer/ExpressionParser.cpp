/*
 * ExpressionParser.cpp
 *
 *  Created on: 6 Mar 2020
 *      Author: David
 */

#include "ExpressionParser.h"

#include "GCodeBuffer.h"
#include <RepRap.h>
#include <General/NamedEnum.h>
#include <General/NumericConverter.h>

#include <limits>

// These can't be declared locally inside ParseIdentifierExpression because NamedEnum includes static data
NamedEnum(NamedConstant, unsigned int, _false, iterations, line, _null, pi, _result, _true);
NamedEnum(Function, unsigned int, abs, acos, asin, atan, atan2, cos, degrees, floor, isnan, max, min, mod, radians, sin, sqrt, tan);

ExpressionParser::ExpressionParser(const GCodeBuffer& p_gb, const char *text, const char *textLimit, int p_column) noexcept
	: currentp(text), startp(text), endp(textLimit), gb(p_gb), column(p_column), stringBuffer(stringBufferStorage, ARRAY_SIZE(stringBufferStorage))
{
}

// Evaluate a bracketed expression
ExpressionValue ExpressionParser::ParseExpectKet(bool evaluate, char closingBracket) THROWS(GCodeException)
{
	auto rslt = Parse(evaluate);
	if (CurrentCharacter() != closingBracket)
	{
		throw ConstructParseException("expected '%c'", (uint32_t)closingBracket);
	}
	AdvancePointer();
	return rslt;
}

// Evaluate an expression, stopping before any binary operators with priority 'priority' or lower
ExpressionValue ExpressionParser::Parse(bool evaluate, uint8_t priority) THROWS(GCodeException)
{
	// Lists of binary operators and their priorities
	static constexpr const char *operators = "?^&|!=<>+-*/";				// for multi-character operators <= and >= and != this is the first character
	static constexpr uint8_t priorities[] = { 1, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 6 };
	constexpr uint8_t UnaryPriority = 10;									// must be higher than any binary operator priority
	static_assert(ARRAY_SIZE(priorities) == strlen(operators));

	// Start by looking for a unary operator or opening bracket
	SkipWhiteSpace();
	const char c = CurrentCharacter();
	ExpressionValue val;
	switch (c)
	{
	case '"':
		ParseQuotedString(stringBuffer.GetRef());
		val.Set(GetAndFix());
		break;

	case '-':
		AdvancePointer();
		val = Parse(evaluate, UnaryPriority);
		switch (val.GetType())
		{
		case TypeCode::Int32:
			val.iVal = -val.iVal;		//TODO overflow check
			break;

		case TypeCode::Float:
			val.fVal = -val.fVal;
			break;

		default:
			throw ConstructParseException("expected numeric value after '-'");
		}
		break;

	case '+':
		AdvancePointer();
		val = Parse(evaluate, UnaryPriority);
		switch (val.GetType())
		{
		case TypeCode::Uint32:
			// Convert enumeration to integer
			val.iVal = (int32_t)val.uVal;
			val.SetType(TypeCode::Int32);
			break;

		case TypeCode::Int32:
		case TypeCode::Float:
			break;

		default:
			throw ConstructParseException("expected numeric or enumeration value after '+'");
		}
		break;

	case '#':
		AdvancePointer();
		SkipWhiteSpace();
		if (isalpha(CurrentCharacter()))
		{
			// Probably applying # to an object model array, so optimise by asking the OM for just the length
			val = ParseIdentifierExpression(evaluate, true);
		}
		else
		{
			val = Parse(evaluate, UnaryPriority);
			if (val.GetType() == TypeCode::CString)
			{
				const char* s = val.sVal;
				val.Set((int32_t)strlen(s));
				stringBuffer.FinishedUsing(s);
				val.SetType(TypeCode::Int32);
			}
			else
			{
				throw ConstructParseException("expected object model value or string after '#");
			}
		}
		break;

	case '{':
		AdvancePointer();
		val = ParseExpectKet(evaluate, '}');
		break;

	case '(':
		AdvancePointer();
		val = ParseExpectKet(evaluate, ')');
		break;

	case '!':
		AdvancePointer();
		val = Parse(evaluate, UnaryPriority);
		ConvertToBool(val, evaluate);
		val.bVal = !val.bVal;
		break;

	default:
		if (isdigit(c))						// looks like a number
		{
			val = ParseNumber();
		}
		else if (isalpha(c))				// looks like a variable name
		{
			val = ParseIdentifierExpression(evaluate, false);
		}
		else
		{
			throw ConstructParseException("expected an expression");
		}
		break;
	}

	// See if it is followed by a binary operator
	do
	{
		SkipWhiteSpace();
		char opChar = CurrentCharacter();
		if (opChar == 0)	// don't pass null to strchr
		{
			return val;
		}

		const char * const q = strchr(operators, opChar);
		if (q == nullptr)
		{
			return val;
		}
		const size_t index = q - operators;
		const uint8_t opPrio = priorities[index];
		if (opPrio <= priority)
		{
			return val;
		}

		AdvancePointer();								// skip the [first] operator character

		// Handle >= and <= and !=
		bool invert = false;
		if (opChar == '!')
		{
			if (CurrentCharacter() != '=')
			{
				throw ConstructParseException("expected '='");
			}
			invert = true;
			AdvancePointer();
			opChar = '=';
		}
		else if ((opChar == '>' || opChar == '<') && CurrentCharacter() == '=')
		{
			invert = true;
			AdvancePointer();
			opChar ^= ('>' ^ '<');			// change < to > or vice versa
		}

		// Allow == && || as alternatives to = & |
		if ((opChar == '=' || opChar == '&' || opChar == '|') && CurrentCharacter() == opChar)
		{
			AdvancePointer();
		}

		// Handle operators that do not always evaluate their second operand
		switch (opChar)
		{
		case '&':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = Parse(evaluate && val.bVal, opPrio);		// get the next operand
				if (val.bVal)
				{
					ConvertToBool(val2, evaluate);
					val.bVal = val.bVal && val2.bVal;
				}
			}
			break;

		case '|':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = Parse(evaluate && !val.bVal, opPrio);		// get the next operand
				if (!val.bVal)
				{
					ConvertToBool(val2, evaluate);
					val.bVal = val.bVal || val2.bVal;
				}
			}
			break;

		case '?':
			ConvertToBool(val, evaluate);
			{
				ExpressionValue val2 = Parse(evaluate && val.bVal, opPrio);		// get the second operand
				if (CurrentCharacter() != ':')
				{
					throw ConstructParseException("expected ':'");
				}
				AdvancePointer();
				ExpressionValue val3 = Parse(evaluate && !val.bVal, opPrio - 1);	// get the third operand, which may be a further conditional expression
				return (val.bVal) ? val2 : val3;
			}

		default:
			// Handle binary operators that always evaluate both operands
			{
				ExpressionValue val2 = Parse(evaluate, opPrio);	// get the next operand
				switch(opChar)
				{
				case '+':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.GetType() == TypeCode::Float)
					{
						val.fVal += val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal += val2.iVal;
					}
					break;

				case '-':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.GetType() == TypeCode::Float)
					{
						val.fVal -= val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal -= val2.iVal;
					}
					break;

				case '*':
					BalanceNumericTypes(val, val2, evaluate);
					if (val.GetType() == TypeCode::Float)
					{
						val.fVal *= val2.fVal;
						val.param = max(val.param, val2.param);
					}
					else
					{
						val.iVal *= val2.iVal;
					}
					break;

				case '/':
					ConvertToFloat(val, evaluate);
					ConvertToFloat(val2, evaluate);
					val.fVal /= val2.fVal;
					val.param = 0;
					break;

				case '>':
					BalanceTypes(val, val2, evaluate);
					switch (val.GetType())
					{
					case TypeCode::Int32:
						val.bVal = (val.iVal > val2.iVal);
						break;

					case TypeCode::Float:
						val.bVal = (val.fVal > val2.fVal);
						break;

					case TypeCode::Bool:
						val.bVal = (val.bVal && !val2.bVal);
						break;

					default:
						throw ConstructParseException("expected numeric or Boolean operands to comparison operator");
					}
					val.SetType(TypeCode::Bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '<':
					BalanceTypes(val, val2, evaluate);
					switch (val.GetType())
					{
					case TypeCode::Int32:
						val.bVal = (val.iVal < val2.iVal);
						break;

					case TypeCode::Float:
						val.bVal = (val.fVal < val2.fVal);
						break;

					case TypeCode::Bool:
						val.bVal = (!val.bVal && val2.bVal);
						break;

					default:
						throw ConstructParseException("expected numeric or Boolean operands to comparison operator");
					}
					val.SetType(TypeCode::Bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '=':
					// Before balancing, handle comparisons with null
					if (val.GetType() == TypeCode::None)
					{
						val.bVal = (val2.GetType() == TypeCode::None);
					}
					else if (val2.GetType() == TypeCode::None)
					{
						val.bVal = false;
					}
					else
					{
						BalanceTypes(val, val2, evaluate);
						switch (val.GetType())
						{
						case TypeCode::ObjectModel:
							throw ConstructParseException("cannot compare objects");

						case TypeCode::Int32:
							val.bVal = (val.iVal == val2.iVal);
							break;

						case TypeCode::Uint32:
							val.bVal = (val.uVal == val2.uVal);
							break;

						case TypeCode::Float:
							val.bVal = (val.fVal == val2.fVal);
							break;

						case TypeCode::Bool:
							val.bVal = (val.bVal == val2.bVal);
							break;

						case TypeCode::CString:
							val.bVal = (strcmp(val.sVal, val2.sVal) == 0);
							break;

						default:
							throw ConstructParseException("unexpected operand type to equality operator");
						}
					}
					val.SetType(TypeCode::Bool);
					if (invert)
					{
						val.bVal = !val.bVal;
					}
					break;

				case '^':
					ConvertToString(val, evaluate);
					ConvertToString(val2, evaluate);
					// We could skip evaluation if evaluate is false, but there is no real need to
					if (stringBuffer.Concat(val.sVal, val2.sVal))
					{
						throw ConstructParseException("too many strings");
					}
					val.sVal = GetAndFix();
					break;
				}
			}
		}
	} while (true);
}

bool ExpressionParser::ParseBoolean() THROWS(GCodeException)
{
	ExpressionValue val = Parse();
	ConvertToBool(val, true);
	return val.bVal;
}

float ExpressionParser::ParseFloat() THROWS(GCodeException)
{
	ExpressionValue val = Parse();
	ConvertToFloat(val, true);
	return val.fVal;
}

int32_t ExpressionParser::ParseInteger() THROWS(GCodeException)
{
	ExpressionValue val = Parse();
	switch (val.GetType())
	{
	case TypeCode::Int32:
		return val.iVal;

	case TypeCode::Uint32:
		if (val.uVal > (uint32_t)std::numeric_limits<int32_t>::max())
		{
			throw ConstructParseException("unsigned integer too large");
		}
		return (int32_t)val.uVal;

	default:
		throw ConstructParseException("expected integer value");
	}
}

uint32_t ExpressionParser::ParseUnsigned() THROWS(GCodeException)
{
	ExpressionValue val = Parse();
	switch (val.GetType())
	{
	case TypeCode::Uint32:
		return val.uVal;

	case TypeCode::Int32:
		if (val.iVal >= 0)
		{
			return (uint32_t)val.iVal;
		}
		throw ConstructParseException("value must be non-negative");

	default:
		throw ConstructParseException("expected non-negative integer value");
	}
}

void ExpressionParser::BalanceNumericTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException)
{
	if (val1.GetType() == TypeCode::Float)
	{
		ConvertToFloat(val2, evaluate);
	}
	else if (val2.GetType() == TypeCode::Float)
	{
		ConvertToFloat(val1, evaluate);
	}
	else if (val1.GetType() != TypeCode::Int32 || val2.GetType() != TypeCode::Int32)
	{
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operands");
		}
		val1.Set((int32_t)0);
		val2.Set((int32_t)0);
	}
}

void ExpressionParser::BalanceTypes(ExpressionValue& val1, ExpressionValue& val2, bool evaluate) const THROWS(GCodeException)
{
	if (val1.GetType() == TypeCode::Float)
	{
		ConvertToFloat(val2, evaluate);
	}
	else if (val2.GetType() == TypeCode::Float)
	{
		ConvertToFloat(val1, evaluate);
	}
	else if (val1.GetType() != val2.GetType())
	{
		if (evaluate)
		{
			throw ConstructParseException("cannot convert operands to same type");
		}
		val1.Set((int32_t)0);
		val2.Set((int32_t)0);
	}
}

void ExpressionParser::EnsureNumeric(ExpressionValue& val, bool evaluate) const THROWS(GCodeException)
{
	switch (val.GetType())
	{
	case TypeCode::Uint32:
		val.SetType(TypeCode::Int32);
		val.iVal = val.uVal;
		break;

	case TypeCode::Int32:
	case TypeCode::Float:
		break;

	default:
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operand");
		}
		val.Set((int32_t)0);
	}
}

void ExpressionParser::ConvertToFloat(ExpressionValue& val, bool evaluate) const THROWS(GCodeException)
{
	switch (val.GetType())
	{
	case TypeCode::Int32:
		val.fVal = (float)val.iVal;
		val.SetType(TypeCode::Float);
		val.param = 1;
		break;

	case TypeCode::Float:
		break;

	default:
		if (evaluate)
		{
			throw ConstructParseException("expected numeric operand");
		}
		val.Set(0.0f);
	}
}

void ExpressionParser::ConvertToBool(ExpressionValue& val, bool evaluate) const THROWS(GCodeException)
{
	if (val.GetType() != TypeCode::Bool)
	{
		if (evaluate)
		{
			throw ConstructParseException("expected Boolean operand");
		}
		val.Set(false);
	}
}

void ExpressionParser::ConvertToString(ExpressionValue& val, bool evaluate) THROWS(GCodeException)
{
	if (val.GetType() != TypeCode::CString)
	{
		if (evaluate)
		{
			stringBuffer.ClearLatest();
			val.AppendAsString(stringBuffer.GetRef());
			val.Set(GetAndFix());
		}
		else
		{
			val.Set("");
		}
	}
}

// Get a C-style pointer to the latest string in the buffer, and start a new one
const char *ExpressionParser::GetAndFix()
{
	const char *const rslt = stringBuffer.LatestCStr();
	if (stringBuffer.Fix())
	{
		throw ConstructParseException("too many strings");
	}
	return rslt;
}

void ExpressionParser::SkipWhiteSpace() noexcept
{
	char c;
	while ((c = CurrentCharacter()) == ' ' || c == '\t')
	{
		AdvancePointer();
	}
}

void ExpressionParser::CheckForExtraCharacters() THROWS(GCodeException)
{
	SkipWhiteSpace();
	if (CurrentCharacter() != 0)
	{
		throw ConstructParseException("Unexpected characters after expression");
	}
}

// Parse a number. The initial character of the string is a decimal digit.
ExpressionValue ExpressionParser::ParseNumber() noexcept
{
	NumericConverter conv;
	conv.Accumulate(CurrentCharacter(), true, true, [this]()->char { AdvancePointer(); return CurrentCharacter(); });		// must succeed because CurrentCharacter is a decimal digit
	return (conv.FitsInInt32())
			? ExpressionValue(conv.GetInt32())
				: ExpressionValue(conv.GetFloat(), constrain<unsigned int>(conv.GetDigitsAfterPoint(), 1, MaxFloatDigitsDisplayedAfterPoint));
}

// Parse an identifier expression
// If 'evaluate' is false then the object model path may not exist, in which case we must ignore error that and parse it all anyway
// This means we can use expressions such as: if {a.b == null || a.b.c == 1}
ExpressionValue ExpressionParser::ParseIdentifierExpression(bool evaluate, bool applyLengthOperator) THROWS(GCodeException)
{
	if (!isalpha(CurrentCharacter()))
	{
		throw ConstructParseException("expected an identifier");
	}

	String<MaxVariableNameLength> id;
	ObjectExplorationContext context("v", applyLengthOperator, 99, gb.MachineState().lineNumber, GetColumn());

	// Loop parsing identifiers and index expressions
	// When we come across an index expression, evaluate it, add it to the context, and place a marker in the identifier string.
	char c;
	while (isalpha((c = CurrentCharacter())) || isdigit(c) || c == '_' || c == '.' || c == '[')
	{
		AdvancePointer();
		if (c == '[')
		{
			const ExpressionValue index = Parse(evaluate);
			if (CurrentCharacter() != ']')
			{
				throw ConstructParseException("expected ']'");
			}
			if (index.GetType() != TypeCode::Int32)
			{
				throw ConstructParseException("expected integer expression");
			}
			AdvancePointer();										// skip the ']'
			context.ProvideIndex(index.iVal);
			c = '^';									// add the marker
		}
		if (id.cat(c))
		{
			throw ConstructParseException("variable name too long");;
		}
	}

	// Check for the names of constants
	NamedConstant whichConstant(id.c_str());
	if (whichConstant.IsValid())
	{
		switch (whichConstant.RawValue())
		{
		case NamedConstant::_true:
			return ExpressionValue(true);

		case NamedConstant::_false:
			return ExpressionValue(false);

		case NamedConstant::_null:
			return ExpressionValue(nullptr);

		case NamedConstant::pi:
			return ExpressionValue(Pi);

		case NamedConstant::iterations:
			{
				const int32_t v = gb.MachineState().GetIterations();
				if (v < 0)
				{
					throw ConstructParseException("'iterations' used when not inside a loop");
				}
				return ExpressionValue(v);
			}

		case NamedConstant::_result:
			{
				int32_t rslt;
				switch (gb.GetLastResult())
				{
				case GCodeResult::ok:
					rslt = 0;
					break;

				case GCodeResult::warning:
				case GCodeResult::warningNotSupported:
					rslt = 1;
					break;

				default:
					rslt = 2;
					break;
				}
				return ExpressionValue(rslt);
			}

		case NamedConstant::line:
			return ExpressionValue((int32_t)gb.MachineState().lineNumber);

		default:
			THROW_INTERNAL_ERROR;
		}
	}

	// Check whether it is a function call
	SkipWhiteSpace();
	if (CurrentCharacter() == '(')
	{
		// It's a function call
		AdvancePointer();
		ExpressionValue rslt = Parse(evaluate);					// evaluate the first operand
		const Function func(id.c_str());
		if (!func.IsValid())
		{
			throw ConstructParseException("unknown function");
		}

		switch (func.RawValue())
		{
		case Function::abs:
			switch (rslt.GetType())
			{
			case TypeCode::Int32:
				rslt.iVal = labs(rslt.iVal);
				break;

			case TypeCode::Float:
				rslt.fVal = fabsf(rslt.fVal);
				break;

			default:
				if (evaluate)
				{
					throw ConstructParseException("expected numeric operand");
				}
				rslt.Set((int32_t)0);
			}
			break;

		case Function::sin:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = sinf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::cos:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = cosf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::tan:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = tanf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::asin:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = asinf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::acos:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = acosf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::atan:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = atanf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::atan2:
			{
				ConvertToFloat(rslt, evaluate);
				SkipWhiteSpace();
				if (CurrentCharacter() != ',')
				{
					throw ConstructParseException("expected ','");
				}
				AdvancePointer();
				SkipWhiteSpace();
				ExpressionValue nextOperand = Parse(evaluate);
				ConvertToFloat(nextOperand, evaluate);
				rslt.fVal = atan2f(rslt.fVal, nextOperand.fVal);
				rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			}
			break;

		case Function::degrees:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = rslt.fVal * RadiansToDegrees;
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::radians:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = rslt.fVal * DegreesToRadians;
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::sqrt:
			ConvertToFloat(rslt, evaluate);
			rslt.fVal = sqrtf(rslt.fVal);
			rslt.param = MaxFloatDigitsDisplayedAfterPoint;
			break;

		case Function::isnan:
			ConvertToFloat(rslt, evaluate);
			rslt.SetType(TypeCode::Bool);
			rslt.bVal = (isnan(rslt.fVal) != 0);
			break;

		case Function::floor:
			{
				ConvertToFloat(rslt, evaluate);
				const float f = floorf(rslt.fVal);
				if (f <= (float)std::numeric_limits<int32_t>::max() && f >= (float)std::numeric_limits<int32_t>::min())
				{
					rslt.SetType(TypeCode::Int32);
					rslt.iVal = (int32_t)f;
				}
				else
				{
					rslt.fVal = f;
				}
			}
			break;

		case Function::mod:
			{
				SkipWhiteSpace();
				if (CurrentCharacter() != ',')
				{
					throw ConstructParseException("expected ','");
				}
				AdvancePointer();
				SkipWhiteSpace();
				ExpressionValue nextOperand = Parse(evaluate);
				BalanceNumericTypes(rslt, nextOperand, evaluate);
				if (rslt.GetType() == TypeCode::Float)
				{
					rslt.fVal = fmod(rslt.fVal, nextOperand.fVal);
				}
				else if (nextOperand.iVal == 0)
				{
					rslt.iVal = 0;
				}
				else
				{
					rslt.iVal %= nextOperand.iVal;
				}
			}
			break;

		case Function::max:
			for (;;)
			{
				SkipWhiteSpace();
				if (CurrentCharacter() != ',')
				{
					break;
				}
				AdvancePointer();
				SkipWhiteSpace();
				ExpressionValue nextOperand = Parse(evaluate);
				BalanceNumericTypes(rslt, nextOperand, evaluate);
				if (rslt.GetType() == TypeCode::Float)
				{
					rslt.fVal = max<float>(rslt.fVal, nextOperand.fVal);
					rslt.param = max(rslt.param, nextOperand.param);
				}
				else
				{
					rslt.iVal = max<int32_t>(rslt.iVal, nextOperand.iVal);
				}
			}
			break;

		case Function::min:
			for (;;)
			{
				SkipWhiteSpace();
				if (CurrentCharacter() != ',')
				{
					break;
				}
				AdvancePointer();
				SkipWhiteSpace();
				ExpressionValue nextOperand = Parse(evaluate);
				BalanceNumericTypes(rslt, nextOperand, evaluate);
				if (rslt.GetType() == TypeCode::Float)
				{
					rslt.fVal = min<float>(rslt.fVal, nextOperand.fVal);
					rslt.param = max(rslt.param, nextOperand.param);
				}
				else
				{
					rslt.iVal = min<int32_t>(rslt.iVal, nextOperand.iVal);
				}
			}
			break;

		default:
			THROW_INTERNAL_ERROR;
		}

		SkipWhiteSpace();
		if (CurrentCharacter() != ')')
		{
			throw ConstructParseException("expected ')'");
		}
		AdvancePointer();
		return rslt;
	}

	// If we are not evaluating then the object expression doesn't have to exist, so don't retrieve it because that might throw an error
	return (evaluate) ? reprap.GetObjectValue(context, nullptr, id.c_str()) : ExpressionValue(nullptr);
}

// Parse a quoted string, given that the current character is double-quote
// This is almost a copy of InternalGetQuotedString in class StringParser
void ExpressionParser::ParseQuotedString(const StringRef& str) THROWS(GCodeException)
{
	str.Clear();
	AdvancePointer();
	for (;;)
	{
		char c = CurrentCharacter();
		AdvancePointer();
		if (c < ' ')
		{
			throw ConstructParseException("control character in string");
		}
		if (c == '"')
		{
			if (CurrentCharacter() != c)
			{
				return;
			}
			AdvancePointer();
		}
		else if (c == '\'')
		{
			if (isalpha(CurrentCharacter()))
			{
				// Single quote before an alphabetic character forces that character to lower case
				c = tolower(CurrentCharacter());
				AdvancePointer();
			}
			else if (CurrentCharacter() == c)
			{
				// Two backslashes are used to represent one
				AdvancePointer();
			}
		}
		if (str.cat(c))
		{
			throw ConstructParseException("string too long");
		}
	}
}

// Return the current character, or 0 if we have run out of string
char ExpressionParser::CurrentCharacter() const noexcept
{
	return (currentp < endp) ? *currentp : 0;
}

int ExpressionParser::GetColumn() const noexcept
{
	return (column < 0) ? column : (currentp - startp) + column;
}

GCodeException ExpressionParser::ConstructParseException(const char *str) const noexcept
{
	return GCodeException(gb.MachineState().lineNumber, GetColumn(), str);
}

GCodeException ExpressionParser::ConstructParseException(const char *str, const char *param) const noexcept
{
	return GCodeException(gb.MachineState().lineNumber, GetColumn(), str, param);
}

GCodeException ExpressionParser::ConstructParseException(const char *str, uint32_t param) const noexcept
{
	return GCodeException(gb.MachineState().lineNumber, GetColumn(), str, param);
}

// End
