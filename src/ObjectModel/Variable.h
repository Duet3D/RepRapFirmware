/*
 * Variable.h
 *
 *  Created on: 6 Mar 2021
 *      Author: David
 */

#ifndef SRC_GCODES_VARIABLE_H_
#define SRC_GCODES_VARIABLE_H_

#include <RepRapFirmware.h>
#include <General/FreelistManager.h>
#include <Platform/Heap.h>
#include <ObjectModel/ObjectModel.h>
#include <General/function_ref.h>
#include <GCodes/GCodeException.h>

// Class to represent a variable having a name and a value
class Variable
{
public:
	Variable(const char *_ecv_array str, size_t strLen, ExpressionValue& pVal, int16_t pScope) THROWS(GCodeException);
	~Variable();

	static bool IsValidVariableName(const char *_ecv_array str) noexcept;

	ReadLockedPointer<const char> GetName() const noexcept { return name.Get(); }
	ExpressionValue GetValue() const noexcept { return val; }
	int8_t GetScope() const noexcept { return scope; }
	void Assign(ExpressionValue& ev) THROWS(GCodeException);
	void AssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *_ecv_array indices) THROWS(GCodeException) pre(numIndices != 0; indices.lim >= numIndices);
	void AssignArray(size_t numElements, function_ref<ExpressionValue(size_t)>) noexcept;

private:
	StringHandle name;
	ExpressionValue val;
	int16_t scope;								// -1 for a parameter, else the block nesting level when it was created
};

// Class to represent a collection of variables.
// For now this is just a linked list, but it could be changed to a hash table for faster lookup and insertion.
class VariableSet
{
public:
	VariableSet() noexcept : root(nullptr) { }
	~VariableSet();
	VariableSet(const VariableSet&) = delete;
	VariableSet& operator=(const VariableSet& other) = delete;

	void AssignFrom(VariableSet& other) noexcept;

	Variable *_ecv_null Lookup(const char *_ecv_array str, bool wantParameter) noexcept;
	const Variable *_ecv_null Lookup(const char *_ecv_array str, size_t length, bool wantParameter) const noexcept pre(length <= strlen(str));
	Variable *InsertNew(const char *_ecv_array str, ExpressionValue pVal, int16_t pScope) THROWS(GCodeException);
	Variable *InsertNew(const char *_ecv_array str, size_t strLen, ExpressionValue pVal, int16_t pScope) THROWS(GCodeException);
	void InsertNewParameter(const char *_ecv_array str, ExpressionValue pVal) THROWS(GCodeException) { InsertNew(str, pVal, -1); }
	void EndScope(uint8_t blockNesting) noexcept;
	void Delete(const char *_ecv_array str) noexcept;
	void Clear() noexcept;

	void IterateWhile(function_ref_noexcept<bool(unsigned int index, const Variable& v) noexcept> func) const noexcept;

private:
	struct LinkedVariable
	{
		DECLARE_FREELIST_NEW_DELETE(LinkedVariable)

		LinkedVariable(const char *_ecv_array str, size_t strLen, ExpressionValue pVal, int16_t pScope, LinkedVariable *p_next) THROWS(GCodeException)
			: next(p_next), v(str, strLen, pVal, pScope) {}

		LinkedVariable * null next;
		Variable v;
	};

	LinkedVariable * null root;
};

#endif /* SRC_GCODES_VARIABLE_H_ */
