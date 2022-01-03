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

// Class to represent a variable having a name and a value
class Variable
{
public:
	Variable(const char *_ecv_array str, ExpressionValue pVal, int8_t pScope) noexcept;
	~Variable();

	ReadLockedPointer<const char> GetName() const noexcept { return name.Get(); }
	ExpressionValue GetValue() const noexcept { return val; }
	int8_t GetScope() const noexcept { return scope; }
	void Assign(ExpressionValue ev) noexcept { val = ev; }

private:
	StringHandle name;
	ExpressionValue val;
	int8_t scope;								// -1 for a parameter, else the block nesting level when it was created
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

	Variable *Lookup(const char *_ecv_array str) noexcept;
	const Variable *Lookup(const char *_ecv_array str) const noexcept;
	void InsertNew(const char *str, ExpressionValue pVal, int8_t pScope) noexcept;
	void InsertNewParameter(const char *str, ExpressionValue pVal) noexcept { InsertNew(str, pVal, -1); }
	void EndScope(uint8_t blockNesting) noexcept;
	void Delete(const char *str) noexcept;
	void Clear() noexcept;

	void IterateWhile(function_ref<bool(unsigned int index, const Variable& v) /*noexcept*/ > func) const noexcept;

private:
	struct LinkedVariable
	{
		DECLARE_FREELIST_NEW_DELETE(LinkedVariable)

		LinkedVariable(const char *_ecv_array str, ExpressionValue pVal, int8_t pScope, LinkedVariable *p_next) : next(p_next), v(str, pVal, pScope) {}

		LinkedVariable * null next;
		Variable v;
	};

	LinkedVariable * null root;
};

#endif /* SRC_GCODES_VARIABLE_H_ */
