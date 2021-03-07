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

// Class to represent a variable having a name and a value
class Variable
{
public:
	friend class VariableSet;

	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<Variable>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<Variable>(p); }

	Variable(const char *str, ExpressionValue pVal, int8_t pScope) noexcept;
	~Variable();

	ExpressionValue GetValue() const { return val; }
	int8_t GetScope() const { return scope; }

private:
	Variable *next;
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

	Variable *Lookup(const char *str) noexcept;
	void Insert(Variable *toInsert) noexcept;

private:
	Variable *root;
};

#endif /* SRC_GCODES_VARIABLE_H_ */
