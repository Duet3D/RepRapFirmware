/*
 * Variable.cpp
 *
 *  Created on: 6 Mar 2021
 *      Author: David
 */

#include "Variable.h"
#include <Platform/OutputMemory.h>
#include <GCodes/GCodeBuffer/GCodeBuffer.h>

// Members of class Variable
Variable::Variable(const char *str, ExpressionValue& pVal, int16_t pScope) THROWS(GCodeException)
	: name(str), val(), scope(pScope)
{
	Assign(pVal);				// this may throw
}

Variable::~Variable()
{
	name.Delete();
	val.Release();
}

bool Variable::IsValidVariableName(const char *str) noexcept
{
	size_t len = strlen(str);
	if (len == 0)
	{
		return false;
	}

	if (!isalpha(str[0]))
	{
		return false;
	}

	for (size_t i = 1; i < len; i++)
	{
		if (!isalnum(str[i]) && str[i] != '_')
		{
			return false;
		}
	}
	return true;
}

// Assign a new value to this variable
void Variable::Assign(ExpressionValue& ev) THROWS(GCodeException)
{
	switch (ev.GetType())
	{
	case TypeCode::ObjectModelArray:
		{
			// Copy the object model array value to the heap
			ArrayHandle ah;
			const ObjectModelArrayTableEntry *const entry = ev.omVal->FindObjectModelArrayEntry(ev.param & 0xFF);
			if (entry == nullptr)
			{
				throw GCodeException("Failed to lookup object model array");
			}
			ReadLocker lock(entry->lockPointer);
			ObjectExplorationContext context;
			context.AddIndex(ev.param >> 8);								// in case it is a 2D array
			const size_t numElements = entry->GetNumElements(ev.omVal, context);
			if (numElements != 0)
			{
				WriteLocker locker(Heap::heapLock);							// prevent other tasks modifying the heap
				ah.Allocate(numElements);
				for (size_t i = 0; i < numElements; ++i)
				{
					context.AddIndex(i);
					ExpressionValue elemVal = entry->GetElement(ev.omVal, context);
					ah.AssignElement(i, elemVal);
					context.RemoveIndex();
				}
			}
			val = ExpressionValue(ah);
		}
		break;

	case TypeCode::ObjectModel_tc:
		throw GCodeException("Cannot assign a value of type 'object' to a variable");

	default:
		val = ev;
		break;
	}
}

// Assign a new value to an indexed part of this variable. There is always at least one index.
void Variable::AssignIndexed(const ExpressionValue& ev, size_t numIndices, const uint32_t *indices) THROWS(GCodeException)
{
	if (val.GetType() != TypeCode::HeapArray)
	{
		throw GCodeException("Expected an array expression");
	}
	val.ahVal.AssignIndexed(ev, numIndices, indices);
}

// Create a new array and assign it to this variable
void Variable::AssignArray(size_t numElements, function_ref<ExpressionValue(size_t)> func) noexcept
{
	ArrayHandle ah;
	WriteLocker locker(Heap::heapLock);						// prevent other tasks modifying the heap
	ah.Allocate(numElements);
	for (size_t i = 0; i < numElements; ++i)
	{
		ExpressionValue elem = func(i);
		ah.AssignElement(i, elem);
	}
	val.SetArrayHandle(ah);
}

// Members of class VariableSet
Variable *_ecv_null VariableSet::Lookup(const char *str, bool wantParameter) noexcept
{
	LinkedVariable *lv;
	for (lv = root; lv != nullptr; lv = lv->next)
	{
		auto vname = lv->v.GetName();
		if (strcmp(vname.Ptr(), str) == 0 && (wantParameter == (lv->v.GetScope() == -1)))
		{
			return &(lv->v);
		}
	}
	return nullptr;
}

const Variable *_ecv_null VariableSet::Lookup(const char *str, size_t length, bool wantParameter) const noexcept
{
	const LinkedVariable *lv;
	for (lv = root; lv != nullptr; lv = lv->next)
	{
		auto vname = lv->v.GetName();
		if (strlen(vname.Ptr()) == length && memcmp(vname.Ptr(), str, length) == 0 && (wantParameter == (lv->v.GetScope() == -1)))
		{
			return &(lv->v);
		}
	}
	return nullptr;
}

Variable *VariableSet::InsertNew(const char *str, ExpressionValue pVal, int16_t pScope) THROWS(GCodeException)
{
	LinkedVariable * const toInsert = new LinkedVariable(str, pVal, pScope, root);
	root = toInsert;
	return &(toInsert->v);
}

// Remove all variables with a scope greater than the parameter
void VariableSet::EndScope(uint8_t blockNesting) noexcept
{
	LinkedVariable *prev = nullptr;
	for (LinkedVariable *lv = root; lv != nullptr; )
	{
		if (lv->v.GetScope() > blockNesting)
		{
			LinkedVariable *temp = lv;
			lv = lv->next;
			if (prev == nullptr)
			{
				root = lv;
			}
			else
			{
				prev->next = lv;
			}
			delete temp;
		}
		else
		{
			prev = lv;
			lv = lv->next;
		}
	}
}

void VariableSet::Delete(const char *str) noexcept
{
	LinkedVariable *prev = nullptr;
	for (LinkedVariable *lv = root; lv != nullptr; lv = lv->next)
	{
		auto vname = lv->v.GetName();
		if (strcmp(vname.Ptr(), str) == 0)
		{
			if (prev == nullptr)
			{
				root = lv->next;
			}
			else
			{
				prev->next = lv->next;
			}
			delete lv;
			break;
		}
		prev = lv;
	}
}

void VariableSet::Clear() noexcept
{
	while (root != nullptr)
	{
		LinkedVariable *lv = root;
		root = lv->next;
		delete lv;
	}
}

VariableSet::~VariableSet()
{
	Clear();
}

// Delete any existing variables and instead use the variables from the argument, taking ownership of them
void VariableSet::AssignFrom(VariableSet& other) noexcept
{
	Clear();
	root = other.root;
	other.root = nullptr;
}

void VariableSet::IterateWhile(function_ref_noexcept<bool(unsigned int, const Variable&) noexcept> func) const noexcept
{
	unsigned int num = 0;
	for (const LinkedVariable *lv = root; lv != nullptr; lv = lv->next)
	{
		if (!func(num, lv->v))
		{
			break;
		}
		++num;
	}
}

// End
