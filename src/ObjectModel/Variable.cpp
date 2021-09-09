/*
 * Variable.cpp
 *
 *  Created on: 6 Mar 2021
 *      Author: David
 */

#include "Variable.h"
#include <Platform/OutputMemory.h>

Variable::Variable(const char *str, ExpressionValue pVal, int8_t pScope) noexcept : name(str), val(pVal), scope(pScope)
{
}

Variable::~Variable()
{
	name.Delete();
	val.Release();
}

Variable* VariableSet::Lookup(const char *str) noexcept
{
	LinkedVariable *lv;
	for (lv = root; lv != nullptr; lv = lv->next)
	{
		auto vname = lv->v.GetName();
		if (strcmp(vname.Ptr(), str) == 0)
		{
			return &(lv->v);
		}
	}
	return nullptr;
}

const Variable* VariableSet::Lookup(const char *str) const noexcept
{
	const LinkedVariable *lv;
	for (lv = root; lv != nullptr; lv = lv->next)
	{
		auto vname = lv->v.GetName();
		if (strcmp(vname.Ptr(), str) == 0)
		{
			return &(lv->v);
		}
	}
	return nullptr;
}

void VariableSet::InsertNew(const char *str, ExpressionValue pVal, int8_t pScope) noexcept
{
	LinkedVariable * const toInsert = new LinkedVariable(str, pVal, pScope, root);
	root = toInsert;
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

void VariableSet::IterateWhile(function_ref<bool(unsigned int, const Variable&) /*noexcept*/ > func) const noexcept
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
