/*
 * Variable.cpp
 *
 *  Created on: 6 Mar 2021
 *      Author: David
 */

#include "Variable.h"
#include <Platform/Outputmemory.h>

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
	Variable *v;
	for (v = root; v != nullptr; v = v->next)
	{
		auto vname = v->name.Get();
		if (strcmp(vname.Ptr(), str) == 0)
		{
			break;
		}
	}
	return v;
}

const Variable* VariableSet::Lookup(const char *str) const noexcept
{
	const Variable *v;
	for (v = root; v != nullptr; v = v->next)
	{
		auto vname = v->name.Get();
		if (strcmp(vname.Ptr(), str) == 0)
		{
			break;
		}
	}
	return v;
}

void VariableSet::Insert(Variable *toInsert) noexcept
{
	toInsert->next = root;
	root = toInsert;
}

void VariableSet::ListAsJson(OutputBuffer *buf) const noexcept
{
	bool needComma = false;
	for (const Variable *v = root; v != nullptr; v = v->next)
	{
		auto vname = v->name.Get();
		buf->catf((needComma) ? ",\"%s\":" : "\"%s\":", vname.Ptr());
		buf->cat("42");		//TEMP!
		//v->val.qq;
	//qq;
		needComma = true;
	}
}

// Remove all variables with a scope greater than the parameter
void VariableSet::EndScope(uint8_t blockNesting) noexcept
{
	Variable *prev = nullptr;
	for (Variable *v = root; v != nullptr; )
	{
		if (v->scope > blockNesting)
		{
			Variable *temp = v;
			v = v->next;
			if (prev == nullptr)
			{
				root = v;
			}
			else
			{
				prev->next = v;
			}
			delete temp;
		}
		else
		{
			prev = v;
			v = v->next;
		}
	}
}

void VariableSet::Clear() noexcept
{
	while (root != nullptr)
	{
		Variable *v = root;
		root = v->next;
		delete v;
	}
}

VariableSet::~VariableSet()
{
	Clear();
}

// End
