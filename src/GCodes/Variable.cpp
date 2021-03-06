/*
 * Variable.cpp
 *
 *  Created on: 6 Mar 2021
 *      Author: David
 */

#include "Variable.h"

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

void VariableSet::Insert(Variable *toInsert) noexcept
{
	toInsert->next = root;
	root = toInsert;
}

VariableSet::~VariableSet()
{
	while (root != nullptr)
	{
		Variable *v = root;
		root = v->next;
		delete v;
	}
}

// End
