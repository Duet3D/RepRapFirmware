/*
 * NamedEnum.h
 *
 *  Created on: 27 Aug 2018
 *      Author: David
 */

#ifndef SRC_NAMEDENUM_H_
#define SRC_NAMEDENUM_H_

// Plumbing to allow overloaded STRINGLIST macro
#define CAT( A, B ) A ## B
#define SELECT( NAME, NUM ) CAT( NAME ## _, NUM )
#define GET_COUNT( _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, COUNT, ... ) COUNT
#define VA_SIZE( ... ) GET_COUNT( __VA_ARGS__, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 )
#define VA_SELECT( NAME, ... ) SELECT( NAME, VA_SIZE(__VA_ARGS__) )(__VA_ARGS__)

// Macro to turn a list of names into a list of strings
#define STRINGLIST( ... ) VA_SELECT( STRINGLIST, __VA_ARGS__ )
#define STRINGLIST_2(_v1,_v2)												#_v1,#_v2
#define STRINGLIST_3(_v1,_v2,_v3)											#_v1,#_v2,#_v3
#define STRINGLIST_4(_v1,_v2,_v3,_v4)										#_v1,#_v2,#_v3,#_v4
#define STRINGLIST_5(_v1,_v2,_v3,_v4,_v5)									#_v1,#_v2,#_v3,#_v4,#_v5
#define STRINGLIST_6(_v1,_v2,_v3,_v4,_v5,_v6)								#_v1,#_v2,#_v3,#_v4,#_v5,#_v6
#define STRINGLIST_7(_v1,_v2,_v3,_v4,_v5,_v6,_v7)							#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7
#define STRINGLIST_8(_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8)						#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7,#_v8
#define STRINGLIST_9(_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8,_v9)					#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7,#_v8,#_v9
#define STRINGLIST_10(_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8,_v9,_v10)				#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7,#_v8,#_v9,#_v10
#define STRINGLIST_11(_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8,_v9,_v10,_v11)		#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7,#_v8,#_v9,#_v10,#_v11
#define STRINGLIST_12(_v1,_v2,_v3,_v4,_v5,_v6,_v7,_v8,_v9,_v10,_v11,_v12)	#_v1,#_v2,#_v3,#_v4,#_v5,#_v6,#_v7,#_v8,#_v9,#_v10,#_v11,#_v12

// Macro to declare an enumeration with printable value names
// Usage example:
// NamedEnum(MakeOfCar, ford, vauxhall, bmw);
// MakeOfCar mycar(MakeOfCar::bmw);
// myCar = MakeOfCar::ford;
// if (myCar == MakeOfCar::vauxhall) { ... }
// printf("%s", myCar.ToString());
#define NamedEnum(_typename, _baseType, _v1, ...) \
class _typename { \
public: \
	enum _E : _baseType { _v1 = 0, __VA_ARGS__ };												/* underlying enumeration */ \
	static constexpr unsigned int NumValues = VA_SIZE(__VA_ARGS__) + 1;							/* count of members */ \
	_typename(_E arg) noexcept { v = arg; }														/* constructor */ \
	explicit _typename(_baseType arg) noexcept { v = static_cast<_E>(arg); }					/* constructor */ \
	_typename(const _typename& arg) noexcept { v = arg.v; }										/* copy constructor */ \
	bool operator==(_typename arg) const noexcept { return v == arg.v; }						/* equality operator */ \
	bool operator!=(_typename arg) const noexcept { return v != arg.v; }						/* inequality operator */ \
	bool operator>(_typename arg) const noexcept { return v > arg.v; }							/* greater-than operator */ \
	bool operator>=(_typename arg) const noexcept { return v >= arg.v; }						/* greater-than-or-equal operator */ \
	bool operator<(_typename arg) const noexcept { return v < arg.v; }							/* less-than operator */ \
	bool operator<=(_typename arg) const noexcept { return v <= arg.v; }						/* less-than-or-equal operator */ \
	bool operator==(_E arg) const noexcept { return v == arg; }									/* equality operator */ \
	bool operator!=(_E arg) const noexcept { return v != arg; }									/* inequality operator */ \
	bool operator>(_E arg) const noexcept { return v > arg; }									/* greater-than operator */ \
	bool operator>=(_E arg) const noexcept { return v >= arg; }									/* greater-than-or-equal operator */ \
	bool operator<(_E arg) const noexcept { return v < arg; }									/* less-than operator */ \
	bool operator<=(_E arg) const noexcept { return v <= arg; }									/* less-than-or-equal operator */ \
	const _typename& operator=(_E arg) noexcept { v = arg; return *this; }						/* assignment operator from underlying enum */ \
	const _typename& operator=(_typename arg) noexcept { v = arg.v; return *this; }				/* copy assignment operator */ \
	constexpr _baseType ToBaseType() const noexcept { return static_cast<_baseType>(v); }		/* conversion to integral base type */ \
	const char* ToString() const noexcept { return (v < NumValues) ? _names[v] : "undefined"; }	/* conversion to C string */ \
	void Assign(_baseType arg) noexcept { v = static_cast<_E>(arg); }							/* assignment from integral base type */ \
private: \
	_E v; \
	static constexpr const char* _names[NumValues] = { STRINGLIST(_v1, __VA_ARGS__) }; \
}

#endif /* SRC_NAMEDENUM_H_ */
