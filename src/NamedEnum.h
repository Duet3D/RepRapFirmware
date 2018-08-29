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
#define STRINGLIST_2(_v1,_v2) #_v1,#_v2
#define STRINGLIST_3(_v1,_v2,_v3) #_v1,#_v2,#_v3
#define STRINGLIST_4(_v1,_v2,_v3,_v4) #_v1,#_v2,#_v3,#_v4
#define STRINGLIST_5(_v1,_v2,_v3,_v4,_v5) #_v1,#_v2,#_v3,#_v4,_v5

// Macro to declare an enumeration with printable value names
// Usage example:
// NamedEnum(MakeOfCar, ford, vauxhall, bmw);
// MakeOfCar mycar(MakeOfCar::bmw);
// myCar = MakeOfCar::ford;
// if (myCar == MakeOfCar::vauxhall) { ... }
// printf("%s", myCar.ToString());
#define NamedEnum(_typename, _v1, ...) \
class _typename { \
public: \
	enum _E : unsigned int { _v1 = 0, __VA_ARGS__ };											/* underlying enumeration */ \
	static constexpr unsigned int NumValues = VA_SIZE(__VA_ARGS__) + 1;							/* count of members */ \
	explicit _typename(_E arg) { v = arg; }														/* constructor */ \
	_typename(const _typename& arg) { v = arg.v; }												/* copy constructor */ \
	bool operator==(_typename arg) const { return v == arg.v; }									/* equality operator */ \
	bool operator!=(_typename arg) const { return v != arg.v; }									/* inequality operator */ \
	bool operator==(_E arg) const { return v == arg; }											/* equality operator */ \
	bool operator!=(_E arg) const { return v != arg; }											/* inequality operator */ \
	const _typename& operator=(_E arg) { v = arg; return *this; }								/* assignment operator from underlying enum */ \
	const _typename& operator=(_typename arg) { v = arg.v; return *this; }						/* copy assignment operator */ \
	unsigned int ToInt() const { return static_cast<unsigned int>(v); }							/* conversion to unsigned integer */ \
	const char* ToString() const { return (v < NumValues) ? _names[v] : "undefined"; }			/* conversion to C string */ \
	void Assign(unsigned int arg) { v = static_cast<_E>(arg); }									/* assignment from integer */ \
private: \
	_E v; \
	static constexpr const char* _names[NumValues] = { STRINGLIST(_v1, __VA_ARGS__) }; \
}

#endif /* SRC_NAMEDENUM_H_ */
