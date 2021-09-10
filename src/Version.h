/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
# define MAIN_VERSION	"3.3+"
# ifdef USE_CAN0
#  define VERSION_SUFFIX	" (CAN0)"
# else
#  define VERSION_SUFFIX	""
# endif
# define VERSION MAIN_VERSION VERSION_SUFFIX
#endif

#ifndef DATE
# include <General/IsoDate.h>
# define DATE IsoDate
#endif

#if 0
// Use this for official releases
# define TIME_SUFFIX
#else
// Use this for internal builds
# define TIME_SUFFIX		" " __TIME__
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman, printm3d"

#endif /* SRC_VERSION_H_ */
