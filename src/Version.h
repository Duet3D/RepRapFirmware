/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
# define MAIN_VERSION	"3.01-RC6"
# ifdef USE_CAN0
#  define VERSION_SUFFIX	" (CAN0)"
# else
#  define VERSION_SUFFIX	""
# endif
# define VERSION MAIN_VERSION VERSION_SUFFIX
#endif

#ifndef DATE
# define DATE "2020-04-02b2"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman, printm3d"

#endif /* SRC_VERSION_H_ */
