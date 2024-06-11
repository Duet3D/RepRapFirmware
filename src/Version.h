/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifndef VERSION
// Note: the complete VERSION string must be in standard version number format and must not contain spaces! This is so that DWC can parse it.
# define MAIN_VERSION	"3.5.2"
# ifdef USE_CAN0
#  define VERSION_SUFFIX	"(CAN0)"
# else
#  define VERSION_SUFFIX	""
# endif
# define VERSION MAIN_VERSION VERSION_SUFFIX
#endif

extern const char *const DATE;
extern const char *const TIME_SUFFIX;

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman, printm3d"

#endif /* SRC_VERSION_H_ */
