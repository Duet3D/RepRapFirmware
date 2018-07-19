/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_


#ifndef VERSION
#ifdef RTOS
# define RTOSVER		"(RTOS)"
# define MAIN_VERSION	"2.01beta3"
#else
# define MAIN_VERSION	"1.22beta3"
# define RTOSVER
#endif

# define VERSION MAIN_VERSION RTOSVER
#endif

#ifndef DATE
# define DATE "2018-07-18b2"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman"

#endif /* SRC_VERSION_H_ */
