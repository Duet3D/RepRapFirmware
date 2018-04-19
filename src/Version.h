/*
 * Version.h
 *
 *  Created on: 25 Dec 2016
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#ifdef RTOS
# define RTOSVER "(RTOS)"
#else
# define RTOSVER
#endif

#ifndef VERSION
# define VERSION "2.0" RTOSVER "beta2"
#endif

#ifndef DATE
# define DATE "2018-04-19b1"
#endif

#define AUTHORS "reprappro, dc42, chrishamm, t3p3, dnewman"

#endif /* SRC_VERSION_H_ */
