/*
 * SmartDrivers.h
 *
 *  Created on: 10 Jan 2025
 *      Author: David
 */

#ifndef SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_
#define SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_

#if SUPPORT_TMC2660
# include "TMC2660.h"
#endif
#if SUPPORT_TMC22xx
# include "TMC22xx.h"
#endif
#if SUPPORT_TMC51xx
# include "TMC51xx.h"
#endif

#endif /* SRC_MOVEMENT_STEPPERDRIVERS_SMARTDRIVERS_H_ */
