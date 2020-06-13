/*
 * DmacManager.h
 *
 *  Created on: 13 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_DMACMANAGER_H_
#define SRC_HARDWARE_DMACMANAGER_H_

#include "RepRapFirmware.h"

#if SAME70
# include "SAME70/DmacManager_SAME70.h"
#elif SAME5x
# include "SAME5x/DmacManager_SAME5x.h"
#endif

#endif /* SRC_HARDWARE_DMACMANAGER_H_ */
