/*
 * DmacManager.cpp
 *
 *  Created on: 12 Sep 2018
 *      Author: David
 */

#include "DmacManager.h"

#if SAME70

namespace DmacManager
{
	static StandardCallbackFunction callbackFunctions[NumDmaChannelsUsed] = { 0 };
	static CallbackParameter callbackParameters[NumDmaChannelsUsed];

	void Init()
	{
		pmc_enable_periph_clk(ID_XDMAC);
	}

	void SetInterruptCallback(const uint8_t channel, StandardCallbackFunction fn, CallbackParameter param)
	{
		if (channel < NumDmaChannelsUsed)
		{
			callbackFunctions[channel] = fn;
			callbackParameters[channel] = param;
		}
	}
}

// DMAC interrupt service routine
extern "C" void XDMAC_Handler()
{
	uint32_t pendingChannels = XDMAC->XDMAC_GIS;
	for (size_t i = 0; i < NumDmaChannelsUsed; ++i)
	{
		if ((pendingChannels & 1u) != 0)
		{
			if (DmacManager::callbackFunctions[i] != nullptr)
			{
				DmacManager::callbackFunctions[i](DmacManager::callbackParameters[i]);	// we rely on the callback to clear the interrupt
			}
			else
			{
				XDMAC->XDMAC_CHID[i].XDMAC_CID = 0xFFFFFFFF;							// no callback, so just clear the interrupt
			}
		}
		pendingChannels >>= 1;
	}
}

#endif

// End
