/*
 * DataCollection.cpp
 *
 *  Created on: 26 Aug 2024
 *      Author: Andy Everitt
 */

#include "DataCollection.h"

#if SUPPORT_DATA_COLLECTION

#include <DmacManager.h>
#include <Heating/Heat.h>
#include <Heating/Sensors/LinearAnalogSensor.h>
#include <Movement/Move.h>
#include <Platform/Platform.h>
#include <Platform/RepRap.h>
#include <RepRapFirmware.h>

#if SAME70

#include <pmc/pmc.h>
#include <xdmac/xdmac.h>

#endif

namespace DataCollection
{
	static uint32_t lastTransmissionTime = 0;

	static __nocache volatile uint8_t buffer[MaxBufferLen];
	static __nocache volatile size_t bufferLen = 0;

	void ClearBuffer()
	{
		for (size_t i = 0; i < MaxBufferLen; i++)
		{
			buffer[i] = 0;
		}
		bufferLen = 0;
	}

	bool AddDataToBuffer(uint8_t val)
	{
		if (bufferLen >= MaxBufferLen)
		{
			return false;
		}

		buffer[bufferLen] = val;
		bufferLen++;
		return true;
	}

	// Set up the PDC or DMAC to send a register and receive the status, but don't enable it yet
	static void SetupDMA() noexcept
	{
#if SAME70
		/* From the data sheet:
		 * Single Block Transfer With Single Microblock
			1. Read the XDMAC Global Channel Status Register (XDMAC_GS) to select a free channel. [we use fixed channel
		 numbers instead.]
			2. Clear the pending Interrupt Status bit(s) by reading the selected XDMAC Channel x Interrupt Status
			Register (XDMAC_CISx).
			3. Write the XDMAC Channel x Source Address Register (XDMAC_CSAx) for channel x.
			4. Write the XDMAC Channel x Destination Address Register (XDMAC_CDAx) for channel x.
			5. Program field UBLEN in the XDMAC Channel x Microblock Control Register (XDMAC_CUBCx) with
			the number of data.
			6. Program the XDMAC Channel x Configuration Register (XDMAC_CCx):
			6.1. Clear XDMAC_CCx.TYPE for a memory-to-memory transfer, otherwise set this bit.
			6.2. Configure XDMAC_CCx.MBSIZE to the memory burst size used.
			6.3. Configure XDMAC_CCx.SAM and DAM to Memory Addressing mode.
			6.4. Configure XDMAC_CCx.DSYNC to select the peripheral transfer direction.
			6.5. Configure XDMAC_CCx.CSIZE to configure the channel chunk size (only relevant for
			peripheral synchronized transfer).
			6.6. Configure XDMAC_CCx.DWIDTH to configure the transfer data width.
			6.7. Configure XDMAC_CCx.SIF, XDMAC_CCx.DIF to configure the master interface used to
			read data and write data, respectively.
			6.8. Configure XDMAC_CCx.PERID to select the active hardware request line (only relevant for
			a peripheral synchronized transfer).
			6.9. Set XDMAC_CCx.SWREQ to use a software request (only relevant for a peripheral
			synchronized transfer).
			7. Clear the following five registers:
			– XDMAC Channel x Next Descriptor Control Register (XDMAC_CNDCx)
			– XDMAC Channel x Block Control Register (XDMAC_CBCx)
			– XDMAC Channel x Data Stride Memory Set Pattern Register (XDMAC_CDS_MSPx)
			– XDMAC Channel x Source Microblock Stride Register (XDMAC_CSUSx)
			– XDMAC Channel x Destination Microblock Stride Register (XDMAC_CDUSx)
			This indicates that the linked list is disabled, there is only one block and striding is disabled.
			8. Enable the Microblock interrupt by writing a ‘1’ to bit BIE in the XDMAC Channel x Interrupt Enable
			Register (XDMAC_CIEx). Enable the Channel x Interrupt Enable bit by writing a ‘1’ to bit IEx in the
			XDMAC Global Interrupt Enable Register (XDMAC_GIE).
			9. Enable channel x by writing a ‘1’ to bit ENx in the XDMAC Global Channel Enable Register
			(XDMAC_GE). XDMAC_GS.STx (XDMAC Channel x Status bit) is set by hardware.
			10. Once completed, the DMA channel sets XDMAC_CISx.BIS (End of Block Interrupt Status bit) and
			generates an interrupt. XDMAC_GS.STx is cleared by hardware. The software can either wait for
			an interrupt or poll the channel status bit.

			The following code is adapted from the code in the HSMCI driver instead.
		*/
		// Transmit
		{
			xdmac_channel_disable(XDMAC, DmacChanDataCollectionTx);
			xdmac_channel_config_t p_cfg = {0, 0, 0, 0, 0, 0, 0, 0};
			p_cfg.mbr_cfg = XDMAC_CC_TYPE_PER_TRAN | XDMAC_CC_MBSIZE_SINGLE | XDMAC_CC_DSYNC_MEM2PER |
							XDMAC_CC_CSIZE_CHK_1 | XDMAC_CC_DWIDTH_BYTE | XDMAC_CC_SIF_AHB_IF0 | XDMAC_CC_DIF_AHB_IF1 |
							XDMAC_CC_SAM_INCREMENTED_AM | XDMAC_CC_DAM_FIXED_AM | XDMAC_CC_PERID(UART2_DmaTxPerid);
			p_cfg.mbr_ubc = bufferLen;
			p_cfg.mbr_sa = reinterpret_cast<uint32_t>(buffer);
			p_cfg.mbr_da = reinterpret_cast<uint32_t>(&(UART_DataCollection->UART_THR));
			xdmac_configure_transfer(XDMAC, DmacChanDataCollectionTx, &p_cfg);
		}
#endif
	}

	static inline void EnableDma() noexcept
	{
#if SAME70
		xdmac_channel_enable(XDMAC, DmacChanDataCollectionTx);
#endif
	}

	// Convert a float to an ASCII array of length 5, implied decimal place before last 2 characters
	static bool SerialiseFloat(float input, uint8_t* output, size_t &len, size_t decimals)
	{
		int limit = 1;
		const size_t decimalAndSignChars = decimals > 0 ? decimals + 2 : 1;
		for (size_t i = decimalAndSignChars; i < len; i++)
		{
			limit *= 10;
		}

		if (unlikely(abs(input) >= limit))
		{
			float decimalResolution = decimals > 0 ? pow(0.1, decimals) : 0;
			input = input < 0 ? -(limit - decimalResolution) : (limit - decimalResolution);
		}

		char charBuf[20];
		SafeSnprintf(charBuf, min(ARRAY_SIZE(charBuf), len+1), "%.*f", decimals, (double)input);
		len = strlen(charBuf);

		for (size_t i = 0; i < len; i++)
		{
			output[i] = static_cast<uint8_t>(charBuf[i]);
		}
		return true;
	}

	bool SendDataToUart()
	{
		SetupDMA();
		EnableDma();
		return true;
	}

	inline bool AddDataToBuffer(uint32_t val)
	{
		bool failed = false;
		char buf[11];
		SafeSnprintf(buf, 11, "%lu", val);
		for (size_t i = 0; i < strlen(buf); i++)
		{
			failed |= AddDataToBuffer((uint8_t)buf[i]);
		}
		return !failed;
	}

	bool AddDataToBuffer(uint8_t *data, size_t len)
	{
		bool failed = false;
		for (size_t i = 0; i < len; i++)
		{
			failed |= AddDataToBuffer(data[i]);
		}
		return !failed;
	}

	static void AddAxisPosition(size_t axisOrExtruder)
	{
		Move& move = reprap.GetMove();

		int32_t currentMicrosteps = move.GetLiveMotorPosition(axisOrExtruder);
		float pos = currentMicrosteps / move.DriveStepsPerMm(axisOrExtruder);
		size_t len = 7;
		uint8_t asciiPos[len+1] = {0};
		SerialiseFloat(pos, asciiPos, len, 2);
		AddDataToBuffer(asciiPos, len);
	}

	void CollectAndSendData()
	{
		ClearBuffer();

		// Add timestamp
		lastTransmissionTime = millis();
		AddDataToBuffer(lastTransmissionTime);
		AddDataToBuffer((uint8_t)',');

		// Add X,Y,Z position to buffer
		for (size_t axis = 0; axis <= 2; axis++)
		{
			AddAxisPosition(axis);
			AddDataToBuffer((uint8_t)',');
		}

		// Add E0 position to buffer
		AddAxisPosition(ExtruderToLogicalDrive(0));

		// Add analog sensor
		// TODO will need to poll this faster
		for (size_t i = 0; i < ARRAY_SIZE(AnalogSensors); i++)
		{
			AddDataToBuffer((uint8_t)',');
			const AnalogSensorInfo sensorInfo = AnalogSensors[i];
			const auto sensor = reprap.GetHeat().FindSensor(sensorInfo.number);
			if (sensor.IsNotNull())
			{
				float fReading;
				size_t len = 7;				// max linearAnalog reading is 5 digits, temperature to 1 d.p. can be 6 digits.
				uint8_t asciiReading[len+1] = {0};
				sensor->GetLatestTemperature(fReading);
				SerialiseFloat(fReading, asciiReading, len, sensorInfo.decimals);
				AddDataToBuffer(asciiReading, len);
			}
		}

		AddDataToBuffer((uint8_t)'\n');

		SendDataToUart();
	}

	uint32_t GetLastTransmissionTime()
	{
		return lastTransmissionTime;
	}
}
#endif


