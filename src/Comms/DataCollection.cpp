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
#include <GCodes/GCodes.h>
#include <Storage/CRC16.h>

#if SAME70

#include <pmc/pmc.h>
#include <xdmac/xdmac.h>

#endif

namespace DataCollection
{
	static uint32_t lastTransmissionTime = 0;

	static constexpr size_t MaxBufferLen = 100;		// max number of bytes supported in a single message
	static constexpr size_t CrcFieldDigits = 5;
	static constexpr size_t MessageSuffixLen = 1 + CrcFieldDigits + 1; // '*' + CRC16 + '\n'
	static __nocache volatile uint8_t buffer[MaxBufferLen];
	static __nocache volatile size_t bufferLen = 0;

	[[nodiscard]] bool IsTxInProgress() noexcept
	{
#  if SAME70
		return (xdmac_channel_get_status(XDMAC) & (XDMAC_GS_ST0 << DmacChanDataCollectionTx)) != 0;
#  else
		return false;
#  endif
	}

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
		TaskCriticalSectionLocker locker;
		if (bufferLen >= MaxBufferLen - MessageSuffixLen)
		{
			return false;
		}

		buffer[bufferLen] = val;
		bufferLen += 1;
		return true;
	}

	static bool AppendCrcAndNewline()
	{
		CRC16 crc;
		for (size_t i = 0; i < bufferLen; i++)
		{
			crc.Update(buffer[i]);
		}

		char crcText[CrcFieldDigits + 1];
		SafeSnprintf(crcText, ARRAY_SIZE(crcText), "%05u", static_cast<unsigned int>(crc.Get()));

		if (!AddDataToBuffer((uint8_t)'*'))
		{
			return false;
		}

		for (size_t i = 0; i < CrcFieldDigits; i++)
		{
			if (!AddDataToBuffer(static_cast<uint8_t>(crcText[i])))
			{
				return false;
			}
		}

		return AddDataToBuffer((uint8_t)'\n');
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
		float limit = 1;
		const size_t decimalAndSignChars = decimals > 0 ? decimals + 2 : 1;
		for (size_t i = decimalAndSignChars; i < len; i++)
		{
			limit *= 10;
		}

		if (unlikely(fabsf(input) >= limit))
		{
			float decimalResolution = 0.0f;
			if (decimals > 0)
			{
				decimalResolution = 1.0f;
				for (size_t i = 0; i < decimals; i++)
				{
					decimalResolution *= 0.1f;
				}
			}
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

	bool AddDataToBuffer(float val, size_t len, size_t decimals)
	{
		uint8_t asciiPos[len+1] = {0};
		SerialiseFloat(val, asciiPos, len, decimals);
		return AddDataToBuffer(asciiPos, len);
	}

	void CollectAndSendData()
	{
		if (IsTxInProgress())
		{
			return;
		}

		ClearBuffer();

		// Add timestamp
		lastTransmissionTime = millis();
		AddDataToBuffer(lastTransmissionTime % 1000);

		// Add X,Y,Z position to buffer
		Move& move = reprap.GetMove();
		float coords[MaxAxes];
		move.UpdateLiveMachineCoordinates(coords, reprap.GetGCodes().GetPrimaryMovementState().currentTool);
		for (size_t axis = 0; axis < reprap.GetGCodes().GetTotalAxes(); axis++)
		{
			AddDataToBuffer((uint8_t)',');
			AddDataToBuffer(coords[axis], 7, 1);
		}

		// Add all extruder positions to buffer
		for (size_t ms = 0; ms < NumMovementSystems; ms++)
		{
			const bool extruding = move.GetTotalExtrusionRate(ms) != 0;
			const int currentTool = reprap.GetGCodes().GetMovementState(ms).GetCurrentToolNumber();
			const int toolToReport = extruding ? currentTool : -1;
			AddDataToBuffer((uint8_t)',');
			if (toolToReport >= 0)
			{
				AddDataToBuffer(static_cast<uint32_t>(toolToReport));
			}
			else
			{
				AddDataToBuffer((uint8_t)'-');
				AddDataToBuffer((uint8_t)'1');
			}
		}

		// Add analog sensor
		// TODO will need to poll this faster
		for (size_t i = 0; i < ARRAY_SIZE(AnalogSensors); i++)
		{
			AddDataToBuffer((uint8_t)',');
			const AnalogSensorInfo sensorInfo = AnalogSensors[i];
			const auto sensor = reprap.GetHeat().FindSensor(sensorInfo.number);
			if (sensor.IsNotNull())
			{
				sensor->Poll();
				float fReading;
				size_t len = 7;				// max linearAnalog reading is 5 digits, temperature to 1 d.p. can be 6 digits.
				uint8_t asciiReading[len+1] = {0};
				sensor->GetLatestTemperature(fReading);
				SerialiseFloat(fReading, asciiReading, len, sensorInfo.decimals);
				AddDataToBuffer(asciiReading, len);
			}
		}

		AppendCrcAndNewline();

		SendDataToUart();
	}

	uint32_t GetLastTransmissionTime()
	{
		return lastTransmissionTime;
	}
}
#endif


