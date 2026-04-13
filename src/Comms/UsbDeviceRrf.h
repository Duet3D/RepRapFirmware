/*
 * UsbDevice.h
 *
 *  Created on: 13 Mar 2026
 *      Author: Christian
 *
 */

#ifndef SRC_COMMS_USBDEVICERRF_H_
#define SRC_COMMS_USBDEVICERRF_H_

#include <RepRapFirmware.h>
#include <Comms/AuxDevice.h>
#include <Platform/OutputMemory.h>
#include <RTOSIface/RTOSIface.h>

class SerialCDC;
class GCodeBuffer;

class UsbDeviceRrf
{
public:
	UsbDeviceRrf() noexcept : device(nullptr), originalDevice(nullptr), originalVBusPin(NoPin), seq(0) { }

	void Init(SerialCDC *p_device, Pin vBusPin, const char *mutexName) noexcept;
	void Shutdown() noexcept;
	void Reinit() noexcept;														// Restore USB GCode processing after SBC connection loss
	bool Flush() noexcept;
	void Reset(Pin vBusPin) noexcept;
	void AppendReply(size_t channel, AuxMode channelMode, const GCodeBuffer *_ecv_null gb, OutputBuffer *buffer, bool rawMessage) noexcept;
	void SendRawMessage(size_t channel, AuxMode channelMode, const GCodeBuffer *_ecv_null gb, const char *_ecv_array message, bool rawMessage) noexcept;
	void SendBlockingMessage(const char *_ecv_array message) noexcept;

	bool IsConnected() const noexcept;
	Mutex& GetMutex() noexcept { return mutex; }
	SerialCDC *GetDevice() noexcept { return device; }

private:
	SerialCDC *_ecv_null device;
	SerialCDC *_ecv_null originalDevice;	// stored at Init() time for Reinit()
	Pin originalVBusPin;					// stored at Init() time for Reinit()
	Mutex mutex;
	volatile OutputStack output;
	uint32_t seq;
};

#endif // SRC_COMMS_USBDEVICERRF_H_
