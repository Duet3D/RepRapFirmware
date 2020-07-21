/*
 * UploadingNetworkResponder.h
 *
 *  Created on: 3 Feb 2019
 *      Author: David
 */

#ifndef SRC_NETWORKING_UPLOADINGNETWORKRESPONDER_H_
#define SRC_NETWORKING_UPLOADINGNETWORKRESPONDER_H_

#include "NetworkResponder.h"

class UploadingNetworkResponder : public NetworkResponder
{
protected:
	UploadingNetworkResponder(NetworkResponder *n) noexcept;

	void ConnectionLost() noexcept override;
	virtual void CancelUpload() noexcept;

#if HAS_MASS_STORAGE
	bool StartUpload(const char* folder, const char *fileName, const OpenMode mode, const uint32_t preAllocSize = 0) noexcept;
	void FinishUpload(uint32_t fileLength, time_t fileLastModified, bool gotCrc, uint32_t expectedCrc) noexcept;

	// File uploads
	FileData fileBeingUploaded;
	uint32_t uploadedBytes;								// how many bytes have already been written
	bool uploadError;
	bool dummyUpload;
#endif

	String<MaxFilenameLength> filenameBeingProcessed;	// usually the name of the file being uploaded, but also used by HttpResponder and FtpResponder
};

#endif /* SRC_NETWORKING_UPLOADINGNETWORKRESPONDER_H_ */
