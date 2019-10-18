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
	UploadingNetworkResponder(NetworkResponder *n);

	void ConnectionLost() override;
	virtual void CancelUpload();

	FileStore * StartUpload(const char* folder, const char *fileName, const OpenMode mode, const uint32_t preAllocSize = 0);
	void FinishUpload(uint32_t fileLength, time_t fileLastModified, bool gotCrc, uint32_t expectedCrc);

	// File uploads
	FileData fileBeingUploaded;
	uint32_t uploadedBytes;								// how many bytes have already been written
	bool uploadError;

	String<MaxFilenameLength> filenameBeingProcessed;	// usually the name of the file being uploaded, but also used by HttpResponder and FtpResponder
};

#endif /* SRC_NETWORKING_UPLOADINGNETWORKRESPONDER_H_ */
