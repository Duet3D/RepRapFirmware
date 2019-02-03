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

	void StartUpload(FileStore *file, const char *fileName);
	void FinishUpload(uint32_t fileLength, time_t fileLastModified);
	virtual void CancelUpload();

	// File uploads
	FileData fileBeingUploaded;
	String<MaxFilenameLength> filenameBeingProcessed;	// usually the name of the file being uploaded, but also used by HttpResponder and FtpResponder
	uint32_t postFileLength, uploadedBytes;				// how many POST bytes do we expect and how many have already been written?
	time_t fileLastModified;
	bool uploadError;
};

#endif /* SRC_NETWORKING_UPLOADINGNETWORKRESPONDER_H_ */
