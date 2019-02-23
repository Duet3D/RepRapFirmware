/*
 * UploadingNetworkResponder.cpp
 *
 *  Created on: 3 Feb 2019
 *      Author: David
 */

#include "UploadingNetworkResponder.h"
#include "Socket.h"
#include "Platform.h"

UploadingNetworkResponder::UploadingNetworkResponder(NetworkResponder *n) : NetworkResponder(n), uploadError(false)
{
}

// This is called when we lose a connection or when we are asked to terminate. Overridden in some derived classes.
void UploadingNetworkResponder::ConnectionLost()
{
	CancelUpload();
	NetworkResponder::ConnectionLost();
}

// Start writing to a new file
void UploadingNetworkResponder::StartUpload(FileStore *file, const char *fileName)
{
	fileBeingUploaded.Set(file);
	filenameBeingProcessed.copy(fileName);
	responderState = ResponderState::uploading;
	uploadError = false;
}

// If this responder has an upload in progress, cancel it
void UploadingNetworkResponder::CancelUpload()
{
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
		if (!filenameBeingProcessed.IsEmpty())
		{
			GetPlatform().GetMassStorage()->Delete(filenameBeingProcessed.c_str());
			filenameBeingProcessed.Clear();
		}
	}
}

// Finish a file upload. Set variable uploadError if anything goes wrong.
void UploadingNetworkResponder::FinishUpload(uint32_t fileLength, time_t fileLastModified)
{
	// Flush remaining data for FSO
	if (!fileBeingUploaded.Flush())
	{
		uploadError = true;
		GetPlatform().Message(ErrorMessage, "Could not flush remaining data while finishing upload!\n");
	}

	// Check the file length is as expected
	if (fileLength != 0 && fileBeingUploaded.Length() != fileLength)
	{
		uploadError = true;
		GetPlatform().MessageF(ErrorMessage, "Uploaded file size is different (%lu vs. expected %lu bytes)!\n", fileBeingUploaded.Length(), fileLength);
	}

	// Close the file
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
	}

	// Delete the file again if an error has occurred
	if (!filenameBeingProcessed.IsEmpty())
	{
		if (uploadError)
		{
			GetPlatform().GetMassStorage()->Delete(filenameBeingProcessed.c_str());
		}
		else if (fileLastModified != 0)
		{
			// Update the file timestamp if it was specified
			(void)GetPlatform().GetMassStorage()->SetLastModifiedTime(filenameBeingProcessed.c_str(), fileLastModified);
		}
		filenameBeingProcessed.Clear();
	}
}

// End
