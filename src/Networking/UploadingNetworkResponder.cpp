/*
 * UploadingNetworkResponder.cpp
 *
 *  Created on: 3 Feb 2019
 *      Author: David
 */

#include "UploadingNetworkResponder.h"
#include "Socket.h"
#include "Platform.h"

UploadingNetworkResponder::UploadingNetworkResponder(NetworkResponder *n) noexcept : NetworkResponder(n)
#if HAS_MASS_STORAGE
	, uploadError(false)
#endif
{
}

// This is called when we lose a connection or when we are asked to terminate. Overridden in some derived classes.
void UploadingNetworkResponder::ConnectionLost() noexcept
{
	CancelUpload();
	NetworkResponder::ConnectionLost();
}

// If this responder has an upload in progress, cancel it
void UploadingNetworkResponder::CancelUpload() noexcept
{
#if HAS_MASS_STORAGE
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
		if (!filenameBeingProcessed.IsEmpty())
		{
			MassStorage::Delete(filenameBeingProcessed.c_str(), true);
			filenameBeingProcessed.Clear();
		}
	}
#endif
}

#if HAS_MASS_STORAGE

// Start writing to a new file
FileStore * UploadingNetworkResponder::StartUpload(const char* folder, const char *fileName, const OpenMode mode, const uint32_t preAllocSize) noexcept
{
	if (!MassStorage::CombineName(filenameBeingProcessed.GetRef(), folder, fileName))
	{
		filenameBeingProcessed.Clear();
		return nullptr;
	}
	if (filenameBeingProcessed.cat(UPLOAD_EXTENSION))
	{
		filenameBeingProcessed.Clear();
		return nullptr;
	}
	FileStore * file = GetPlatform().OpenFile(folder, filenameBeingProcessed.c_str(), mode, preAllocSize);
	if (file == nullptr)
	{
		filenameBeingProcessed.Clear();
		return file;
	}
	fileBeingUploaded.Set(file);
	responderState = ResponderState::uploading;
	uploadError = false;
	return file;
}

// Finish a file upload. Set variable uploadError if anything goes wrong.
void UploadingNetworkResponder::FinishUpload(uint32_t fileLength, time_t fileLastModified, bool gotCrc, uint32_t expectedCrc) noexcept
{
	// Flush remaining data for FSO
	if (!fileBeingUploaded.Flush())
	{
		uploadError = true;
		GetPlatform().Message(ErrorMessage, "Could not flush remaining data while finishing upload\n");
	}

	// Check the file length is as expected
	if (fileLength != 0 && fileBeingUploaded.Length() != fileLength)
	{
		uploadError = true;
		GetPlatform().MessageF(ErrorMessage, "Uploaded file size is different (%lu vs. expected %lu bytes)\n", fileBeingUploaded.Length(), fileLength);
	}
	else if (gotCrc && expectedCrc != fileBeingUploaded.GetCrc32())
	{
		uploadError = true;
		GetPlatform().MessageF(ErrorMessage, "Uploaded file CRC is different (%08" PRIx32 " vs. expected %08" PRIx32 ")\n", fileBeingUploaded.GetCrc32(), expectedCrc);
	}

	// Close the file
	if (fileBeingUploaded.IsLive())
	{
		fileBeingUploaded.Close();
	}

	// Delete the file again if an error has occurred
	if (!filenameBeingProcessed.IsEmpty())
	{
		const char *uploadFilename = filenameBeingProcessed.c_str();
		if (uploadError)
		{
			MassStorage::Delete(uploadFilename, true);
		}
		else
		{
			String<MaxFilenameLength> origFilename;
			origFilename.catn(uploadFilename, filenameBeingProcessed.GetRef().strlen() - strlen(UPLOAD_EXTENSION));

			// Delete possibly existing files with that name (i.e. prepare "overwrite")
			MassStorage::Delete(origFilename.c_str(), true);

			// Rename the uploaded file to it's original name
			MassStorage::Rename(uploadFilename, origFilename.c_str(), true);

			if (fileLastModified != 0)
			{
				// Update the file timestamp if it was specified
				(void)MassStorage::SetLastModifiedTime(origFilename.c_str(), fileLastModified);
			}
		}
		filenameBeingProcessed.Clear();
	}
}

#endif

// End
