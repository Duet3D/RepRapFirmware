/*
 * UploadingNetworkResponder.cpp
 *
 *  Created on: 3 Feb 2019
 *      Author: David
 */

#include "UploadingNetworkResponder.h"
#include "Socket.h"
#include <Platform/Platform.h>

UploadingNetworkResponder::UploadingNetworkResponder(NetworkResponder *n) noexcept : NetworkResponder(n)
#if HAS_MASS_STORAGE
	, uploadError(false), dummyUpload(false)
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
			(void)MassStorage::Delete(filenameBeingProcessed.GetRef(), ErrorMessageMode::messageAlways);
			filenameBeingProcessed.Clear();
		}
	}
#endif
}

#if HAS_MASS_STORAGE

// Start writing to a new file, returning true if successful
bool UploadingNetworkResponder::StartUpload(const char* folder, const char *fileName, const OpenMode mode, const uint32_t preAllocSize) noexcept
{
	if (fileName[0] == 0 || StringEndsWithIgnoreCase(fileName, ".dummy"))
	{
		filenameBeingProcessed.Clear();
		dummyUpload = true;
	}
	else
	{
		if (!MassStorage::CombineName(filenameBeingProcessed.GetRef(), folder, fileName))
		{
			filenameBeingProcessed.Clear();
			return false;
		}
		if (filenameBeingProcessed.cat(UPLOAD_EXTENSION))
		{
			filenameBeingProcessed.Clear();
			return false;
		}
		FileStore * const file = GetPlatform().OpenFile(folder, filenameBeingProcessed.c_str(), mode, preAllocSize);
		if (file == nullptr)
		{
			filenameBeingProcessed.Clear();
			return false;
		}
		fileBeingUploaded.Set(file);
		dummyUpload = false;
	}
	responderState = ResponderState::uploading;
	uploadError = false;
	return true;
}

// Finish a file upload. Set variable uploadError if anything goes wrong.
void UploadingNetworkResponder::FinishUpload(uint32_t fileLength, time_t fileLastModified, bool gotCrc, uint32_t expectedCrc) noexcept
{
	if (!dummyUpload)
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
			if (uploadError)
			{
#if 0	// Temporary code to save files with upload errors and report successful uploads
				String<MaxFilenameLength> newName;
				MassStorage::CombineName(newName.GetRef(), Platform::GetGCodeDir(), "uploadError_");
				newName.catf("%04u", (unsigned int)(millis() % 10000));
				MassStorage::Rename(filenameBeingProcessed.c_str(), newName.c_str(), true, true);
#else
				(void)MassStorage::Delete(filenameBeingProcessed.GetRef(), ErrorMessageMode::messageUnlessMissing);
#endif
			}
			else
			{
				String<MaxFilenameLength> origFilename;
				origFilename.copy(filenameBeingProcessed.c_str(), filenameBeingProcessed.GetRef().strlen() - strlen(UPLOAD_EXTENSION));

				// Rename the uploaded file to it's original name
				MassStorage::Rename(filenameBeingProcessed.c_str(), origFilename.c_str(), true, true);

				if (fileLastModified != 0)
				{
					// Update the file timestamp if it was specified
					(void)MassStorage::SetLastModifiedTime(origFilename.c_str(), fileLastModified);
				}
#if 0	// Temporary code to save files with upload errors and report successful uploads
				GetPlatform().Message(GenericMessage, "Successful upload\n");
#endif
			}
			filenameBeingProcessed.Clear();
		}
	}
}

#endif

// End
