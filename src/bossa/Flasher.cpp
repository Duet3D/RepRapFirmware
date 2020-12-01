///////////////////////////////////////////////////////////////////////////////
// BOSSA
//
// Copyright (c) 2011-2018, ShumaTech
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////

#include "Flasher.h"
#include "RepRap.h"
#include "Platform.h"
#include "General/Vector.hpp"
#if HAS_LINUX_INTERFACE
# include "Linux/LinuxInterface.h"
#endif

void
Flasher::erase(uint32_t foffset) THROWS(GCodeException)
{
#if ORIGINAL_BOSSA_CODE
    _observer.onStatus("Erase flash\n");
#endif
    _flash->eraseAll(foffset);
    _flash->eraseAuto(false);
}

int Flasher::GetNextChunk(char* buffer, const uint32_t length, uint32_t& offset, const char* filename) noexcept
{
	uint32_t read = 0;
#if HAS_LINUX_INTERFACE
	if (!reprap.UsingLinuxInterface())
#endif
	{
		read = infile->Read(buffer, length);
	}
#if HAS_LINUX_INTERFACE
	else
	{
		read = length;
		const bool success = reprap.GetLinuxInterface().GetFileChunk(filename, offset, (char*)buffer, read, fileSize);
		if (!success) {
			return -1;
		}
	}
#endif
	offset += read;
	return read;
}

bool
Flasher::write(const char* filename, uint32_t& foffset) THROWS(GCodeException)
{
    uint32_t pageSize = _flash->pageSize();
    uint32_t numPages;
    size_t fbytes = 0;

    if (foffset == 0)
    {
    	pageNum = 0;
    	Platform& platform = reprap.GetPlatform();
#if HAS_LINUX_INTERFACE
    	if (!reprap.UsingLinuxInterface())
#endif
    	{
    		infile = MassStorage::OpenFile(filename, OpenMode::read, 0);
    		if (infile == nullptr)
    		{
    			platform.MessageF(ErrorMessage, "Failed to open file %s\n", filename);
    			throw GCodeException(nullptr);
    		}
    		fileSize = infile->Length();
    	}
#if HAS_LINUX_INTERFACE
    	else
    	{
    		char dummyBuf[1];
    		uint32_t dummyLen = 0;
    		if (!reprap.GetLinuxInterface().GetFileChunk(filename, 0, dummyBuf, dummyLen, fileSize))
    		{
    			platform.MessageF(ErrorMessage, "Failed to open file %s\n", filename);
    			throw GCodeException(nullptr);
    		}
    	}
#endif
		if (fileSize == 0)
		{
	    	if (infile != nullptr)
			{
	        	infile->Close();
			}
			platform.MessageF(ErrorMessage, "Firmware file is empty %s\n", filename);
			throw GCodeException(nullptr);
		}
    }

    try
    {

        numPages = (fileSize + pageSize - 1) / pageSize;
        if (foffset == 0)
        {
            if (numPages > _flash->numPages())
            {
                throw FileSizeError("Flasher::write: FileSizeError");
            }

            _observer.onStatus("Writing %ld bytes to PanelDue flash memory\n", fileSize);
        }

		uint8_t buffer[pageSize];
		uint32_t pageOffset = foffset / pageSize;

		if ((fbytes = GetNextChunk((char*)buffer, pageSize, foffset, filename)) > 0)
		{
			_observer.onProgress(pageNum, numPages);

			_flash->loadBuffer(buffer, fbytes);
			_flash->writePage(pageOffset /* + pageNum */);

			pageNum++;
			if (!(pageNum == numPages || fbytes != pageSize))
			{
				return false;				// We get back in the next PanelDueFlasher::Spin() iteration
			}
		}
    }
    catch(...)
    {
    	if (infile != nullptr)
		{
        	infile->Close();
		}

        throw;
    }

	if (infile != nullptr)
	{
		infile->Close();
	}
    _observer.onProgress(numPages, numPages);
    return true;
}

bool
Flasher::verify(const char* filename, uint32_t& pageErrors, uint32_t& totalErrors, uint32_t& foffset) THROWS(GCodeException)
{
    uint32_t pageSize = _flash->pageSize();
    uint8_t bufferA[pageSize];
    uint8_t bufferB[pageSize];
    uint32_t numPages;
    uint32_t pageOffset;
    uint32_t byteErrors = 0;
    size_t fbytes;

    pageErrors = 0;
    totalErrors = 0;


    pageOffset = foffset / pageSize;

    if (foffset == 0)
    {
    	pageNum = 0;

    	// Note that we can skip all checks for file validity here since
    	// we would not get here from write() if any of these failed

#if HAS_LINUX_INTERFACE
    	if (!reprap.UsingLinuxInterface())
#endif
    	{
    		infile = MassStorage::OpenFile(filename, OpenMode::read, 0);
    	}
    }

    try
    {

        numPages = (fileSize + pageSize - 1) / pageSize;
        if (foffset == 0)
        {
            if (numPages > _flash->numPages())
                throw FileSizeError("Flasher::verify: FileSizeError");

            _observer.onStatus("Verifying %ld bytes of PanelDue flash memory\n", fileSize);
        }

        if ((fbytes = GetNextChunk((char*)bufferA, pageSize, foffset, filename)) > 0)
        {
            byteErrors = 0;

            _observer.onProgress(pageNum, numPages);

            {
                _flash->readPage(pageOffset /*+ pageNum */, bufferB);

                for (uint32_t i = 0; i < fbytes; i++)
                {
                    if (bufferA[i] != bufferB[i])
                        byteErrors++;
                }
            }

            if (byteErrors != 0)
            {
                pageErrors++;
                totalErrors += byteErrors;
            }

            pageNum++;
            if (!(pageNum == numPages || fbytes != pageSize))
            {
                return false;				// We get back in the next PanelDueFlasher::Spin() iteration
            }
        }
    }
    catch(...)
    {
    	if (infile != nullptr)
    	{
            infile->Close();
    	}
        throw;
    }

	if (infile != nullptr)
	{
        infile->Close();
	}

     _observer.onProgress(numPages, numPages);

#if ORIGINAL_BOSSA_CODE // This has to be checked outside because we use the return value now used to indicate if we finished
    if (pageErrors != 0)
        return false;
#endif

    return true;
}

void
Flasher::lock(/* string& regionArg, */ bool enable) THROWS(GCodeException)
{
#if ORIGINAL_BOSSA_CODE
    if (regionArg.empty())
    {
        _observer.onStatus("%s all regions\n", enable ? "Lock" : "Unlock");
#endif
        Vector<bool, 16> regions(_flash->lockRegions(), enable);
        _flash->setLockRegions(regions);
#if ORIGINAL_BOSSA_CODE
    }
    else
    {
        size_t pos = 0;
        size_t delim;
        uint32_t region;
        string sub;
        std::vector<bool> regions = _flash->getLockRegions();

        do
        {
            delim = regionArg.find(',', pos);
            sub = regionArg.substr(pos, delim - pos);
            region = strtol(sub.c_str(), NULL, 0);
            _observer.onStatus("%s region %d\n", enable ? "Lock" : "Unlock", region);
            regions[region] = enable;
            pos = delim + 1;
        } while (delim != string::npos);

        _flash->setLockRegions(regions);
    }
#endif
}
