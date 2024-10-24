/*
 * HttpResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_HTTPRESPONDER_H_
#define SRC_NETWORKING_HTTPRESPONDER_H_

#include "UploadingNetworkResponder.h"

typedef unsigned int HttpSessionKey;

class HttpResponder : public UploadingNetworkResponder
{
public:
	explicit HttpResponder(NetworkResponder *_ecv_from _ecv_null n) noexcept;
	bool Spin() noexcept override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) noexcept override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol, const NetworkInterface *_ecv_from interface) noexcept override;	// terminate the responder if it is serving the specified protocol on the specified interface
	void Diagnostics(MessageType mtype) const noexcept override;

	static void InitStatic() noexcept;
	static void Disable() noexcept;
	static void DisableInterface(const NetworkInterface *_ecv_from iface) noexcept;
	static void HandleGCodeReply(const char *_ecv_array reply) noexcept;
	static void HandleGCodeReply(OutputBuffer *_ecv_null reply) noexcept;
	static uint16_t GetReplySeq() noexcept { return seq; }
	static void CheckSessions() noexcept;
	static void CommonDiagnostics(MessageType mtype) noexcept;

protected:
	void CancelUpload() noexcept override;
	void SendData() noexcept override;

private:
	static const size_t MaxHttpSessions = 8;			// maximum number of simultaneous HTTP sessions
	static const uint16_t WebMessageLength = 1460;		// maximum length of the web message we accept after decoding
	static const size_t MaxCommandWords = 4;			// max number of space-separated words in the command
	static const size_t MaxQualKeys = 5;				// max number of key/value pairs in the qualifier
	static const size_t MaxHeaders = 30;				// max number of key/value pairs in the headers
	static const uint32_t HttpSessionTimeout = 8000;	// HTTP session timeout in milliseconds
	static const uint32_t MaxFileInfoGetTime = 2000;	// maximum length of time we spend getting file info, to avoid the client timing out (actual time will be a little longer than this)
	static const uint32_t MaxBufferWaitTime = 1000;		// maximum length of time we spend waiting for a buffer before we discard gcodeReply buffers

	enum class HttpParseState
	{
		doingCommandWord,			// receiving a word in the first line of the HTTP request
		doingFilename,				// receiving the filename (second word in the command line)
		doingFilenameEsc1,			// received '%' in the filename (e.g. we are being asked for a filename with spaces in it)
		doingFilenameEsc2,			// received '%' and one hex digit in the filename
		doingQualifierKey,			// receiving a key name in the HTTP request
		doingQualifierValue,		// receiving a key value in the HTTP request
		doingQualifierValueEsc1,	// received '%' in the qualifier
		doingQualifierValueEsc2,	// received '%' and one hex digit in the qualifier
		doingHeaderKey,				// receiving a header key
		expectingHeaderValue,		// expecting a header value
		doingHeaderValue,			// receiving a header value
		doingHeaderContinuation		// received a newline after a header value
	};

	struct KeyValueIndices
	{
		const char *_ecv_array key;
		const char *_ecv_array value;
	};

	// HTTP sessions
	struct HttpSession
	{
		HttpSessionKey key;
		IPAddress ip;
		const NetworkInterface *_ecv_from iface;
		uint32_t lastQueryTime;
		uint16_t postPort;
		bool isPostUploading;
	};

	bool Authenticate(bool withSessionKey, HttpSessionKey &sessionKey) noexcept;
	bool CheckAuthenticated() noexcept;
	bool RemoveAuthentication() noexcept;

	bool CharFromClient(char c) noexcept;
	void SendFile(const char *_ecv_array nameOfFileToSend, bool isWebFile) noexcept;
	void SendGCodeReply() noexcept;
	void SendJsonResponse(const char *_ecv_array command) noexcept;
	bool GetJsonResponse(const char *_ecv_array request, OutputBuffer *_ecv_null &response, bool& keepOpen) noexcept;
	void ProcessMessage() noexcept;
	void ProcessRequest() noexcept;
	void RejectMessage(const char *_ecv_array s, unsigned int code = 500) noexcept;
	bool SendFileInfo(bool quitEarly) noexcept;
	void AddCorsHeader() noexcept;

#if HAS_MASS_STORAGE
	void DoUpload() noexcept;
#endif

	const char *_ecv_array null GetKeyValue(const char *_ecv_array key) const noexcept;		// return the value of the specified key, or nullptr if not present
	const char *_ecv_array null GetHeaderValue(const char *_ecv_array key) const noexcept;	// return the value of the specified header, or nullptr if not present
	HttpSessionKey GetSessionKey() const noexcept;	// try to get the optional X-Session-Key header value used to identify HTTP sessions

	static void RemoveSession(size_t sessionToRemove) noexcept;

	HttpParseState parseState;

	// Buffers for processing HTTP input
	char clientMessage[WebMessageLength + 3];		// holds the command, qualifier, and headers
	size_t clientPointer;							// current index into clientMessage
	char decodeChar;								// the character we are decoding in a URL-encoded argument

	// Parser state
	const char *_ecv_array _ecv_null commandWords[MaxCommandWords];
	KeyValueIndices qualifiers[MaxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
	KeyValueIndices headers[MaxHeaders];			// offsets into clientHeader of the key/value pairs
	size_t numCommandWords;
	size_t numQualKeys;								// number of qualifier keys we have found, <= maxQualKeys
	size_t numHeaderKeys;							// number of keys we have found, <= maxHeaders

	// rr_fileinfo requests
	uint32_t startedProcessingRequestAt;			// when we started processing the current HTTP request
	// rr_fileinfo also uses fileBeingProcessed in the networkResponder class

	uint32_t postFileLength;
	uint32_t postFileExpectedCrc;
	time_t fileLastModified;
	bool postFileGotCrc;

	// Keeping track of HTTP sessions
	static HttpSession sessions[MaxHttpSessions];
	static unsigned int numSessions;
	static unsigned int clientsServed;

	// Responses from GCodes class
	static volatile uint16_t seq;					// Sequence number for G-Code replies
	static volatile OutputStack gcodeReply;
	static Mutex gcodeReplyMutex;
};

#endif /* SRC_NETWORKING_HTTPRESPONDER_H_ */
