/*
 * HttpResponder.h
 *
 *  Created on: 14 Apr 2017
 *      Author: David
 */

#ifndef SRC_NETWORKING_HTTPRESPONDER_H_
#define SRC_NETWORKING_HTTPRESPONDER_H_

#include "NetworkResponder.h"

class HttpResponder : public NetworkResponder
{
public:
	HttpResponder(NetworkResponder *n);
	bool Spin() override;								// do some work, returning true if we did anything significant
	bool Accept(Socket *s, NetworkProtocol protocol) override;	// ask the responder to accept this connection, returns true if it did
	void Terminate(NetworkProtocol protocol) override;			// terminate the responder if it is serving the specified protocol
	void Diagnostics(MessageType mtype) const override;

	static void HandleGCodeReply(const char *reply);
	static void HandleGCodeReply(OutputBuffer *reply);
	static uint32_t GetReplySeq() { return seq; }
	static void CheckSessions();
	static void CommonDiagnostics(MessageType mtype);

protected:
	void ConnectionLost() override;
	void CancelUpload() override;
	void SendData() override;

private:
	static const size_t MaxHttpSessions = 8;			// maximum number of simultaneous HTTP sessions
	static const uint16_t WebMessageLength = 1460;		// maximum length of the web message we accept after decoding
	static const size_t MaxCommandWords = 4;			// max number of space-separated words in the command
	static const size_t MaxQualKeys = 5;				// max number of key/value pairs in the qualifier
	static const size_t MaxHeaders = 30;				// max number of key/value pairs in the headers
	static const uint32_t HttpSessionTimeout = 8000;	// HTTP session timeout in milliseconds

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
		const char* key;
		const char* value;
	};

	// HTTP sessions
	struct HttpSession
	{
		uint32_t ip;
		uint32_t lastQueryTime;
		bool isPostUploading;
		uint16_t postPort;
	};

	bool Authenticate();
	bool CheckAuthenticated();
	bool RemoveAuthentication();

	bool CharFromClient(char c);
	void SendFile(const char* nameOfFileToSend, bool isWebFile);
	void SendGCodeReply();
	void SendJsonResponse(const char* command);
	bool GetJsonResponse(const char* request, OutputBuffer *&response, bool& keepOpen);
	void ProcessMessage();
	void RejectMessage(const char* s, unsigned int code = 500);
	bool SendFileInfo();

	void DoUpload();

	const char* GetKeyValue(const char *key) const;	// return the value of the specified key, or nullptr if not present

	HttpParseState parseState;

	// Buffers for processing HTTP input
	char clientMessage[WebMessageLength + 3];		// holds the command, qualifier, and headers
	size_t clientPointer;							// current index into clientMessage
	char decodeChar;								// the character we are decoding in a URL-encoded argument

	// Parser state
	const char* commandWords[MaxCommandWords];
	KeyValueIndices qualifiers[MaxQualKeys + 1];	// offsets into clientQualifier of the key/value pairs, the +1 is needed so that values can contain nulls
	KeyValueIndices headers[MaxHeaders];			// offsets into clientHeader of the key/value pairs
	size_t numCommandWords;
	size_t numQualKeys;								// number of qualifier keys we have found, <= maxQualKeys
	size_t numHeaderKeys;							// number of keys we have found, <= maxHeaders

	// rr_fileinfo requests
	char filenameBeingProcessed[MaxFilenameLength];	// The filename being processed (for rr_fileinfo)

	// Keeping track of HTTP sessions
	static HttpSession sessions[MaxHttpSessions];
	static unsigned int numSessions;
	static unsigned int clientsServed;

	// Responses from GCodes class
	static uint32_t seq;							// Sequence number for G-Code replies
	static OutputStack *gcodeReply;

	static NetworkResponderLock fileInfoLock;		// PrintMonitor::GetFileInfoResponse is single threaded at present, so use this to control access
};

#endif /* SRC_NETWORKING_HTTPRESPONDER_H_ */
