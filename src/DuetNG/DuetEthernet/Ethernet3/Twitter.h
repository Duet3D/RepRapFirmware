/*
  Twitter.cpp - Arduino library to Post messages to Twitter using OAuth.
  Copyright (c) NeoCat 2010-2011. All right reserved.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 */

// ver1.2 - Use <Udp.h> to support IDE 0019 or later
// ver1.3 - Support IDE 1.0

#ifndef TWITTER_H
#define TWITTER_H

#include <inttypes.h>
#include <Ethernet3.h>

class Twitter
{
private:
	uint8_t parseStatus;
	int statusCode;
	const char *token;
	EthernetClient client;
public:
	Twitter(const char *user_and_passwd);
	
	bool doPost(const char *msg);
	bool checkStatus(Print *debug = NULL);
	int  wait(Print *debug = NULL);
	int  status(void) { return statusCode; }
};

#endif	//TWITTER_H
