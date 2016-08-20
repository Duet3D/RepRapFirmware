/*
 *  sha1.h
 *
 *  Copyright (C) 1998, 2009
 *  Paul E. Jones <paulej@packetizer.com>
 *  All Rights Reserved
 *
 *****************************************************************************
 *  $Id: sha1.h 12 2009-06-22 19:34:25Z paulej $
 *****************************************************************************
 *
 *  Description:
 *      This class implements the Secure Hashing Standard as defined
 *      in FIPS PUB 180-1 published April 17, 1995.
 *
 *      Many of the variable names in the SHA1Context, especially the
 *      single character names, were used because those were the names
 *      used in the publication.
 *
 *      Please read the file sha1.c for more information.
 *
 *	Updated to use stdint.h and be CPP compatible.
 */

#ifndef _SHA1_H_
#define _SHA1_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 
 *  This structure will hold context information for the hashing
 *  operation
 */
typedef struct SHA1Context
{
    uint32_t Message_Digest[5]; /* Message Digest (output)          */

    uint32_t Length_Low;        /* Message length in bits           */
    uint32_t Length_High;       /* Message length in bits           */

    uint8_t Message_Block[64]; /* 512-bit message blocks      */
    int32_t Message_Block_Index;    /* Index into message block array   */

    bool Computed;               /* Is the digest computed?          */
    bool Corrupted;              /* Is the message digest corruped?  */
} SHA1Context;

/*
 *  Function Prototypes
 */
void SHA1Reset(SHA1Context* context);
bool SHA1Result(SHA1Context* context);
void SHA1Input( SHA1Context* context,
                const uint8_t* message_array,
                uint32_t length);

#ifdef __cplusplus
}
#endif
#endif
