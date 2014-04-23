/* 
 * File:   ihms_hw.h
 *
 * Created on November 29, 2013, 12:14 PM
 */

#ifndef IHMS_HW_H
#define	IHMS_HW_H

#include "TCPIPConfig.h"
#include "TCPIP Stack/TCPIP.h"
#include "TCPIP Stack/Helpers.h"



// defines
#define META_SIZE                 256
#define META_CIK_SIZE             40
#define META_SERVER_SIZE          6
#define META_PAD0_SIZE            2
#define META_MARK_SIZE            8
#define META_UUID_SIZE            12
#define META_PAD1_SIZE            4
#define META_RSVD_SIZE            48    // TODO - flash block size is 128 - either make MFR these 48 RSVD bytes or instrument flash routine to use next block for MFR
#define META_MFR_SIZE             128 + META_RSVD_SIZE

typedef struct {
    char cik[META_CIK_SIZE];                   // our client interface key
    char server[META_SERVER_SIZE];             // ip address of cloud server (not using DNS at this stage)
    char pad0[META_PAD0_SIZE];                 // pad 'server' to 8 bytes
    char mark[META_MARK_SIZE];                 // watermark
    char uuid[META_UUID_SIZE];                 // UUID in ascii
    char pad1[META_PAD1_SIZE];                 // pad 'uuid' to 16 bytes
    char mfr[META_MFR_SIZE];                   // manufacturer data structure
} cloud_meta;


#define WATERMARK "ihms"

typedef enum
{
    META_CIK,
    META_SERVER,
    META_MARK,
    META_UUID,
    META_MFR,
    META_NONE
} MetaElements;



static enum _GenericTCPState
{
  SM_HOME = 0,
  SM_SOCKET_OBTAINED,
  SM_PACKAGE_SEND,
  SM_PROCESS_RESPONSE,
  SM_DISCONNECT,
  SM_DONE
};


int  IHMS_ReadUUID(unsigned char * UUID_buf);
void IHMS_EraseMeta(void);
void IHMS_WriteMetaItem(unsigned char * buffer, unsigned short len, unsigned char element);
void IHMS_ReadMetaItem(unsigned char * buffer, unsigned short destBytes, unsigned char element);
void IHMS_SocketClose(long socket);
long IHMS_SocketOpenTCP(void);
long IHMS_ServerConnect(long socket);
unsigned char IHMS_SocketSend(long socket, char * buffer, unsigned char len);
unsigned char IHMS_SocketRecv(long socket, char * buffer, unsigned char len);
void IHMS_MSDelay(unsigned short delay);
void IHMS_itoa(char *str, int value, int base);

#endif	/* IHMS_HW_H */

