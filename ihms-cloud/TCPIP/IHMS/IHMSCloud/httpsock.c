#include "stdlib.h"
#include "httpsock.h"

TCP_SOCKET hmSocket = INVALID_SOCKET;
#define IHMS_ADDR 0xbd050000

int wait_count = 0;
int recv_done = 0;
int send_count = 0;


// local variables
//  IHMS - Intelligent Health Monitoring System
//  hm - Health Monitoring

// Defines the cloud server to be accessed

//static BYTE ServerName[] = "192.168.33.10";

// Defines the port to be accessed for this application

#if defined(STACK_USE_SSL_CLIENT)
    static WORD ServerPort = HTTPS_PORT;
	// Note that if HTTPS is used, the ServerName and URL
	// must change to an SSL enabled server.
#else
    static WORD ServerPort = HTTP_PORT;
#endif

// local functions

// externs
extern APP_CONFIG AppConfig;

int GenericTCPState = SM_DONE;


// global variables

/*****************************************************************************
*
*  IHMS_ReadUUID
*
*  \param  UUID_buf - buffer to return hexadecimal MAC
*
*  \return 0 if failure; len of UUID if success;
*
*  \brief  Reads the MAC address from the hardware
*
*****************************************************************************/
int
IHMS_ReadUUID(unsigned char * UUID_buf)
{
  int retval = 0;
  MAC_ADDR macbuf;
  unsigned char macstr[20];

      WF_GetMacAddress(macbuf.v);
      sprintf((char *)macstr,"%02X%02X%02X%02X%02X%02X", macbuf.v[0], macbuf.v[1], macbuf.v[2], macbuf.v[3], macbuf.v[4], macbuf.v[5]);
      retval = strlen((char *)macstr);
      memcpy(UUID_buf, macstr, retval);
      UUID_buf[retval] = 0;
  return retval;
}



/*****************************************************************************
*
*  IHMS_SocketClose
*
*  \param  socket - socket handle
*
*  \return None
*
*  \brief  Closes a socket
*
*****************************************************************************/
void
IHMS_SocketClose(long socket)
{
  // Send everything immediately
  if (GenericTCPState == SM_DISCONNECT)
  {
    if (TCPIsConnected((TCP_SOCKET)socket) && recv_done == 1)
    {
      TCPClose((TCP_SOCKET)socket);
    //  TCPClose((TCP_SOCKET)socket);
      recv_done = 0;
      socket = INVALID_SOCKET;
      hmSocket = INVALID_SOCKET;
    }
    send_count = 0;
    GenericTCPState = SM_DONE;
  }
  return;
}


/*****************************************************************************
*
*  IHMS_SocketOpenTCP
*
*  \param  server - server hostname or ip
*
*  \return -1: failure; Other: socket handle
*
*  \brief  Opens a TCP socket
*
*****************************************************************************/
long
IHMS_SocketOpenTCP(void)
{

  DWORD serverip = 0;
  const unsigned char server[6] = {192,168,33,68};


  serverip = (server[3] << 24 & 0xff000000) | (server[2] << 16 & 0xff0000)
          | (server[1] << 8 & 0xff00) | (server[0] & 0xff);

  if (GenericTCPState == SM_DONE)
    GenericTCPState = SM_HOME;

  // Start the TCP server, listening on RX_PERFORMANCE_PORT
  if (GenericTCPState == SM_HOME)
  {
    if (hmSocket == INVALID_SOCKET)
    {
      hmSocket = TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, ServerPort, TCP_PURPOSE_TCP_CLIENT);

      // TCP_OPEN_RAM_HOST for using dns name as server name
     //  TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, server_port, TCP_PURPOSE_GENERIC_TCP_CLIENT);

      if (hmSocket == INVALID_SOCKET)
      {
        return -1;
      }
    }
    GenericTCPState = SM_SOCKET_OBTAINED;
  }

  return (long)hmSocket;
}

/*****************************************************************************
*
*  IHMS_ServerConnect
*
*  \param  None
*
*  \return socket - socket handle
*
*  \brief  The function opens a TCP socket
*
*****************************************************************************/
long
IHMS_ServerConnect(long sock)
{
  if (GenericTCPState == SM_SOCKET_OBTAINED)
  {
    if (TCPWasReset((TCP_SOCKET)sock))
    {
      recv_done = 1;
      GenericTCPState = SM_DISCONNECT;
      wait_count = 0;
      return -1;
    }
    if (TCPIsConnected((TCP_SOCKET)sock))
    {
      GenericTCPState = SM_PACKAGE_SEND;
    }
    else
    {
      wait_count++;
      if (wait_count > 10)
      {
        recv_done = 1;
        GenericTCPState = SM_DISCONNECT;
        wait_count = 0;
        return -1;
      }
    }
  }
  return (long)sock;
}

/*****************************************************************************
*
*  IHMSL_SocketSend
*
*  \param  socket - socket handle; buffer - string buffer containing info to
*          send; len - size of string in bytes;
*
*  \return Number of bytes sent
*
*  \brief  Sends data out to the internet
*
*****************************************************************************/
unsigned char
IHMS_SocketSend(long socket, char * buffer, unsigned char len)
{
  int send_len = 0;
  int sent_len = 0;


  int w = TCPIsPutReady((TCP_SOCKET)socket);

   if(w>0 && len>0)
     {
        send_len = w<len? w:len;

        send_len = send_len<=250? send_len:250;

        send_len = TCPPutArray((TCP_SOCKET)socket, (BYTE *)buffer, send_len);

        len -= send_len;

        TCPFlush((TCP_SOCKET)socket);
  
        sent_len +=send_len;

     }
  
  return sent_len;
}


/*****************************************************************************
*
*  IHMS_SocketRecv
*
*  \param  socket - socket handle; buffer - string buffer to put info we
*          receive; len - size of buffer in bytes;
*
*  \return Number of bytes received
*
*  \brief  Receives data from the internet
*
*****************************************************************************/
unsigned char
IHMS_SocketRecv(long socket, char * buffer, unsigned char len)
{
  WORD w, wGetLen;

//  if (GenericTCPState == SM_PACKAGE_SEND && send_count >= 4)
//  {
//    GenericTCPState++;
//  }

  //if (GenericTCPState == SM_PROCESS_RESPONSE)
 // {

    w = TCPIsGetReady((TCP_SOCKET)socket);
    buffer[0] = 0;
    if (w)
    {
      wGetLen = w;
      TCPGetArray((TCP_SOCKET)socket, (BYTE *)buffer, len);
      if (w < len)
      {
   //     GenericTCPState = SM_DISCONNECT;
        recv_done =1;
        send_count = 0;
      }

      return len;
    }
//  }

  return 0;
}



/*****************************************************************************
*
*  IHMS_itoa
*
*  \param  char *str - convert string
*          int value - the converted value
*          int base - define the convert type
*
*  \return None
*
*  \brief  Delays for specified milliseconds
*
*****************************************************************************/
void
IHMS_itoa(char *str, int value, int base)
{
  if (base == 10)
    uitoa(value, (BYTE *)str);

  return;
}


