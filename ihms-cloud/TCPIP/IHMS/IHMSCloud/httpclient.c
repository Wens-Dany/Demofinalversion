/*****************************************************************************
*
*  httpclient.c - cloud communications.
*
*****************************************************************************/
#include "Main.h"
#include "httpsock.h"
#include "httpclient.h"
#include "TCPIP Stack/Tick.h"
#include <string.h>

//local defines

#define META_SIZE                 256
#define META_CIK_SIZE             40
#define META_SERVER_SIZE          6
#define META_PAD0_SIZE            2
#define META_MARK_SIZE            8
#define META_UUID_SIZE            12
#define META_PAD1_SIZE            4
#define META_RSVD_SIZE            48          // TODO - flash block size is 128 - either make MFR these 48 RSVD bytes or instrument flash routine to use next block for MFR
#define META_MFR_SIZE             128 + META_RSVD_SIZE

enum lineTypes
{
  CIK_LINE,
  HOST_LINE,
  CONTENT_LINE,
  ACCEPT_LINE,
  LENGTH_LINE,
  GETDATA_LINE,
  POSTDATA_LINE,
  EMPTY_LINE
};

enum {
    CLOUD_HOME,
    CLOUD_READING,
    CLOUD_WRITING,
    CLOUD_BUTTON_STATUS_UPDATE,
    CLOUD_CUSTOMIZE,
    CLOUD_END
};

#define STR_CIK_HEADER "CIK: "
#define STR_CONTENT_LENGTH "Content-Length: "
#define STR_GET_URL "GET /ping.php?"
#define STR_HTTP "  HTTP/1.1\r\n"
#define STR_HOST "Host: cloud.ihms.com\r\n"
#define STR_ACCEPT "Accept: application/x-www-form-urlencoded; charset=utf-8\r\n"
#define STR_CONTENT "Content-Type: application/x-www-form-urlencoded; charset=utf-8\r\n"
#define STR_VENDOR "vendor="
#define STR_MODEL "model="
#define STR_SN "sn="
#define STR_CRLF "\r\n"


enum StatusCodes
{
    STATUS_INIT,
    STATUS_READY,
    STATUS_RCV,
    STATUS_END
};

int ping = 0;  // global ping value
int button_state = 0;  // monitor button status
//int nvmram_verify = 0;  // before you write data into internal flash, you should erase flash
int once_init = 0;  // confirm firmware once
int setup_wifi = 1;  // recover the WiFi setting after default WiFi ready
int led2_delay_count = 0; // global value to count the number of led blinks
DWORD cloudstatus_time_tick = 0; // global value to record the cloud status indicate time tick
DWORD demo_delay_time_tick = 0; // global value to record the demo task time tick.
DWORD led2_time_tick = 0;  // global value to record the LED2 time tick
DWORD network_err_time_tick = 0;
int latest_hm_code = STATUS_END; // records Exosite latest status
int delay_count = 0; // global value to record the demo delay count
int cloud_status = 0; // helps for swicth the next cloud status
int once_update = 1;  // overwrite push button status on cloud on push button release
int badcik = 1; // CIK is invalid
int wifi_fail_count = 0; // counts the WiFi fail times
int tcp_fail_count = 0; // counts the TCP fail times
int rev_time_wait = 0;  // counts the HTTP response times
char header[260];   // for send HTTP HEADER and POST content in one packet

BOOL tx_buffer_clear=TRUE;

int remain_count = 0;

extern BOOL send_data;
//char data[15100];

static TCP_SOCKET	sock = INVALID_SOCKET;

int workon_demo = 0;

// local functions
int info_assemble(const char * vendor, const char *model, const char *sn);
int init_UUID(unsigned char if_nbr);
int get_http_status(long socket);
//long connect_to_cloud();
void sendLine(long socket, unsigned char LINE, const char * payload);

void addLine(char * httpheader, unsigned char LINE, const char * payload);

// global functions
int Cloud_Post();
int Cloud_Read(char * palias, char * pbuf, unsigned char buflen);
int Cloud_Init(const char *vendor, const char *model, int reset);
int Cloud_Activate(void);
//void IHMS_SetCIK(char * pCIK);
//int IHMS_GetCIK(char * pCIK);

//local defines
int unknown_status = 0;

// local functions
int button_monitor(void);
int read_command(void);
int heart_beat_report(void);
int update_dev_ip(void);
void status_led1_handler(int count);
int status_led2_handler(int count, int delay);
int show_cloud_status(void);
int wifi_fw_detect(void);
int task_delay(int delay_time, int count);
void check_cloud_activated(void);

// externs

// global variables
static int status_code =  STATUS_END;
static int cloud_initialized = 0;
int http_status = 0;

int sent_count = 0;
BOOL  sent_header = TRUE;

char * datapoint;

/*****************************************************************************
*
* header_assemble
*
*  \param  char * total-header,  one line of header content
*
*
*  \return string length of total-header
*
*  \brief  assemble all HTTP header content for sending them at same time
*
*****************************************************************************/
int
header_assember(char * httpheader, const char *line)
{
  int line_len = 0;
  int header_len = 0;
  int len = 0;


  // verify the header length not overflow
  header_len = strlen(httpheader);
  line_len = strlen(line);
  len = header_len + line_len;
 // if (len > HEADER_LENGTH)
 //   return -1;

  httpheader = httpheader + header_len;
  memcpy(httpheader, line, line_len);

  httpheader = httpheader+line_len;
  *httpheader = 0;
  return len;
}

int PostHeaderGenerate(char * cgiprogram, int cmd, int start, int end )
{
  int length;
  char DataLen[12];
  MAC_ADDR macbuf;
  unsigned char macstr[20];
  header[0] = 0;

  // The initial data length is
  length = 3 + 4 +  6  +  5  +  5  +  4 + 12;        //    id= + cmd= + start= + stop= + data= + 4*&  +  6 bytes MAC
  sprintf(DataLen, "%d", RESULT);
  length +=strlen(DataLen);

  sprintf(DataLen, "%d", start);
  length +=strlen(DataLen);
  
  sprintf(DataLen, "%d", end);
  length +=strlen(DataLen);

  sprintf(DataLen, "%10d", length);

  memset(header, 0, 250);  // 250 should be enough for HTTP header except POST data part
  addLine(header, POSTDATA_LINE, cgiprogram);
  addLine(header, HOST_LINE, NULL);
  addLine(header, CONTENT_LINE, NULL);
  header_assember(header, "Connection: close\r\n");
  addLine(header, LENGTH_LINE, DataLen);   // Attn: the content-length number must be exactly equal to the size of POST data,
                                           //otherwise we cannot get the whole data on HTTP server side, or make server be waiting statement of receiving data
  // The first parameter device id

  header_assember(header, "id=");

  WF_GetMacAddress(macbuf.v);
  sprintf((char *)macstr,"%02X%02X%02X%02X%02X%02X&", macbuf.v[0], macbuf.v[1], macbuf.v[2], macbuf.v[3], macbuf.v[4], macbuf.v[5]);
  header_assember(header, macstr);

  // cmd for identifying the data usage

  header_assember(header, "cmd");
  sprintf(DataLen, "=%d&", RESULT);
  header_assember(header, DataLen);

  // starting time for collecting data
  header_assember(header, "start");
  sprintf(DataLen, "=%d&", start);
  header_assember(header, DataLen);

  // ending time for collecting data
  header_assember(header, "stop");
  sprintf(DataLen, "=%d&", end);
  header_assember(header, DataLen);

  // The last parameter in POST is collected data

  header_assember(header, "data=");

  return length;
}
/*  datatype: 1-char,   2 - hex
 *  Have tested the hex data collected from motion sensor
 */
int PostHeaderUpdateLength(int old_len, int datasize, int datatype)
{
    char * pos;
    // first of all, extract current content-length
    // record the current header length, and record the lenght after adding a data pair
    // then the new content-length = current content-length +(new length - old length)
    char DataTmp[11];
    int length = 0;

    if(datatype==2)
    {
           length = datasize*2;
      }
    else if(datatype==1)
    {
          length = datasize;
    }

    old_len +=length;

  // now locate the position of content-length
  pos= strstr(header, STR_CONTENT_LENGTH);
  if(pos)
  {
      pos += strlen(STR_CONTENT_LENGTH);
      sprintf(DataTmp,"%10d", old_len);

      memcpy(pos, DataTmp, strlen(DataTmp));

  }
  return old_len;
}

/* adding the sensor data into HTTP POST data buffer for sending them on the fly
 *
 * DataType:  Hex   BYTE
 */
void AddPostData(int count, int size)
{
  char DataTmp[3];
  int i;
  char * point = header;
  
  for(i=0; i<size; i++)
    {
         sprintf(DataTmp,"%02x", motionsensor[count]);

         memcpy(point, DataTmp, strlen(DataTmp));
 
       //  header_assember(header, DataTmp);
         count++;
         point +=strlen(DataTmp);
    }
  *point = 0;
  return;
}
/********************************************
 * current HTTP response page format:
HTTP/1.1 200 OK
Date: Thu, 23 Jan 2014 07:52:06 GMT
Server: Apache/2.2.15 (CentOS)
X-Powered-By: PHP/5.3.3
Content-Length: 18
Connection: close
Content-Type: text/html; charset=UTF-8

key=100&cmd=300
 *********************************************
 * the length of the string is  210
 *
 *
 *
 */

int GetServerCmd(char *cmdstr, char *key)
{
   char * pos1;
   char * pos2;
   int value = 0;
   char tmp[2];
   if(strlen(cmdstr)<200) { LEDS_ON();
           return 0;}   // the response page might be incomplete

   pos1= strstr(cmdstr, key);
  
   if(pos1>0)
   {
       pos1+=strlen(key);
       pos2 = strstr(pos1, "&");

       if(pos2>0)
       {
           int i=1;
           while(*pos2!=61)
           {
              // LEDS_ON(); while(1);
               pos2--;
               tmp[0]= *pos2;
               tmp[1]=0;
               value +=atoi(tmp)*i;

               i=i*10;
           //    if(i>1000)  { LEDS_ON(); while(1);}

      
           }

           return value;
           
       }
       else             // the data pair is the last one
       {
           value = atoi(pos1);
           return  value;     
       }
   }
   return 0;
}


/*****************************************************************************
*
*  addLine
*
*  \param  Which line type
*
*  \return socket handle
*
*  \brief  Sends data out
*
*****************************************************************************/
void
addLine(char * httpheader, unsigned char LINE, const char * payload)
{
  char strBuf[70];
  unsigned char strLen;

  switch(LINE) {
    case CIK_LINE:
      strLen = strlen(STR_CIK_HEADER);
      memcpy(strBuf,STR_CIK_HEADER,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += strlen(STR_CRLF);
      break;
    case HOST_LINE:
      strLen = strlen(STR_HOST);
      memcpy(strBuf,STR_HOST,strLen);
      break;
    case CONTENT_LINE:
      strLen = strlen(STR_CONTENT);
      memcpy(strBuf,STR_CONTENT,strLen);
      break;
    case ACCEPT_LINE:
      strLen = strlen(STR_ACCEPT);
      memcpy(strBuf,STR_ACCEPT,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      break;
    case LENGTH_LINE: // Content-Length: NN
      strLen = strlen(STR_CONTENT_LENGTH);
      memcpy(strBuf,STR_CONTENT_LENGTH,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      break;
    case GETDATA_LINE:
      strLen = strlen(STR_GET_URL);
      memcpy(strBuf,STR_GET_URL,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, strlen(STR_HTTP));
      strLen += strlen(STR_HTTP);
      break;
    case POSTDATA_LINE:
      strLen = strlen("POST ");
      memcpy(strBuf,"POST ", strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, strlen(STR_HTTP));
      strLen += strlen(STR_HTTP);
      break;
    case EMPTY_LINE:
      strLen = strlen(STR_CRLF);
      memcpy(strBuf,STR_CRLF,strLen);
      break;
    default:
      break;
  }

  strBuf[strLen] = 0;
  header_assember(httpheader, strBuf);
  return;

}



/*****************************************************************************
*
*  check_network_connected
*
*  \param  None
*
*  \return  TRUE if network ready, FALSE if not
*
*  \brief  Checks to see that network is still connected.
*
*****************************************************************************/
unsigned char
check_network_connected(void)
{
  int network_connected  = 0;

  if (WFisConnected() != TRUE)
  {
    LED1_IO = 0;
    LED2_IO = 0;
    network_connected = 0;
    if (CFGCXT.type != WF_SOFT_AP && AppConfig.networkType == WF_INFRASTRUCTURE && network_connected == 0)
    {
      if (task_delay(TICK_MINUTE, 1))
        wifi_fail_count++;
    }
  }

  if (DHCPIsBound(0))
    network_connected = 1;
  else
    network_connected = 0;

  if (tcp_fail_count > 5 || unknown_status > 5)
  {
    LED1_IO = 0;
    LED2_IO = 0;
  }
  if (wifi_fail_count > 2)
  {
    wifi_fail_count = 0;

 //   Reset();
  }
  if (tcp_fail_count > 50 || unknown_status > 50)
  {
    tcp_fail_count = 0;
    unknown_status = 0;
    wifi_fail_count = 0;
    network_connected = 0;
//    Reset();
  }
  return network_connected;
}


/*****************************************************************************
*
* info_assemble
*
*  \param  char * vendor, custom's vendor name
*          char * model, custom's model name
*
*  \return string length of assembly customize's vendor information
*
*  \brief  Initializes the customer's vendor and model name for
*          provisioning
*
*****************************************************************************/
int
info_assemble(const char * vendor, const char *model, const char *sn)
{
  int info_len = 0;
  int assemble_len = 0;
  char * vendor_info = provision_info;

  memset(provision_info, 0, IHMS_LENGTH);

  // verify the assembly length
  assemble_len = strlen(STR_VENDOR) + strlen(vendor)
                 + strlen(STR_MODEL) + strlen(model)
                 + strlen(STR_SN) + 3;
  if (assemble_len > 95)
    return info_len;

  // vendor=
  memcpy(vendor_info, STR_VENDOR, strlen(STR_VENDOR));
  info_len = strlen(STR_VENDOR);

  // vendor="custom's vendor"
  memcpy(&vendor_info[info_len], vendor, strlen(vendor));
  info_len += strlen(vendor);

  // vendor="custom's vendor"&
  vendor_info[info_len] = '&'; // &
  info_len += 1;

  // vendor="custom's vendor"&model=
  memcpy(&vendor_info[info_len], STR_MODEL, strlen(STR_MODEL));
  info_len += strlen(STR_MODEL);

  // vendor="custom's vendor"&model="custom's model"
  memcpy(&vendor_info[info_len], model, strlen(model));
  info_len += strlen(model);

  // vendor="custom's vendor"&model="custom's model"&
  vendor_info[info_len] = '&'; // &
  info_len += 1;

  // vendor="custom's vendor"&model="custom's model"&sn=
  memcpy(&vendor_info[info_len], STR_SN, strlen(STR_SN));
  info_len += strlen(STR_SN);

  // vendor="custom's vendor"&model="custom's model"&sn="device's sn"
  memcpy(&vendor_info[info_len], sn, strlen(sn));
  info_len += strlen(sn);

  vendor_info[info_len] = 0;

  return info_len;
}

/*****************************************************************************
*
* Cloud_Init
*
*  \param  char * vendor - vendor name
*          char * model  - model name
*          char if_nbr   - network interface
*          int reset     - reset the settings to Exosite default
*
*  \return 1 success; 0 failure
*
*  \brief  Initializes the Exosite meta structure, UUID and
*          provision information
*
*****************************************************************************/
int
Cloud_Init(const char *vendor, const char *model, int reset)
{
  char struuid[IHMS_SN_MAXLENGTH];
  unsigned char uuid_len = 0;

  //IHMS_meta_init(reset);          //always initialize ihms meta structure

  uuid_len = IHMS_ReadUUID((unsigned char *)struuid);


  //IHMS_WriteMetaItem((unsigned char *)struuid, uuid_len, META_UUID);

  // read UUID into 'sn'
  info_assemble(vendor, model, struuid);

  cloud_initialized = 1;

  status_code = STATUS_INIT;

  return 1;
}


/*****************************************************************************
*
* Cloud_Activate
*
*  \param  None
*
*  \return 1  - activation success
*          0  - activation failure
*
*  \brief  Called after Init has been run in the past, but maybe comms were
*          down and we have to keep trying
*
*****************************************************************************/
int
Cloud_Activate(void)
{
  int length;
  char DataLen[5];
  int newcik = 0;
//  int http_status = 0;
  char *cmp_ss = "Content-Length:";
  char *cmp = cmp_ss;
  DWORD serverip = 0;
  const unsigned char server[6] = SERVERIP;
  serverip = (server[3] << 24 & 0xff000000) | (server[2] << 16 & 0xff0000)
          | (server[1] << 8 & 0xff00) | (server[0] & 0xff);
  long w, r;
  char rev[300];
  unsigned char strLen, len;
  unsigned char cik_len_valid = 0;
  char *p;
  unsigned char crlf = 0;
  unsigned char ciklen = 0;
  int   time_out = 0;
  // Flag cloud_initialized is set by Cloud_Init()
  /*
  if (!cloud_initialized) {
    status_code = STATUS_INIT;
    return newcik;
  }
    */

  // clean the content of http header array

  if(status_code == STATUS_INIT||status_code == STATUS_END){

      //to launch a new HTTP POST operation, clean the content of header at first

      if (sock == INVALID_SOCKET)
      {
        sock = TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, HTTP_PORT, TCP_PURPOSE_TCP_CLIENT);

      // TCP_OPEN_RAM_HOST for using dns name as server name
     //  TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, server_port, TCP_PURPOSE_GENERIC_TCP_CLIENT);

       if (sock  == INVALID_SOCKET) {
             status_code = STATUS_INIT;
       //      LEDS_OFF();
        //     LEDS_ON();
            return 0;
         }
       status_code = STATUS_READY;

      }
      status_code == STATUS_READY;
  }
  else if(status_code == STATUS_READY)
  {
      // Get activation Serial Number
        DelayMs(20);
        w = TCPIsPutReady(sock);

        if(w<2000ul) {
           //   StackTask();
          //  TCPFlush((TCP_SOCKET)sock);
              return 0; }

  //      LED1_ON(); LED2_OFF();

       // if(w>2000ul) { LED2_ON(); LED1_OFF();}

     //   DelayMs(100);
        length = strlen(provision_info);

        IHMS_itoa(DataLen, length, 10); //make a string for length

        sendLine(sock, POSTDATA_LINE, "/activate.php");
        sendLine(sock, HOST_LINE, NULL);
        sendLine(sock, CONTENT_LINE, NULL);
     //   IHMS_SocketSend(sock, "Connection: close\r\n", sizeof("Connection: close\r\n")-1);

        sendLine(sock, LENGTH_LINE, DataLen);

        IHMS_SocketSend(sock, provision_info, length);

        status_code = STATUS_RCV;
  }
  else if(status_code == STATUS_RCV)
  {
     DelayMs(20);
     r = TCPIsGetReady((TCP_SOCKET) sock);

     if(r<234u){ LED2_ON();  return 0;}
     // now read all data in RX buffer

    int count = 0;
    do
    {
        r = TCPGetArray((TCP_SOCKET)sock, (BYTE *)&rev[count], 300);
        count  = count + r;
        rev[count]=0;

        r = TCPIsGetReady((TCP_SOCKET) sock);

     }while(r>0u);

     rev[count] = 0 ;
      strLen = strlen(rev);
      len = strLen;
      p = rev;
      // Find 4 consecutive \r or \n - should be: \r\n\r\n
      while (0 < len && 4 > crlf)
      {
        if ('\r' == *p || '\n' == *p)
        {
          ++crlf;
        }
        else
        {
          crlf = 0;
          if (*cmp == *p)
          {
            // check the cik length from http response
            cmp++;
            if (cmp > cmp_ss + strlen(cmp_ss) - 1)
              cik_len_valid = 1;
          }
          else
            cmp = cmp_ss;
        }
        ++p;
        --len;
      }

     if(len>0)
     {
         LED1_ON();
         LED2_OFF();
     }



      // The body is the cik
        // TODO, be more robust - match Content-Length header value to CIK_LENGTH
      strncpy(CIK, p, CIK_LENGTH);

      CIK[40] = 0;
      newcik = 1;

    //IHMS_SocketClose(sock);

    TCPClose((TCP_SOCKET)sock);
    status_code = STATUS_END;
    sock = INVALID_SOCKET;

    return newcik;
  }

 // status_code = STATUS_INIT;
  return newcik;

}

/*****************************************************************************
*
* Cloud_Post
*
*  \param  pbuf - string buffer containing data to be sent
*          bufsize - number of bytes to send
*
*  \return 1 success; 0 failure
*
*  \brief  Writes data to Exosite cloud
*
*****************************************************************************/
int
Cloud_Post()
{
  int length;
  char DataLen[10];
//  int http_status = 0;
  char *cmp_ss = "Content-Length:";
  char *cmp = cmp_ss;
  DWORD serverip = 0;
  const unsigned char server[6] = SERVERIP ;
  serverip = (server[3] << 24 & 0xff000000) | (server[2] << 16 & 0xff0000)
          | (server[1] << 8 & 0xff00) | (server[0] & 0xff);
  long w, r;
  char rev[300];
  unsigned char len;
  char *p;
  unsigned char crlf = 0;
  int  time_out = 0;
  int tx_buff_size = 250;


  int  tmp_len =0 ;

  if(status_code == STATUS_INIT||status_code == STATUS_END){

      if (sock == INVALID_SOCKET)
      {
        sock = TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, HTTP_PORT, TCP_PURPOSE_TCP_CLIENT);

      // TCP_OPEN_RAM_HOST for using dns name as server name
     //  TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, server_port, TCP_PURPOSE_GENERIC_TCP_CLIENT);

       if (sock  == INVALID_SOCKET) {
             status_code = STATUS_INIT;
       //      LEDS_OFF();
        //     LEDS_ON();
            return 0;
         }
       status_code = STATUS_READY;

      }
      else{
          status_code == STATUS_READY;
      }
  }
  else if(status_code == STATUS_READY)
  {
       if(sent_header) DelayMs(20);
        w = TCPIsPutReady(sock);

       if(w<=250ul) {
              return 0; }

       if(sent_header){

             memset(header, 0, sizeof(header));
             length = PostHeaderGenerate("/rcvmotion.php", cmd_no, (int) start_time, (int) end_time);
             length = PostHeaderUpdateLength(length, MAXSENSORDATASIZE, 2);
             remain_count =0 ;
             sent_header = FALSE;
             sent_count = 0;
       }
       else 
       {
           if(tx_buffer_clear)
           {
             memset(header, 0, sizeof(header));
        
             int tx_num = MAXSENSORDATASIZE - sent_count;
             tx_num = tx_num>=125? 125:tx_num;

            AddPostData(sent_count, tx_num);

        //     memset(header, 65, 250);

            remain_count =0 ;
            sent_count +=tx_num;
           }
       }

    //    LED2_ON(); LED1_OFF();

       int send_len = strlen(&header[remain_count]);

   //   LED2_ON(); LED1_OFF();
      //if(send_len > 254)  {LED2_ON(); LED1_OFF()};
      /*  The max size of sliding window for TCP packet is 254? after testing.
       *  Don't know the reason, but if we set the number of sending data to 250,
       * the program works fine.
       *
       */
      tmp_len = send_len>250? 250:send_len;
  //    int tmp_len = IHMS_SocketSend(sock, &header[sent_count], send_len );

      tmp_len = TCPPutArray(sock, (BYTE *) &header[remain_count], tmp_len);

      TCPFlush((TCP_SOCKET)sock);

      LED2_ON(); LED1_OFF();

     if(tmp_len<send_len)
     {
         remain_count += tmp_len;
         tx_buffer_clear = FALSE;
         return 0;
     }

     remain_count = 0;
     tx_buffer_clear = TRUE;

     if(sent_count<MAXSENSORDATASIZE)    return 0 ;

     memset(header, 0, sizeof(header));
     sent_count = 0;
     remain_count = 0;
     status_code = STATUS_RCV;
  
  }
   else if(status_code == STATUS_RCV)
  {
     DelayMs(20);
     r = TCPIsGetReady((TCP_SOCKET) sock);

     if(r<200u){ LED2_ON();  return 0;}
     // now read all data in RX buffer

    int count = 0;
    do
    {
        r = TCPGetArray((TCP_SOCKET)sock, (BYTE *)&rev[count], 300);
        count  = count + r;
        rev[count]=0;

        r = TCPIsGetReady((TCP_SOCKET) sock);

     }while(r>0u);

    rev[count] = 0 ;

    TCPClose((TCP_SOCKET)sock);
    status_code = STATUS_END;
    sock = INVALID_SOCKET;
    status_code = STATUS_END;

    sent_header = TRUE;
    //now it's time to read time

    GetServerCmd(rev, "cmd=");
    cmd_no = GetServerCmd(rev, "no=");
    DelayMs(100);
    return 1;

  }

  return 0;
}

/*****************************************************************************
*
* Cloud_GetCmd
*
*  \param  pbuf - string buffer containing data to be sent
*          bufsize - number of bytes to send
*
*  \return 1 success; 0 failure
*
*  \brief  Writes data to Exosite cloud
*
*****************************************************************************/
int
Cloud_GetCmd()
{
  int length;
  char DataLen[10];
//  int http_status = 0;
  char *cmp_ss = "Content-Length:";
  char *cmp = cmp_ss;
  DWORD serverip = 0;
  const unsigned char server[6] = SERVERIP ;
  serverip = (server[3] << 24 & 0xff000000) | (server[2] << 16 & 0xff0000)
          | (server[1] << 8 & 0xff00) | (server[0] & 0xff);
  long w, r;
  char rev[300];
  unsigned char len;
  char *p;
  unsigned char crlf = 0;
  int  time_out = 0;
  int tx_buff_size = 250;

  int  tmp_len =0 ;

  if(status_code == STATUS_INIT||status_code == STATUS_END){

      if (sock == INVALID_SOCKET)
      {
        sock = TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, HTTP_PORT, TCP_PURPOSE_TCP_CLIENT);

      // TCP_OPEN_RAM_HOST for using dns name as server name
     //  TCPOpen(serverip, TCP_OPEN_IP_ADDRESS, server_port, TCP_PURPOSE_GENERIC_TCP_CLIENT);

       if (sock  == INVALID_SOCKET) {
             status_code = STATUS_INIT;
       //      LEDS_OFF();
        //     LEDS_ON();
            return 0;
         }
       status_code = STATUS_READY;

      }
      else  status_code == STATUS_READY;
  }
  else if(status_code == STATUS_READY)
  {
       if(sent_header) DelayMs(20);
        w = TCPIsPutReady(sock);

       if(w<=250ul) {
              return 0; }

       if(sent_header){

             memset(header, 0, sizeof(header));
             length = PostHeaderGenerate("/sendcmd.php", command, 0, 0);
             remain_count =0 ;
             sent_header = FALSE;
             sent_count = 0;
       }
    //    LED2_ON(); LED1_OFF();

       int send_len = strlen(&header[remain_count]);

   //   LED2_ON(); LED1_OFF();
      //if(send_len > 254)  {LED2_ON(); LED1_OFF()};
      /*  The max size of sliding window for TCP packet is 254? after testing.
       *  Don't know the reason, but if we set the number of sending data to 250,
       * the program works fine.
       *
       */
      tmp_len = send_len>250? 250:send_len;
  //    int tmp_len = IHMS_SocketSend(sock, &header[sent_count], send_len );

      tmp_len = TCPPutArray(sock, (BYTE *) &header[remain_count], tmp_len);

      TCPFlush((TCP_SOCKET)sock);

      LED2_ON(); LED1_OFF();

     if(tmp_len<send_len)
     {
         remain_count += tmp_len;
         return 0;
     }
     memset(header, 0, sizeof(header));
     sent_count = 0;
     remain_count = 0;
     status_code = STATUS_RCV;

  }
   else if(status_code == STATUS_RCV)
  {
     DelayMs(20);
     r = TCPIsGetReady((TCP_SOCKET) sock);

     if(r<200u){ LED2_ON();  return 0;}
     // now read all data in RX buffer

    int count = 0;
    do
    {
        r = TCPGetArray((TCP_SOCKET)sock, (BYTE *)&rev[count], 300);
        count  = count + r;
        rev[count]=0;

        r = TCPIsGetReady((TCP_SOCKET) sock);

     }while(r>0u);

    rev[count] = 0 ;

    TCPClose((TCP_SOCKET)sock);
    status_code = STATUS_END;
    sock = INVALID_SOCKET;
    status_code = STATUS_END;

    sent_header = TRUE;
    //now it's time to read time

    command = GetServerCmd(rev, "cmd=");
    cmd_no = GetServerCmd(rev, "no=");

    return 1;

  }

  return 0;
}

/*****************************************************************************
*
* Cloud_Read
*
*  \param  palias - string, name of the datasource alias to read from
*          pbuf - read buffer to put the read response into
*          buflen - size of the input buffer
*
*  \return number of bytes read
*
*  \brief  Reads data from Exosite cloud
*
*****************************************************************************/
int
Cloud_Read(char * palias, char * pbuf, unsigned char buflen)
{
  int success = 0;
  int http_status = 0;
  unsigned char strLen, len, vlen;
  char *p, *pcheck;

  if (!cloud_initialized) {
    status_code = STATUS_INIT;
    return success;
  }

  long sock = connect_to_cloud();
  if (sock < 0) {
    status_code = STATUS_END;
    return 0;
  }

  sendLine(sock, GETDATA_LINE, palias);
  sendLine(sock, HOST_LINE, NULL);
  sendLine(sock, CIK_LINE, CIK);
  sendLine(sock, ACCEPT_LINE, "\r\n");

  pcheck = palias;
  vlen = 0;

  http_status = get_http_status(sock);
  if (200 == http_status)
  {
    char strBuf[RX_SIZE];
    unsigned char crlf = 0;

    do
    {
      strLen = IHMS_SocketRecv(sock, strBuf, RX_SIZE);
      len = strLen;
      p = strBuf;

      // Find 4 consecutive \r or \n - should be: \r\n\r\n
      while (0 < len && 4 > crlf)
      {
        if ('\r' == *p || '\n' == *p)
        {
          ++crlf;
        }
        else
        {
          crlf = 0;
        }
        ++p;
        --len;
      }

      // The body is "<key>=<value>"
      if (0 < len && 4 == crlf && buflen > vlen)
      {
        // Move past "<key>"
        while (0 < len && 0 != *pcheck)
        {
          if (*pcheck == *p)
          {
            ++pcheck;
          }
          else
          {
            pcheck = palias;
          }
          ++p;
          --len;
        }

        // Match '=',  we should now have '<key>='
        if (0 < len && 0 == *pcheck && '=' == *p)
        {
          ++p;
          --len;
        }

        // read in the rest of the body as the value
        while (0 < len && buflen > vlen)
        {
          pbuf[vlen++] = *p++;
          --len;
        }
      }
    } while (RX_SIZE == strLen);
  }

  IHMS_SocketClose(sock);

  if (0 == http_status)
    status_code = STATUS_END;
  if (200 == http_status)
  {
    status_code = STATUS_END;
  }
  if (204 == http_status)
  {
    status_code = STATUS_END;
  }
  if (401 == http_status)
  {
    status_code = STATUS_END;
  }

  return vlen;
}


/*****************************************************************************
*
* connect_to_cloud
*
*  \param  None
*
*  \return success: socket handle; failure: -1;
*
*  \brief  Establishes a connection with Exosite API server
*
*****************************************************************************/
long
connect_to_cloud(void)
{
  unsigned char connectRetries = 0;
  long sock = -1;

  while (connectRetries++ <= IHMS_MAX_CONNECT_RETRY_COUNT) {
    sock = IHMS_SocketOpenTCP();
    if (sock == -1)
    {
      continue;
    }
    if (IHMS_ServerConnect(sock) < 0)  // Try to connect
    {
      // return and let the caller retry us if they want
      continue;
    } else {
      connectRetries = 0;
      break;
    }
  }

  // Success
  return sock;
}


/*****************************************************************************
*
* get_http_status
*
*  \param  socket handle, pointer to expected HTTP response code
*
*  \return http response code, 0 tcp failure
*
*  \brief  Reads first 12 bytes of HTTP response and extracts the 3 byte code
*
*****************************************************************************/
int
get_http_status(long socket)
{
  char rxBuf[12];
  int rxLen = 0;
  int code = 0;

  rxLen = IHMS_SocketRecv(socket, rxBuf, 12);

  if (12 == rxLen)
  {
    // exampel '4','0','4' =>  404  (as number)
    code = (((rxBuf[9] - 0x30) * 100) +
            ((rxBuf[10] - 0x30) * 10) +
            (rxBuf[11] - 0x30));
    return code;
  }
  return 0;
}


/*****************************************************************************
*
*  sendLine
*
*  \param  Which line type
*
*  \return socket handle
*
*  \brief  Sends data out
*
*****************************************************************************/
void
sendLine(long socket, unsigned char LINE, const char * payload)
{
  char strBuf[70];
  unsigned char strLen;

  switch(LINE) {
    case CIK_LINE:
      strLen = strlen(STR_CIK_HEADER);
      memcpy(strBuf,STR_CIK_HEADER,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += strlen(STR_CRLF);
      break;
    case HOST_LINE:
      strLen = strlen(STR_HOST);
      memcpy(strBuf,STR_HOST,strLen);
      break;
    case CONTENT_LINE:
      strLen = strlen(STR_CONTENT);
      memcpy(strBuf,STR_CONTENT,strLen);
      break;
    case ACCEPT_LINE:
      strLen = strlen(STR_ACCEPT);
      memcpy(strBuf,STR_ACCEPT,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      break;
    case LENGTH_LINE: // Content-Length: NN
      strLen = strlen(STR_CONTENT_LENGTH);
      memcpy(strBuf,STR_CONTENT_LENGTH,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      memcpy(&strBuf[strLen],STR_CRLF, 2);
      strLen += 2;
      break;
    case GETDATA_LINE:
      strLen = strlen(STR_GET_URL);
      memcpy(strBuf,STR_GET_URL,strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, strlen(STR_HTTP));
      strLen += strlen(STR_HTTP);
      break;
    case POSTDATA_LINE:
      strLen = strlen("POST ");
      memcpy(strBuf,"POST ", strLen);
      memcpy(&strBuf[strLen],payload, strlen(payload));
      strLen += strlen(payload);
      memcpy(&strBuf[strLen],STR_HTTP, strlen(STR_HTTP));
      strLen += strlen(STR_HTTP);
      break;
    case EMPTY_LINE:
      strLen = strlen(STR_CRLF);
      memcpy(strBuf,STR_CRLF,strLen);
      break;
    default:
      break;
  }

  strBuf[strLen] = 0;
  IHMS_SocketSend(socket, strBuf, strLen);

  return;
}


/*****************************************************************************
*
*  check_cloud_activated
*
*  \param  None
*
*  \return  None
*
*  \brief  Checks cloud status is activated
*
*****************************************************************************/
void check_cloud_activated(void)
{
  int activate_status = 0;

    activate_status = Cloud_Activate();
    if (activate_status)
      badcik = 0;
    else
      badcik = 1;

  return;
}


/*****************************************************************************
*
*  HTTP Client
*
*  \param  None
*
*  \return  None
*
*  \brief  Main function of Exosite demo code
*
*****************************************************************************/
void HTTPClient(void)
{
    int i;
 // char header[10000];   // for send HTTP HEADER and POST content in one packet

  if (!check_network_connected())
    return;

  // worked on demo app every 1/3 second
 // if (badcik==1 ||send_data==TRUE)

  if (badcik==1 ||send_data==TRUE||task_delay(TICK_SECOND, 1))
            workon_demo =1;

  if (workon_demo)
  {

    if (badcik == 1)
    {
      badcik = 1;
      check_cloud_activated();

      if (status_code == STATUS_END )
      {
           cloud_status = CLOUD_HOME;
           workon_demo = 0;
           // generate a new data set
      //    generateData(1500);
      }
    }
    else
    {
      if ( cloud_status == CLOUD_HOME)
      {


          if(send_data == TRUE)
          {
            if(Cloud_Post())
            {
                cloud_status =  CLOUD_HOME;
                workon_demo = 0;
                send_data = FALSE;
                LEDS_OFF();
                DelayMs(100);

              }
             else    workon_demo = 1;  //waiting for HTTP response
          }
          else
          {
            // ping or poll for a command from cloud server
              if(Cloud_GetCmd())
              {
                    workon_demo = 0;
                    cloud_status =  CLOUD_HOME;
                    command = 1;
                    LEDS_OFF();
              }
              

          }
      }

   /*
     else if (CLOUD_READING == cloud_status)
     {
      //   if (read_command())
         cloud_status++;
     }
      else if (CLOUD_WRITING == cloud_status)
      {
          if (heart_beat_report())
          cloud_status++;
      }
      else if (CLOUD_BUTTON_STATUS_UPDATE == cloud_status)
      {
   //     if (button_monitor())
      //    cloud_status = CLOUD_READING;
          cloud_status = CLOUD_HOME;
      }
   */
    }
  }

  // indicate last status
//   show_cloud_status();

  return;
}

/*****************************************************************************
*
*  task_delay
*
*  \param  None
*
*  \return  delay counting is done
*
*  \brief  Demo tasks delay function
*
*****************************************************************************/
int task_delay(int delay_time, int count)
{
  if (TickGet() - demo_delay_time_tick > delay_time)
  {
    delay_count++;
    demo_delay_time_tick = TickGet();
  }

  if (count <= delay_count)
  {
    delay_count = 0;
    return 1;
  }
  else
    return 0;
}
