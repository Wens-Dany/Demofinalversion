/* 
 * File:   httpclient.h
 * Author: chim
 *
 * Created on November 29, 2013, 7:07 PM
 */

#ifndef HTTPCLIENT_H
#define	HTTPCLIENT_H

#define IHMS_VENDOR_MAXLENGTH                 20
#define IHMS_MODEL_MAXLENGTH                  20
#define IHMS_SN_MAXLENGTH                     25
#define IHMS_DEMO_UPDATE_INTERVAL            4000// ms
#define CIK_LENGTH                            40

//local defines
#define IHMS_MAX_CONNECT_RETRY_COUNT 5
#define IHMS_LENGTH IHMS_SN_MAXLENGTH + IHMS_MODEL_MAXLENGTH + IHMS_VENDOR_MAXLENGTH
#define RX_SIZE 50
#define CIK_LENGTH 40
#define MAC_LEN 6

char provision_info[IHMS_LENGTH];
char CIK[CIK_LENGTH + 1];
//#define HEADER_LENGTH            300
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

#endif	/* HTTPCLIENT_H */

int info_assemble(const char * vendor, const char *model, const char *sn);
int init_UUID(unsigned char if_nbr);
int get_http_status(long socket);
long connect_to_cloud();
void sendLine(long socket, unsigned char LINE, const char * payload);

// global functions
int Cloud_Post();
int Cloud_Read(char * palias, char * pbuf, unsigned char buflen);
int Cloud_Init(const char *vendor, const char *model, int reset);
int Cloud_Activate(void);
//void IHMS_SetCIK(char * pCIK);
//int IHMS_GetCIK(char * pCIK);
int Cloud_StatusCode(void);


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
void httpclient(void);
