/*********************************************************************
 *
 *  Main Application Entry Point
 *   -Demonstrates how to call and use the Microchip WiFi Module and
 *    TCP/IP stack
 *   -Reference: Microchip TCP/IP Stack Help (TCPIP Stack Help.chm)
 *
 *********************************************************************
 * FileName:           Main.c
 * Dependencies:    TCPIP.h
 * Processor:          PIC32MX695F512H
 * Compiler:           Microchip  XC32 Compiler
 * Company:          Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2013 Microchip Technology Inc.  All rights
 * reserved.
 *
 * File Description:
 * Change History:
 * Date         Comment
 * ----------   -----------------------------------------
 * 10/22/2012    Initial release  Wifi G Demo        (MRF24WG)
 ********************************************************************/
/*
  * This macro uniquely defines this file as the main entry point.
  * There should only be one such definition in the entire project,
  * and this file must define the AppConfig variable as described below.
  */
#define THIS_IS_STACK_APPLICATION

#include "Main.h"

#if defined( WF_CONSOLE )
#include "TCPIP Stack/WFConsole.h"
//#include "IperfApp.h" // removing to compile for IDE v1.90
#endif

//
// Differences to wifi comm demo board (MRF24WB0MA) :
//        Wifi comm demo board is centered on variable CPElements.
//        Wifi comm demo SSID : MCHP_xxxx
//        SW0 functions : On powerup initiates self test.
//
//        Wifi G demo board is centered on variable AppConfig, since this is the generic approach adopted by
//        TCPIP demo/ezconsole/ezconfig apps.
//        Wifi G demo	  SSID : MCHP_G_xxxx
//        SW0 functions : On powerup initiates self test.
//                                When running, initiates reboot to factory default conditions.
//

//
//  Wifi G Demo Web Pages
//  Generate a WifiGDemoMPFSImg.c file using the MPFS utility (refer to Convert WebPages to MPFS.bat)
//  that gets compiled into source code and programmed into the flash of the uP.

APP_CONFIG AppConfig;

static unsigned short wOriginalAppConfigChecksum;    // Checksum of the ROM defaults for AppConfig
//extern unsigned char TelnetPut(unsigned char c);

// extern function for Exosite Demo
extern void Store_App_Config(void);


extern BOOL motionsensor_init(void);

extern int button_state;

UINT8 g_scan_done = 0;        // WF_PRESCAN   This will be set wheneven event scan results are ready.
UINT8 g_prescan_waiting = 1;  // WF_PRESCAN   This is used only to allow POR prescan once.


// Private helper functions.
static void InitAppConfig(void);
static void InitializeBoard(void);
static void SelfTest(void);
extern void WF_Connect(void);

BOOL send_data = FALSE;

void connect_ap(void)
{
  // int x;

   UINT8 ssidLen;

   CFGCXT.security = WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
   memcpy(CFGCXT.key, MY_DEFAULT_PSK_PHRASE, sizeof(MY_DEFAULT_PSK_PHRASE));
   CFGCXT.key[sizeof(MY_DEFAULT_PSK_PHRASE)] = 0;
   memcpy(CFGCXT.ssid, MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
   CFGCXT.ssid[sizeof(MY_DEFAULT_SSID_NAME)]= 0;

   WF_CPGetSsid(1, (UINT8*)&CFGCXT.prevSSID, &ssidLen);
                // WF_CPGetSsid(ConnectionProfileID, (UINT8*)&CFGCXT.prevSSID, &ssidLen);

   CFGCXT.prevSSID[ssidLen] = 0;
   CFGCXT.type = WF_INFRASTRUCTURE;

       // Copy wifi cfg data to be committed
#if defined ( EZ_CONFIG_STORE )
    strcpy((char *)AppConfig.MySSID, (char *)CFGCXT.ssid);
    AppConfig.SsidLength = strlen((char*)(CFGCXT.ssid));

	/* Going to set security type */
    AppConfig.SecurityMode =  CFGCXT.security;

    /* Going to save the key, if required */
    if (CFGCXT.security != WF_SECURITY_OPEN)
    {
        BYTE  key_size =0;

        switch ((BYTE)CFGCXT.security)
        {
            case WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:  //wpa passphrase
                key_size = strlen((char*)(CFGCXT.key)); //ascii so use strlen
                break;
            case WF_SECURITY_WPA_AUTO_WITH_KEY: //wpa pre-calculated key!!!
                key_size = 32;
                break;
            case WF_SECURITY_WEP_40:
                key_size = 5;
                break;
            case WF_SECURITY_WEP_104:
                key_size = 13;
                break;

        }
        memcpy(AppConfig.SecurityKey, CFGCXT.key, key_size);
        AppConfig.SecurityKey[strlen((char*)(CFGCXT.key))] = 0;

    }

    /* Going to save the network type */
     AppConfig.networkType = CFGCXT.type;
     AppConfig.dataValid = 1; /* Validate wifi configuration */
   //  CFGCXT.isWifiDoneConfigure =1;
     WF_START_EASY_CONFIG();

#endif
}



// Exception Handlers
// If your code gets here, you either tried to read or write
// a NULL pointer, or your application overflowed the stack
// by having too many local variables or parameters declared.
void _general_exception_handler(unsigned cause, unsigned status)
{
	Nop();
	Nop();
}

// ************************************************************
// Main application entry point.
// ************************************************************
int main(void)
{
    static DWORD t = 0;
    static DWORD dwLastIP = 0;

#if defined (EZ_CONFIG_STORE)
    static DWORD ButtonPushStart = 0;
#endif
    UINT8         channelList[] = MY_DEFAULT_CHANNEL_LIST_PRESCAN;  // WF_PRESCAN
    tWFScanResult bssDesc;
#if 0
    INT8 TxPower;   // Needed to change MRF24WG transmit power.
#endif

    // Initialize application specific hardware
    InitializeBoard();

    // Initialize TCP/IP stack timer
    TickInit();

    #if defined(STACK_USE_MPFS2)
    // Initialize the MPFS File System
	// Generate a WifiGDemoMPFSImg.c file using the MPFS utility (refer to Convert WebPages to MPFS.bat)
	// that gets compiled into source code and programmed into the flash of the uP.
    MPFSInit();
    #endif

    // Initialize Stack and application related NV variables into AppConfig.
    InitAppConfig();

    // Initialize core stack layers (MAC, ARP, TCP, UDP) and
    // application modules (HTTP, SNMP, etc.)
    StackInit();


#if 0
    // Below is used to change MRF24WG transmit power.
    // This has been verified to be functional (Jan 2013)
    if (AppConfig.networkType == WF_SOFT_AP)
    {
        WF_TxPowerGetMax(&TxPower);
        WF_TxPowerSetMax(TxPower);
    }
#endif

    // Run Self Test if SW0 pressed on startup
    if(SW0_IO == 0)
        SelfTest();

    // run i2c init to enable motion senser

    if(!motionsensor_init()) LED1_ON();    // if init fails, what we can do ?
    // possible call a motion sensor self test here

    #if defined ( EZ_CONFIG_SCAN )
    // Initialize WiFi Scan State Machine NV variables
    WFInitScan();
    #endif

    // WF_PRESCAN: Pre-scan before starting up as SoftAP mode
    WF_CASetScanType(MY_DEFAULT_SCAN_TYPE);
    WF_CASetChannelList(channelList, sizeof(channelList));

    if (WFStartScan() == WF_SUCCESS) {
       SCAN_SET_DISPLAY(SCANCXT.scanState);
       SCANCXT.displayIdx = 0;
    }

    /* I don't know the reason, but reconfiguration of the network setting of
     * Wi-Fi Module is involved so many functions in the program. At this moment
     * putting the program here works fine.
     * Nov 16 2013
     */
    connect_ap();
    // Needed to trigger g_scan_done
    WFRetrieveScanResult(0, &bssDesc);

    #if defined(STACK_USE_ZEROCONF_LINK_LOCAL)
    // Initialize Zeroconf Link-Local state-machine, regardless of network type.
    ZeroconfLLInitialize();
    #endif

    #if defined(STACK_USE_ZEROCONF_MDNS_SD)
    // Initialize DNS Host-Name from TCPIPConfig.h, regardless of network type.
    mDNSInitialize(MY_DEFAULT_HOST_NAME);
    mDNSServiceRegister(
            // (const char *) AppConfig.NetBIOSName,        // base name of the service. Ensure uniformity with CheckHibernate().
            (const char *) "DemoWebServer",          // base name of the service. Ensure uniformity with CheckHibernate().
            "_http._tcp.local",                      // type of the service
            80,	                                     // TCP or UDP port, at which this service is available
            ((const BYTE *)"path=/index.htm"),       // TXT info
            1,                                       // auto rename the service when if needed
            NULL,                                    // no callback function
            NULL                                     // no application context
            );
    mDNSMulticastFilterRegister();
    #endif

    // just get the WI-FI MAC address as device identity

    Cloud_Init("ihms","devicev100", 0);

    #if defined(WF_CONSOLE)
    // Initialize the WiFi Console App
    WFConsoleInit();
    #endif

    // Now that all items are initialized, begin the co-operative
    // multitasking loop.  This infinite loop will continuously
    // execute all stack-related tasks, as well as your own
    // application's functions.  Custom functions should be added
    // at the end of this loop.
    // Note that this is a "co-operative mult-tasking" mechanism
    // where every task performs its tasks (whether all in one shot
    // or part of it) and returns so that other tasks can do their
    // job.
    // If a task needs very long time to do its job, it must be broken
    // down into smaller pieces so that other tasks can have CPU time.
    while(1)
    {
         if (AppConfig.networkType == WF_SOFT_AP) {
            if (g_scan_done) {
                if (g_prescan_waiting) {
                     SCANCXT.displayIdx = 0;
                     while (IS_SCAN_STATE_DISPLAY(SCANCXT.scanState)) {
                         WFDisplayScanMgr();
                     }

                     #if defined(WF_CS_TRIS)
                     WF_Connect();
                     #endif
                     g_scan_done = 0;
                     g_prescan_waiting = 0;
                }
            }
         }

        #if defined (EZ_CONFIG_STORE)
        // Hold SW0 for 4 seconds to reset to defaults.
        if (SW0_IO == 0u) {  // Button is pressed
            button_state = 1;
            if (ButtonPushStart == 0)  //Just pressed
                ButtonPushStart = TickGet();
            else
            {
                if(TickGet() - ButtonPushStart > 2*TICK_SECOND)
                {

                    start_time = TickGet();
                    collect_sensor_data(0);
                    end_time = TickGet();
                    send_data = TRUE;
                }
            }     //  RestoreWifiConfig();

        }
        else
        {
            ButtonPushStart = 0; //Button release reset the clock
        }
        // now check the command received from cloud server, run it if command != 0
        // command == 1   --  start a new data collection from motion sensor


        if(command==1)
        {
                    start_time = TickGet();
                    collect_sensor_data(0);
                    end_time = TickGet();
                    send_data = TRUE;
                
                    command = 0;   //reset the command flag after executing the corresponding operation
        }

         /*{
                    start_time = TickGet();
                    collect_sensor_data(0);
                    end_time = TickGet();
                    send_data = TRUE;

                    command = 0;   //reset the command flag after executing the corresponding operation
        }*/


        if (AppConfig.saveSecurityInfo)
        {
            // set true by WF_ProcessEvent after connecting to a new network
            // get the security info, and if required, push the PSK to EEPROM
            if ((AppConfig.SecurityMode == WF_SECURITY_WPA_WITH_PASS_PHRASE) ||
                (AppConfig.SecurityMode == WF_SECURITY_WPA2_WITH_PASS_PHRASE) ||
                (AppConfig.SecurityMode == WF_SECURITY_WPA_AUTO_WITH_PASS_PHRASE))
            {
                // only need to save when doing passphrase
                tWFCPElements profile;
                UINT8 connState;
                UINT8 connID;
                WF_CMGetConnectionState(&connState, &connID);
                WF_CPGetElements(connID, &profile);

                memcpy((char*)AppConfig.SecurityKey, (char*)profile.securityKey, 32);
                AppConfig.SecurityMode--; // the calc psk is exactly one below for each passphrase option
                AppConfig.SecurityKeyLength = 32;

                SaveAppConfig(&AppConfig);
            }

            AppConfig.saveSecurityInfo = FALSE;
        }
        #endif // EZ_CONFIG_STORE

        // Blink LED0 twice per sec when unconfigured, once per sec after config
        if((TickGet() - t >= TICK_SECOND/(4ul - (CFGCXT.isWifiDoneConfigure*2ul))))
        {
            t = TickGet();
            LED0_INV();
        }

        // This task performs normal stack task including checking
        // for incoming packet, type of packet and calling
        // appropriate stack entity to process it.
        StackTask();

        // This task invokes each of the core stack application tasks
        StackApplications();

        // Enable WF_USE_POWER_SAVE_FUNCTIONS
        WiFiTask();

        #if defined(STACK_USE_ZEROCONF_LINK_LOCAL)
        ZeroconfLLProcess();
        #endif

        #if defined(STACK_USE_ZEROCONF_MDNS_SD)
        mDNSProcess();
        #endif

  //    Exosite_Demo();
        HTTPClient();
        // Process application specific tasks here.
        // Any custom modules or processing you need to do should
        // go here.
        #if defined(WF_CONSOLE)
		WFConsoleProcess();
		WFConsoleProcessEpilogue();
		#endif

		// If the local IP address has changed (ex: due to DHCP lease change)
		// write the new IP address to the LCD display, UART, and Announce
		// service
		if(dwLastIP != AppConfig.MyIPAddr.Val)
		{
			dwLastIP = AppConfig.MyIPAddr.Val;
			DisplayIPValue(AppConfig.MyIPAddr);

			#if defined(STACK_USE_ANNOUNCE)
			AnnounceIP();
	 		#endif

			#if defined(STACK_USE_ZEROCONF_MDNS_SD)
			mDNSFillHostRecord();
	 		#endif
		}

    }
}

/****************************************************************************
  Function:
    static void InitializeBoard(void)

  Description:
    This routine initializes the hardware.  It is a generic initialization
    routine for many of the Microchip development boards, using definitions
    in HardwareProfile.h to determine specific initialization.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void InitializeBoard(void)
{
    // Note: WiFi Module hardware Initialization handled by StackInit() Library Routine

    // Enable multi-vectored interrupts
    INTEnableSystemMultiVectoredInt();

    // Enable optimal performance
    SYSTEMConfigPerformance(GetSystemClock());

    // Use 1:1 CPU Core:Peripheral clocks
    mOSCSetPBDIV(OSC_PB_DIV_1);

    // Disable JTAG port so we get our I/O pins back, but first
    // wait 50ms so if you want to reprogram the part with
    // JTAG, you'll still have a tiny window before JTAG goes away.
    // The PIC32 Starter Kit debuggers use JTAG and therefore must not
    // disable JTAG.
    DelayMs(50);
    DDPCONbits.JTAGEN = 0;

    // LEDs
    LEDS_OFF();
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;

    // Push Button
    SW0_TRIS = 1;

}

static ROM BYTE SerializedMACAddress[6] = {MY_DEFAULT_MAC_BYTE1, MY_DEFAULT_MAC_BYTE2, MY_DEFAULT_MAC_BYTE3, MY_DEFAULT_MAC_BYTE4, MY_DEFAULT_MAC_BYTE5, MY_DEFAULT_MAC_BYTE6};

/*********************************************************************
 * Function:        void InitAppConfig(void)
 *
 * PreCondition:    MPFSInit() is already called.
 *
 * Input:           None
 *
 * Output:          Write/Read non-volatile config variables.
 *
 * Side Effects:    None
 *
 * Overview:        None
 *
 * Note:            None
 ********************************************************************/
static void InitAppConfig(void)
{
#if defined(EEPROM_CS_TRIS) || defined(SPIFLASH_CS_TRIS)
		unsigned char vNeedToSaveDefaults = 0;
#endif

		while(1)
		{
			// Start out zeroing all AppConfig bytes to ensure all fields are
			// deterministic for checksum generation
			memset((void*)&AppConfig, 0x00, sizeof(AppConfig));

			AppConfig.Flags.bIsDHCPEnabled = TRUE;
			AppConfig.Flags.bInConfigMode = TRUE;
			memcpypgm2ram((void*)&AppConfig.MyMACAddr, (ROM void*)SerializedMACAddress, sizeof(AppConfig.MyMACAddr));
	//		{
	//			_prog_addressT MACAddressAddress;
	//			MACAddressAddress.next = 0x157F8;
	//			_memcpy_p2d24((char*)&AppConfig.MyMACAddr, MACAddressAddress, sizeof(AppConfig.MyMACAddr));
	//		}


			// SoftAP on certain setups with IP 192.168.1.1 has problem with DHCP client assigning new IP address on redirection.
			// 192.168.1.1 is a common IP address with most APs. This is still under investigation.
			// For now, assign this as 192.168.1.3

			AppConfig.MyIPAddr.Val = 192ul | 168ul<<8ul | 1ul<<16ul | 3ul<<24ul;
			AppConfig.DefaultIPAddr.Val = AppConfig.MyIPAddr.Val;
			AppConfig.MyMask.Val = 255ul | 255ul<<8ul | 0ul<<16ul | 0ul<<24ul;
			AppConfig.DefaultMask.Val = AppConfig.MyMask.Val;
			AppConfig.MyGateway.Val = AppConfig.MyIPAddr.Val;
			AppConfig.PrimaryDNSServer.Val = AppConfig.MyIPAddr.Val;
			AppConfig.SecondaryDNSServer.Val = AppConfig.MyIPAddr.Val;

			// Load the default NetBIOS Host Name
			memcpypgm2ram(AppConfig.NetBIOSName, (ROM void*)MY_DEFAULT_HOST_NAME, 16);
			FormatNetBIOSName(AppConfig.NetBIOSName);

		#if defined(WF_CS_TRIS)
			// Load the default SSID Name
			WF_ASSERT(sizeof(MY_DEFAULT_SSID_NAME) <= sizeof(AppConfig.MySSID));
			memcpypgm2ram(AppConfig.MySSID, (ROM void*)MY_DEFAULT_SSID_NAME, sizeof(MY_DEFAULT_SSID_NAME));
			AppConfig.SsidLength = sizeof(MY_DEFAULT_SSID_NAME) - 1;
			AppConfig.SecurityMode = MY_DEFAULT_WIFI_SECURITY_MODE;
			if (AppConfig.SecurityMode == WF_SECURITY_WEP_40)
			{
				AppConfig.WepKeyIndex  = MY_DEFAULT_WEP_KEY_INDEX;
				memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_WEP_KEYS_40, sizeof(MY_DEFAULT_WEP_KEYS_40) - 1);
				AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_WEP_KEYS_40) - 1;
			}
			else if (AppConfig.SecurityMode == WF_SECURITY_WEP_104)
			{
				AppConfig.WepKeyIndex  = MY_DEFAULT_WEP_KEY_INDEX;
				memcpypgm2ram(AppConfig.SecurityKey, (ROM void*)MY_DEFAULT_WEP_KEYS_104, sizeof(MY_DEFAULT_WEP_KEYS_104) - 1);
				AppConfig.SecurityKeyLength = sizeof(MY_DEFAULT_WEP_KEYS_104) - 1;
			}
			AppConfig.networkType = MY_DEFAULT_NETWORK_TYPE;
			AppConfig.dataValid = 0;
		#endif

			// Compute the checksum of the AppConfig defaults as loaded from ROM
			wOriginalAppConfigChecksum = CalcIPChecksum((BYTE*)&AppConfig, sizeof(AppConfig));

#if defined(EEPROM_CS_TRIS)
            NVM_VALIDATION_STRUCT NVMValidationStruct;

            // Check to see if we have a flag set indicating that we need to
            // save the ROM default AppConfig values.
            if(vNeedToSaveDefaults)
                SaveAppConfig(&AppConfig);

            // Read the NVMValidation record and AppConfig struct out of EEPROM/Flash
                XEEReadArray(0x0000, (BYTE*)&NVMValidationStruct, sizeof(NVMValidationStruct));
                XEEReadArray(sizeof(NVMValidationStruct), (BYTE*)&AppConfig, sizeof(AppConfig));

            // Check EEPROM/Flash validitity.  If it isn't valid, set a flag so
            // that we will save the ROM default values on the next loop
            // iteration.
            if((NVMValidationStruct.wConfigurationLength != sizeof(AppConfig)) ||
               (NVMValidationStruct.wOriginalChecksum != wOriginalAppConfigChecksum) ||
               (NVMValidationStruct.wCurrentChecksum != CalcIPChecksum((BYTE*)&AppConfig, sizeof(AppConfig))))
            {
                // Check to ensure that the vNeedToSaveDefaults flag is zero,
                // indicating that this is the first iteration through the do
                // loop.  If we have already saved the defaults once and the
                // EEPROM/Flash still doesn't pass the validity check, then it
                // means we aren't successfully reading or writing to the
                // EEPROM/Flash.  This means you have a hardware error and/or
                // SPI configuration error.
                if(vNeedToSaveDefaults)
                {
                    while(1);
                }

                // Set flag and restart loop to load ROM defaults and save them
                vNeedToSaveDefaults = 1;
                continue;
            }

            // If we get down here, it means the EEPROM/Flash has valid contents
            // and either matches the ROM defaults or previously matched and
            // was run-time reconfigured by the user.  In this case, we shall
            // use the contents loaded from EEPROM/Flash.
            break;
#endif
			break;

		}


    #if defined (EZ_CONFIG_STORE)
		// Set configuration for ZG from NVM
		/* Set security type and key if necessary, convert from app storage to ZG driver */

		if (AppConfig.dataValid)
			CFGCXT.isWifiDoneConfigure = 1;

		AppConfig.saveSecurityInfo = FALSE;
    #endif // EZ_CONFIG_STORE

}

/****************************************************************************
  Function:
    void SelfTest()

  Description:
    This routine performs a self test of the hardware.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
static void SelfTest(void)
{
    char value = 0;

    // Verify MRF24WB/G0MA MAC Address
    if(AppConfig.MyMACAddr.v[0] == 0x00 && AppConfig.MyMACAddr.v[1] == 0x1E)
    {

        // Toggle LED's
        while(1)
        {
            LED0_IO = value;
            LED1_IO = value >> 1;
            LED2_IO = value >> 2;

            DelayMs(400);

            if(value == 8)
                value = 0;
            else
                value++;

        }
    }
    else    // MRF24WG0MA Failure
    {
        while(1)
        {
            LEDS_ON();
            DelayMs(700);
            LEDS_OFF();
            DelayMs(700);
        }
    }
}

/****************************************************************************
  Function:
    void DisplayIPValue(IP_ADDR IPVal)

  Description:
    This routine formats and prints the current IP Address.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void DisplayIPValue(IP_ADDR IPVal)
{
	printf("%u.%u.%u.%u", IPVal.v[0], IPVal.v[1], IPVal.v[2], IPVal.v[3]);
}

/****************************************************************************
  Function:
	void SaveAppConfig(const APP_CONFIG *ptrAppConfig)

  Description:
    This routine saves the AppConfig into EEPROM.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void SaveAppConfig(const APP_CONFIG *ptrAppConfig)
{
    NVM_VALIDATION_STRUCT NVMValidationStruct;

    // Get proper values for the validation structure indicating that we can use
    // these EEPROM/Flash contents on future boot ups
    NVMValidationStruct.wOriginalChecksum = wOriginalAppConfigChecksum;
    NVMValidationStruct.wCurrentChecksum = CalcIPChecksum((BYTE*)ptrAppConfig, sizeof(APP_CONFIG));
    NVMValidationStruct.wConfigurationLength = sizeof(APP_CONFIG);
    // store the WiFi setting into internal flash
 //   Store_App_Config();
#if 0
    // Write the validation struct and current AppConfig contents to EEPROM/Flash
    XEEBeginWrite(0x0000);
    XEEWriteArray((BYTE*)&NVMValidationStruct, sizeof(NVMValidationStruct));
    XEEWriteArray((BYTE*)ptrAppConfig, sizeof(APP_CONFIG));
#endif
}

#if defined (EZ_CONFIG_STORE)
/****************************************************************************
  Function:
	void RestoreWifiConfig(void)

  Description:
    This routine performs reboot when SW0 is pressed.

  Precondition:
    None

  Parameters:
    None - None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void RestoreWifiConfig(void)
{
#if 0
    XEEBeginWrite(0x0000);
    XEEWrite(0xFF);
    XEEWrite(0xFF);
    XEEEndWrite();
#endif
    // reboot here...
    //LED_PUT(0x00);
    while(SW0_IO == 0u);
    Reset();
}
#endif // EZ_CONFIG_STORE
