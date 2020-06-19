/********************************** (C) COPYRIGHT *******************************
* File Name          : peripheral.C
* Author             : WCH
* Version            : V1.0
* Date               : 2018/12/10
* Description        : 外设从机多连接应用程序，初始化广播连接参数，然后广播，连接主机后，
*                      请求更新连接参数，通过自定义服务传输数据           
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "CONFIG.h"
#include "CH57x_common.h"
#include "devinfoservice.h"
#include "gattprofile.h"
#include "peripheral.h"
#include "HAL.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event  执行周期事件的频率
#define SBP_PERIODIC_EVT_PERIOD 1000

// How often to perform read rssi event 执行读取rssi事件的频率
#define SBP_READ_RSSI_EVT_PERIOD 3200

// Parameter update delay 							参数更新延迟
#define SBP_PARAM_UPDATE_DELAY 6400

// What is the advertising interval when device is discoverable (units of 625us, 80=50ms)     //广播时间间隔
#define DEFAULT_ADVERTISING_INTERVAL 80

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 20=25ms)
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 20

// Maximum connection interval (units of 1.25ms, 100=125ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 100

// Slave latency to use parameter update
#define DEFAULT_DESIRED_SLAVE_LATENCY 0

// Supervision timeout value (units of 10ms, 100=1s)
#define DEFAULT_DESIRED_CONN_TIMEOUT 100

// Company Identifier: WCH
#define WCH_COMPANY_ID 0x07D7

#define INFO_FIRST 0xaa
#define INFO_SEC 0x55

u8 DeviceID[20] = "00010000100026000005"; //设备号ID 00010000100026000005
uint16 m = 0;
uint16 Number_Records = 0;
int8 rssi_t = 0;
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 Peripheral_TaskID = INVALID_TASK_ID; // Task ID for internal task/event processing

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
    {
        // complete name
        0x0A, // length of this data
        GAP_ADTYPE_LOCAL_NAME_COMPLETE,
        'X', 'i', 'a', 'o', 'W', 'e', 'n', 'l', 'g',
        // connection interval range
        0x05, // length of this data
        GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
        LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL), // 100ms
        HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
        LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL), // 1s
        HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

        // Tx power level
        0x02, // length of this data
        GAP_ADTYPE_POWER_LEVEL,
        0 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
    {
        // Flags; this sets the device to use limited discoverable
        // mode (advertises for 30 seconds at a time) instead of general
        // discoverable mode (advertises indefinitely)
        0x02, // length of this data
        GAP_ADTYPE_FLAGS,
        DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

        // service UUID, to notify central devices what services are included
        // in this peripheral
        0x11,                   // length of this data
        GAP_ADTYPE_128BIT_MORE, // some of the UUID's, but not all
        0x00, 0x03, 0xcd, 0xd0, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Peripheral";

// Connection item list
static peripheralConnItem_t peripheralConnList;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent);
static void performPeriodicTask(void);
static void simpleProfileChangeCB(uint8 paramID, uint8 len);
static void peripheralParamUpdateCB(uint16 connHandle, uint16 connInterval,
                                    uint16 connSlaveLatency, uint16 connTimeout);
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList);
static void peripheralRssiCB(uint16 connHandle, int8 rssi);
static void peripheralChar4Notify(uint8 *pValue, uint16 len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t Peripheral_PeripheralCBs =
    {
        peripheralStateNotificationCB, // Profile State Change Callbacks             状态改变回调函数
        peripheralRssiCB,              // When a valid RSSI is read from controller (not used by application)  //RSSI更新回调函数
        peripheralParamUpdateCB        //参数更新完成回调函数
};

// Broadcast Callbacks
static gapRolesBroadcasterCBs_t Broadcaster_BroadcasterCBs =
    {
        NULL, // Not used in peripheral role
        NULL  // Receive scan request callback
};

// GAP Bond Manager Callbacks
static gapBondCBs_t Peripheral_BondMgrCBs =
    {
        NULL, // Passcode callback (not used by application)
        NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t Peripheral_SimpleProfileCBs =
    {
        simpleProfileChangeCB // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Peripheral_Init
 *
 * @brief   Initialization function for the Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by TMOS.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void Peripheral_Init()
{
  Peripheral_TaskID = TMOS_ProcessEventRegister(Peripheral_ProcessEvent); //注册任务处理回调函数             //蓝牙外设任务处理回调函数

  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;

    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable); //设置是否广播
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);            //设置被扫描到，模块要发送的响应数据
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);                //广播的数据
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);   //最小连接间隔
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);   //最大连接间隔
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 bonding = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    GAPBondMgr_SetParameter(GAPBOND_PERI_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PERI_PAIRING_MODE, sizeof(uint8), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_PERI_MITM_PROTECTION, sizeof(uint8), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_PERI_IO_CAPABILITIES, sizeof(uint8), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_PERI_BONDING_ENABLED, sizeof(uint8), &bonding);
  }

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the SimpleProfile Characteristic Values设置SimpleProfile特征值
  {
    uint8 charValue1[SIMPLEPROFILE_CHAR1_LEN] = {1};
    uint8 charValue2[SIMPLEPROFILE_CHAR2_LEN] = {2};
    uint8 charValue3[SIMPLEPROFILE_CHAR3_LEN] = {3};
    uint8 charValue4[SIMPLEPROFILE_CHAR4_LEN] = {4};
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = {1, 2, 3, 4, 5};

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN, charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, SIMPLEPROFILE_CHAR2_LEN, charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, SIMPLEPROFILE_CHAR3_LEN, charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5);
  }

  // Init Connection Item
  peripheralInitConnItem(&peripheralConnList);

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&Peripheral_SimpleProfileCBs);

  // Register receive scan request callback
  GAPRole_BroadcasterSetCB(&Broadcaster_BroadcasterCBs);

  // Setup a delayed profile startup
  tmos_set_event(Peripheral_TaskID, SBP_START_DEVICE_EVT);
}

/*********************************************************************
 * @fn      peripheralInitConnItem
 *
 * @brief   Init Connection Item
 *
 * @param   peripheralConnList -
 *
 * @return  NULL
 */
static void peripheralInitConnItem(peripheralConnItem_t *peripheralConnList)
{
  peripheralConnList->connHandle = GAP_CONNHANDLE_INIT;
  peripheralConnList->connInterval = 0;
  peripheralConnList->connSlaveLatency = 0;
  peripheralConnList->connTimeout = 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessEvent
 *
 * @brief   Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id - The TMOS assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 Peripheral_ProcessEvent(uint8 task_id, uint16 events)
{

  //  VOID task_id; // TMOS required parameter that isn't used in this function

  if (events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;

    if ((pMsg = tmos_msg_receive(Peripheral_TaskID)) != NULL)
    {
      Peripheral_ProcessTMOSMsg((tmos_event_hdr_t *)pMsg);
      // Release the TMOS message
      tmos_msg_deallocate(pMsg);
    }
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if (events & SBP_START_DEVICE_EVT)
  { //设备事件处理
    // Start the Device
    GAPRole_PeripheralStartDevice(Peripheral_TaskID, &Peripheral_BondMgrCBs, &Peripheral_PeripheralCBs);
    return (events ^ SBP_START_DEVICE_EVT);
  }

  if (events & SBP_PERIODIC_EVT)
  {
    // Restart timer
    if (SBP_PERIODIC_EVT_PERIOD)
    {
      tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);
    }
    // Perform periodic application task
    performPeriodicTask();
    return (events ^ SBP_PERIODIC_EVT);
  }
  //Uart Event
  //  if(events &SBP_PARAM_UART)
  //	{
  //		if(SBP_PERIODIC_EVT_PERIOD)
  //		{
  //			tmos_start_task( Peripheral_TaskID,SBP_PARAM_UART, SBP_PERIODIC_EVT_PERIOD*10 );
  //		}
  //		PRINT("Hellow Task!");
  //		return (events ^ SBP_PARAM_UART);
  //	}
  if (events & SBP_PARAM_UPDATE_EVT)
  {
    // Send connect param update request
    GAPRole_PeripheralConnParamUpdateReq(peripheralConnList.connHandle,
                                         DEFAULT_DESIRED_MIN_CONN_INTERVAL,
                                         DEFAULT_DESIRED_MAX_CONN_INTERVAL,
                                         DEFAULT_DESIRED_SLAVE_LATENCY,
                                         DEFAULT_DESIRED_CONN_TIMEOUT,
                                         Peripheral_TaskID);

    return (events ^ SBP_PARAM_UPDATE_EVT);
  }

  if (events & SBP_READ_RSSI_EVT)
  {
    GAPRole_ReadRssiCmd(peripheralConnList.connHandle);
    tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);
    return (events ^ SBP_READ_RSSI_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      Peripheral_ProcessTMOSMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void Peripheral_ProcessTMOSMsg(tmos_event_hdr_t *pMsg)
{
  switch (pMsg->event)
  {
  default:
    break;
  }
}

/*********************************************************************
 * @fn      Peripheral_LinkEstablished
 *
 * @brief   Process link established.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkEstablished(gapRoleEvent_t *pEvent)
{
  gapEstLinkReqEvent_t *event = (gapEstLinkReqEvent_t *)pEvent;

  // See if already connected
  if (peripheralConnList.connHandle != GAP_CONNHANDLE_INIT)
  {
    GAPRole_TerminateLink(pEvent->linkCmpl.connectionHandle);
    PRINT("Connection max...\n");
  }
  else
  {
    peripheralConnList.connHandle = event->connectionHandle;
    peripheralConnList.connInterval = event->connInterval;
    peripheralConnList.connSlaveLatency = event->connLatency;
    peripheralConnList.connTimeout = event->connTimeout;

    // Set timer for periodic event
    tmos_start_task(Peripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD);

    // Set timer for param update event
    tmos_start_task(Peripheral_TaskID, SBP_PARAM_UPDATE_EVT, SBP_PARAM_UPDATE_DELAY);

    // Start read rssi
    tmos_start_task(Peripheral_TaskID, SBP_READ_RSSI_EVT, SBP_READ_RSSI_EVT_PERIOD);

    //Uart************
    //tmos_start_task( Peripheral_TaskID, SBP_PARAM_UART, SBP_PERIODIC_EVT_PERIOD*10 );

    PRINT("Conn %x - Int %x \n", event->connectionHandle, event->connInterval);
  }
}

/*********************************************************************
 * @fn      Peripheral_LinkTerminated
 *
 * @brief   Process link terminated.
 *
 * @param   pEvent - event to process
 *
 * @return  none
 */
static void Peripheral_LinkTerminated(gapRoleEvent_t *pEvent)
{
  gapTerminateLinkEvent_t *event = (gapTerminateLinkEvent_t *)pEvent;

  if (event->connectionHandle == peripheralConnList.connHandle)
  {
    peripheralConnList.connHandle = GAP_CONNHANDLE_INIT;
    peripheralConnList.connInterval = 0;
    peripheralConnList.connSlaveLatency = 0;
    peripheralConnList.connTimeout = 0;
    tmos_stop_task(Peripheral_TaskID, SBP_PERIODIC_EVT);
    tmos_stop_task(Peripheral_TaskID, SBP_READ_RSSI_EVT);
    tmos_stop_task(Peripheral_TaskID, SBP_PARAM_UART); //  Test*********************

    // Restart advertising
    {
      uint8 advertising_enable = TRUE;
      GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertising_enable);
    }
  }
  else
  {
    PRINT("ERR..\n");
  }
}

/*********************************************************************
 * @fn      peripheralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void peripheralRssiCB(uint16 connHandle, int8 rssi)
{
  //PRINT("RSSI -%d dB Conn  %x \n", -rssi, connHandle); //取得RSSI信号强度 和连接句柄
  rssi_t = rssi;
}

/*********************************************************************
 * @fn      peripheralParamUpdateCB
 *
 * @brief   Parameter update complete callback
 *
 * @param   connHandle - connect handle
 *          connInterval - connect interval
 *          connSlaveLatency - connect slave latency
 *          connTimeout - connect timeout
 *          
 * @return  none
 */
static void peripheralParamUpdateCB(uint16 connHandle, uint16 connInterval,
                                    uint16 connSlaveLatency, uint16 connTimeout)
{
  if (connHandle == peripheralConnList.connHandle)
  {
    peripheralConnList.connInterval = connInterval;
    peripheralConnList.connSlaveLatency = connSlaveLatency;
    peripheralConnList.connTimeout = connTimeout;

    PRINT("Update %x - Int %x \n", connHandle, connInterval);
  }
  else
  {
    PRINT("ERR..\n");
  }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.         //处理状态改变的回调函数
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB(gapRole_States_t newState, gapRoleEvent_t *pEvent)
{
  switch (newState)
  {
  case GAPROLE_STARTED: //开始但未广播
    PRINT("Initialized..\n");
    break;

  case GAPROLE_ADVERTISING: //现在广播
    if (pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
    {
      Peripheral_LinkTerminated(pEvent);
      PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
    }
    PRINT("Advertising..\n");
    break;

  case GAPROLE_CONNECTED: //连接成功
    if (pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
    {
      Peripheral_LinkEstablished(pEvent);
    }
    PRINT("Connected..\n");
    break;

  case GAPROLE_CONNECTED_ADV:
    PRINT("Connected Advertising..\n");
    break;

  case GAPROLE_WAITING:
    if (pEvent->gap.opcode == GAP_END_DISCOVERABLE_DONE_EVENT)
    {
      PRINT("Waiting for advertising..\n");
    }
    else if (pEvent->gap.opcode == GAP_LINK_TERMINATED_EVENT)
    {
      Peripheral_LinkTerminated(pEvent);
      PRINT("Disconnected.. Reason:%x\n", pEvent->linkTerminate.reason);
    }
    else if (pEvent->gap.opcode == GAP_LINK_ESTABLISHED_EVENT)
    {
      if (pEvent->gap.hdr.status != SUCCESS)
      {
        PRINT("Waiting for advertising..\n");
      }
      else
      {
        PRINT("Error..\n");
      }
    }
    else
    {
      PRINT("Error..%x\n", pEvent->gap.opcode);
    }
    break;

  case GAPROLE_ERROR:
    PRINT("Error..\n");
    break;

  default:
    break;
  }
}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          TMOS event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask(void)
{
  //uint8 index = 0;
  uint8 i = 0;
  uint8 notiData[SIMPLEPROFILE_CHAR4_LEN] = {0};
  m++;
  //发送ID的前10位                                        -----1
  notiData[0] = INFO_FIRST;
  notiData[1] = INFO_SEC;
  notiData[2] = 1;
  for (i = 3; i < 13; i++)
  {
    notiData[i] = DeviceID[i - 3];
  }
  notiData[19] = 0xf6;
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);

  //发送后10位																						---2
  notiData[2] = 2;
  for (i = 3; i < 13; i++)
  {
    notiData[i] = DeviceID[i + 7];
  }
  notiData[19] = 0xf6;
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);

  //运动数据																						----3
  notiData[2] = 3;
  for (i = 3; i < 20; i++)
  {
    notiData[i] = 0;
  }
  notiData[3] = (m * 60) >> 8;
  notiData[4] = (m * 60) & 0x00ff;
  notiData[5] = Number_Records >> 8;
  notiData[6] = Number_Records & 0x00ff;
  notiData[7] = Number_Records * 2 >> 8;
  notiData[8] = Number_Records * 2 & 0x00ff;
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
  //4
  notiData[2] = 4;
  for (i = 3; i < 20; i++)
  {
    notiData[i] = 0;
  }
  notiData[7] = Number_Records >> 8;
  notiData[8] = Number_Records & 0x00ff;
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
  //5
  notiData[2] = 5;
  for (i = 3; i < 20; i++)
  {
    notiData[i] = 0;
  }
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
  //6
  notiData[2] = 0;
  notiData[19] = 0xff;
  peripheralChar4Notify(notiData, SIMPLEPROFILE_CHAR4_LEN);
}

/*********************************************************************
 * @fn      peripheralChar4Notify
 *
 * @brief   Prepare and send simpleProfileChar4 notification
 *
 * @param   pValue - data to notify
 *          len - length of data
 *
 * @return  none
 */
static void peripheralChar4Notify(uint8 *pValue, uint16 len)
{
  attHandleValueNoti_t noti;
  noti.len = len;
  noti.pValue = GATT_bm_alloc(peripheralConnList.connHandle, ATT_HANDLE_VALUE_NOTI, noti.len, NULL, 0); //开辟一块内存
  tmos_memcpy(noti.pValue, pValue, noti.len);
  if (simpleProfile_Notify(peripheralConnList.connHandle, &noti) != SUCCESS)
  {
    GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
  }
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
//len ： 信息长度
static void simpleProfileChangeCB(uint8 paramID, uint8 len)
{

  switch (paramID)
  {
  case SIMPLEPROFILE_CHAR1:
  {
    uint8 newValue[SIMPLEPROFILE_CHAR1_LEN];
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, newValue);
    PRINT("profile ChangeCB CHAR1.. \n");
    break;
  }

  case SIMPLEPROFILE_CHAR3:
  {
    uint8 i;
    uint8 newValue[SIMPLEPROFILE_CHAR3_LEN];
    SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, newValue);
    //PRINT("profile ChangeCB CHAR3..==%s\n",newValue);
    PRINT("Receive DataLen%d:\n", len);
    PRINT("Receive Data:");
    for (i = 0; i < len; i++)
    {
      PRINT("%d", newValue[i]);
    }
    PRINT("\n");
    break;
  }

  default:
    // should not reach here!
    break;
  }
}

/*********************************************************************
*********************************************************************/
