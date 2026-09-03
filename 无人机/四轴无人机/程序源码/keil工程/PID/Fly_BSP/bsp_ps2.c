/**
 * ============================================================
 * bsp_ps2.c — PS2无线手柄USB-HID驱动
 * ============================================================
 * 功能:
 *   1. USB HID设备识别与注册 (有线/无线PC/无线Android三种模式)
 *   2. PS2手柄数据解码: 按键位掩码 + 4模拟摇杆 (8bit ADC)
 *   3. 按键事件状态机: 单击/双击/长按识别 (基于计时计数器)
 *
 * 数据流: USB → HID Report → PS2_Decode → ps2_info 结构体
 * 接口: PS2Interface_t { getKeyState, getKeyEvent, LX, LY, RX, RY }
 * ============================================================
 */

#include "bsp_ps2.h"
#include "usbh_hid.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "bsp_led.h"
#include "bsp_buzzer.h"

//软件定时器,用于usb插入与拔出的提示
static TimerHandle_t TimerUSBinsert = NULL;
static TimerHandle_t TimerUSBunplugged = NULL;

//PS2手柄类型
static PS2_TYPE_t ps2_type = UnKnown_Dev;

//PS2 16个按键数值读取
static uint16_t ps2_KeyVal = 0;

extern PS2KEY_State_t ps2_checkkey(uint8_t bit);
extern uint8_t ps2_checkkeystate(uint8_t bit);

//usb手柄数据结构体
PS2INFO_t ps2_info = { 
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.getKeyEvent = ps2_checkkey,
	.getKeyState = ps2_checkkeystate
};

//ps2手柄的默认值,当手柄出现插拔时恢复默认值
PS2INFO_t ps2_defaultVal = {
	.LX = 127,
	.LY = 127,
	.RX = 127,
	.RY = 127,
	.getKeyEvent = ps2_checkkey,
	.getKeyState = ps2_checkkeystate
};

//PS2_HID类相关函数声明
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

//usb手柄的初始化声明
extern USBH_StatusTypeDef USBH_HID_PS2Init(USBH_HandleTypeDef *phost);

//定义ps2的hid类
USBH_ClassTypeDef  PS2_HID_Class =
{
  .Name = "HID",
  .ClassCode = USB_HID_CLASS, //有线ps2手柄,默认为HID设备
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};

USBH_ClassTypeDef  WiredlessPS2_HID_Class =
{
  .Name = "HID",
  .ClassCode = 0xff, //无线ps2手柄,安卓模式下,枚举时该类返回的代码为0xff
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};


/**
  * @brief  USBH_HID_InterfaceInit
  *         The function init the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  HID_HandleTypeDef *HID_Handle;
  uint16_t ep_mps;
  uint8_t max_ep;
  uint8_t num = 0U;
  uint8_t interface;

	USBH_UsrLog("start find interface now");
	
	//接口匹配,主要对下面的这个数组接口进行匹配
	//phost->device.CfgDesc.Itf_Desc[0,1,2,3...,n...].bInterfaceSubClass

	//0xff表示匹配所有类型
	interface = USBH_FindInterface(phost, 0xFFU, 0xFFU, 0xFFU);
	
	if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) /* No Valid Interface */
	{
		USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
		return USBH_FAIL;
	}

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
  HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for HID Handle");
    return USBH_FAIL;
  }

  /* Initialize hid handler */
  (void)USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));

  HID_Handle->state = USBH_HID_ERROR;

  //根据HID设备的 PID、VID进行不同的设备类型识别
  if( phost->device.DevDesc.idProduct == Wired_PS2_PID && phost->device.DevDesc.idVendor==Wired_PS2_VID )
  {	  //有线PS2手柄的PID/VID
	  USBH_UsrLog("Wired PS2 device found!");
	  ps2_type = Wired_PS2;
	  HID_Handle->Init = USBH_HID_PS2Init;
  }
  
  else if( phost->device.DevDesc.idProduct == Wireless_PC_PS2_PID && phost->device.DevDesc.idVendor==Wireless_PC_PS2_VID )
  {	  //无线PS2手柄 PC模式 PID/VID
	  USBH_UsrLog("Wireless PC PS2 device found!");
	  ps2_type = Wiredless_PC_PS2;
	  HID_Handle->Init = USBH_HID_PS2Init;
  }
  
  else if( phost->device.DevDesc.idProduct == Wireless_Android_PS2_PID && phost->device.DevDesc.idVendor==Wireless_Android_PS2_VID )
  {	  //无线PS2手柄 Android模式 的PID/VID
	  USBH_UsrLog("Wireless Android PS2 device found!");
	  ps2_type = Wiredless_Android_PS2;
	  HID_Handle->Init = USBH_HID_PS2Init;
  }
  
  else
  {
	ps2_type = UnKnown_Dev;//未知的设备类型
    USBH_UsrLog("Protocol not supported.");
    return USBH_FAIL;
  }

  HID_Handle->state     = USBH_HID_INIT;
  HID_Handle->ctl_state = USBH_HID_REQ_INIT;
  HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
  HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;
  
  if (HID_Handle->poll < HID_MIN_POLL)
  {
    HID_Handle->poll = HID_MIN_POLL;
  }

  /* Check of available number of endpoints */
  /* Find the number of EPs in the Interface Descriptor */
  /* Choose the lower number in order not to overrun the buffer allocated */
  max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
            phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);


  /* Decode endpoint IN and OUT address from interface descriptor */
  for (num = 0U; num < max_ep; num++)
  {
    if ((phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) != 0U)
    {
      HID_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for IN endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
    }
    else
    {
      HID_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->OutPipe = USBH_AllocPipe(phost, HID_Handle->OutEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for OUT endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
    }
  }
  return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceDeInit
  *         The function DeInit the Pipes used for the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->InPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->InPipe);
    (void)USBH_FreePipe(phost, HID_Handle->InPipe);
    HID_Handle->InPipe = 0U;     /* Reset the pipe as Free */
  }

  if (HID_Handle->OutPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->OutPipe);
    (void)USBH_FreePipe(phost, HID_Handle->OutPipe);
    HID_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
  }

  if ((phost->pActiveClass->pData) != NULL)
  {
    USBH_free(phost->pActiveClass->pData);
    phost->pActiveClass->pData = 0U;
  }

  //设备拔出时,将进行反初始化,ps2手柄数据也需要一起复位
  memcpy(&ps2_info,&ps2_defaultVal,sizeof(PS2INFO_t));
  
  //ps2手柄类型、按键值复位
  ps2_type = UnKnown_Dev;
  ps2_KeyVal = 0;
  
  //tips
 // xTimerStart(TimerUSBunplugged,portMAX_DELAY);
  
  return USBH_OK;
}

/**
  * @brief  USBH_HID_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{

  USBH_StatusTypeDef status         = USBH_BUSY;
  USBH_StatusTypeDef classReqStatus = USBH_BUSY;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  /* Switch HID state machine */
  switch (HID_Handle->ctl_state)
  {
    case USBH_HID_REQ_INIT:
    case USBH_HID_REQ_GET_HID_DESC:

      USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.CfgDesc_Raw);

      HID_Handle->ctl_state = USBH_HID_REQ_GET_REPORT_DESC;

      break;
    case USBH_HID_REQ_GET_REPORT_DESC:

      /* Get Report Desc */
      classReqStatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->HID_Desc.wItemLength);
      if (classReqStatus == USBH_OK)
      {
        /* The descriptor is available in phost->device.Data */
        HID_Handle->ctl_state = USBH_HID_REQ_SET_IDLE;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog("Control error: HID: Device Get Report Descriptor request failed");
        status = USBH_FAIL;
      }
      else
      {
        /* .. */
      }

      break;

    case USBH_HID_REQ_SET_IDLE:

      classReqStatus = USBH_HID_SetIdle(phost, 0U, 0U);

      /* set Idle */
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
      }
      else
      {
        if (classReqStatus == USBH_NOT_SUPPORTED)
        {
          HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
        }
      }
      break;

    case USBH_HID_REQ_SET_PROTOCOL:
      /* set protocol */
      classReqStatus = USBH_HID_SetProtocol(phost, 0U);
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_IDLE;

        /* all requests performed */
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        status = USBH_OK;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog("Control error: HID: Device Set protocol request failed");
        status = USBH_FAIL;
      }
      else
      {
        /* .. */
      }
      break;

    case USBH_HID_REQ_IDLE:
    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_Process
  *         The function is for managing state machine for HID data transfers
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_OK;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  uint32_t XferSize;

  switch (HID_Handle->state)
  {
    case USBH_HID_INIT:
      status = HID_Handle->Init(phost);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_IDLE;
      }
      else
      {
        USBH_ErrLog("HID Class Init failed");
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_IDLE:
		
	//发送GetReport请求，请求HID设备返回数据.在此之前 HID_Handle->pData 必须完成初始化,这一步通常在 HID_Handle->Init 函数完成,否则将无法请求成功
	
      status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_SYNC;
      }
      else if (status == USBH_BUSY)
      {
        HID_Handle->state = USBH_HID_IDLE;
        status = USBH_OK;
      }
      else if (status == USBH_NOT_SUPPORTED)
      {
        HID_Handle->state = USBH_HID_SYNC;
        status = USBH_OK;
      }
      else
      {
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_SYNC:
      /* Sync with start of Even Frame */
      if ((phost->Timer & 1U) != 0U)
      {
        HID_Handle->state = USBH_HID_GET_DATA;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_GET_DATA:
		
	//接收HID设备的数据,保存在 HID_Handle->pData
      (void)USBH_InterruptReceiveData(phost, HID_Handle->pData,
                                      (uint8_t)HID_Handle->length,
                                      HID_Handle->InPipe);

      HID_Handle->state = USBH_HID_POLL;
      HID_Handle->timer = phost->Timer;
      HID_Handle->DataReady = 0U;
      break;

    case USBH_HID_POLL:
      if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE)
      {
        XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);

        if ((HID_Handle->DataReady == 0U) && (XferSize != 0U) && (HID_Handle->fifo.buf != NULL))
        {
		  //将保存在HID_Handle->pData的HID设备数据写入到fifo中，后续通过fifo来获取hid设备的数据
          (void)USBH_HID_FifoWrite(&HID_Handle->fifo, HID_Handle->pData, HID_Handle->length); 
          HID_Handle->DataReady = 1U;
          USBH_HID_EventCallback(phost);

#if (USBH_USE_OS == 1U)
          phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
          (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
          (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
        }
      }
      else
      {
        /* IN Endpoint Stalled */
        if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL)
        {
          /* Issue Clear Feature on interrupt IN endpoint */
          if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK)
          {
            /* Change state to issue next IN token */
            HID_Handle->state = USBH_HID_GET_DATA;
          }
        }
      }
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_SOFProcess
  *         The function is for managing the SOF Process
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->state == USBH_HID_POLL)
  {
    if ((phost->Timer - HID_Handle->timer) >= HID_Handle->poll)
    {
      HID_Handle->state = USBH_HID_GET_DATA;

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
    }
  }
  return USBH_OK;
}

/**
  * @brief  USBH_ParseHIDDesc
  *         This function Parse the HID descriptor
  * @param  desc: HID Descriptor
  * @param  buf: Buffer where the source descriptor is available
  * @retval None
  */
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf)
{
  USBH_DescHeader_t *pdesc = (USBH_DescHeader_t *)buf;
  uint16_t CfgDescLen;
  uint16_t ptr;

  CfgDescLen = LE16(buf + 2U);

  if (CfgDescLen > USB_CONFIGURATION_DESC_SIZE)
  {
    ptr = USB_LEN_CFG_DESC;

    while (ptr < CfgDescLen)
    {
      pdesc = USBH_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_HID)
      {
        desc->bLength = *(uint8_t *)((uint8_t *)pdesc + 0U);
        desc->bDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 1U);
        desc->bcdHID = LE16((uint8_t *)pdesc + 2U);
        desc->bCountryCode = *(uint8_t *)((uint8_t *)pdesc + 4U);
        desc->bNumDescriptors = *(uint8_t *)((uint8_t *)pdesc + 5U);
        desc->bReportDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 6U);
        desc->wItemLength = LE16((uint8_t *)pdesc + 7U);
        break;
      }
    }
  }
}

///////////////////////// PS2 初始化实现、解码实现 ///////////////////////////////
//ps2手柄的输入报告
static uint8_t ps2_report_data[64] = { 0 }; //用于存放HID设备发送过来的数据

//usb插入
//#include "bsp_led.h"
static void timer_usb_inset_callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t beep = &UserBuzzer;
	beep->on();
	vTaskDelay(80);
	beep->off();
	vTaskDelay(50);
	beep->on();
	vTaskDelay(400);
	beep->off();
}

//usb拔出
static void timer_usb_pull_callback(TimerHandle_t xTimer)
{
	pBuzzerInterface_t beep = &UserBuzzer;
	beep->on();
	vTaskDelay(800);
	beep->off();
	vTaskDelay(50);
	beep->on();
	vTaskDelay(200);
	beep->off();
}

//ps2初始化函数
USBH_StatusTypeDef USBH_HID_PS2Init(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//HID_Handle->length在初始化设备时由HID设备确定
	if (HID_Handle->length > sizeof(ps2_report_data))
	{
		HID_Handle->length = (uint16_t)sizeof(ps2_report_data);
	}
	
	//初始化pData,必须步骤.
	HID_Handle->pData = ps2_report_data;
	
	if ((HID_QUEUE_SIZE * sizeof(ps2_report_data)) > sizeof(phost->device.Data))
	{	//sizeof(phost->device.Data) 大小由 USBH_MAX_DATA_BUFFER 配置
		return USBH_FAIL;
	}
	else
	{
		//初始化fifo
		USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(ps2_report_data)));
	}
	
	//该函数最终在  phost->pActiveClass->BgndProcess(phost); 执行，执行环境为任务内.可添加蜂鸣器提示插入
	//创建两个定时器
	if( TimerUSBinsert==NULL ) TimerUSBinsert = xTimerCreate("USBinTips",pdMS_TO_TICKS(10),pdFALSE,NULL,timer_usb_inset_callback);
	if( TimerUSBunplugged==NULL ) TimerUSBunplugged = xTimerCreate("USBoutTips",pdMS_TO_TICKS(10),pdFALSE,NULL,timer_usb_pull_callback);
	xTimerStart(TimerUSBinsert,portMAX_DELAY);
	
	return USBH_OK;
}

//3种模式的手柄数据解包函数
static void Wired_PS2_Decode(const uint8_t *data);
static void Wiredless_PC_PS2_Decode(const uint8_t *data);
static void Wiredless_Android_PS2_Decode(const uint8_t *data);

//ps2数据解码,入口参数为HID设备.运行环境为任务.
USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//检查是否成功识别到手柄类型
	if( ps2_type == UnKnown_Dev ) return USBH_FAIL;
	
	//检查hid设备是否有数据
	if ((HID_Handle->length == 0U) || (HID_Handle->fifo.buf == NULL))
	{
		return USBH_FAIL;
	}
	
	//从fifo中读取hid数据放入ps2_report_data，并进行解析
	if (USBH_HID_FifoRead(&HID_Handle->fifo, (uint8_t* )ps2_report_data, HID_Handle->length) == HID_Handle->length)
	{
		//根据不同的ps2设备执行不同的解码函数
		if(  Wired_PS2 == ps2_type )
		{
			Wired_PS2_Decode(ps2_report_data);
		}
		else if( Wiredless_PC_PS2 == ps2_type )
		{
			Wiredless_PC_PS2_Decode(ps2_report_data);
		}
		else if( Wiredless_Android_PS2 == ps2_type )
		{
			Wiredless_Android_PS2_Decode(ps2_report_data);
		}
		
		return USBH_OK;
	}
	return   USBH_FAIL;

}

////////////////// PS2按键检测函数 ///////////////
//16个ps2按键
#define PS2_KEY_NUM 16 



//按键检测时间,单位ms
#define PS2_LONGPRESS_TIEM 1000 //长按检测时间
#define PS2_CLICK_TIME     400  //单双击检测时间
#define PS2_KEYFILTER_TIME 50   //按键滤波时间

//状态机的状态值
typedef enum{
    WaitToPress = 0,
    WaitToRelease ,
    KEYPress    ,
    KEYUp       ,
    LONG_CLICK  ,
}PS2KEY_CheckState;

//检测按键按下的辅助变量
typedef struct 
{
    uint8_t keystate;       //按键的状态,0表示松开,1表示被按下
    uint32_t timebase;      //按键检测的时基
    uint32_t statetime;     //状态统计时间
    PS2KEY_CheckState statemachine; //检测按键的状态机
}PS2_CheckKey_t;

PS2_CheckKey_t ps2key[PS2_KEY_NUM] = { 0 };

PS2KEY_State_t ps2_checkkey(uint8_t bit)
{
    PS2_CheckKey_t* key = &ps2key[bit]; //指定要检测哪一个按键
    
    //读取对应的键值状态
    key->keystate = (ps2_KeyVal>>bit)&0x01;

    switch (key->statemachine)
    {
        case WaitToPress:
            if( PS2KEY_PressDOWN == key->keystate )
            {
                key->timebase = HAL_GetTick();
                key->statemachine = KEYPress;
            } 
            break;
        case KEYPress:
            //统计第一次按下按键以后的时间(无符号溢出时自然环绕.不需要溢出判断)
            key->statetime = HAL_GetTick() - key->timebase;

            //检查按键是否有松开
            if( PS2KEY_PressUP == key->keystate )
            {
                //单次按键按下的时间太短,忽略.作为滤波作用
                if( key->statetime < PS2_KEYFILTER_TIME ) key->statemachine = WaitToPress;
                else
                {
                   key->timebase = HAL_GetTick();//重新更新时基,用于下一个状态的检测
                   key->statemachine = KEYUp;    //按键按下一定时间,又弹起,进入下一个检测状态
                }
            }
            else if( key->statetime > PS2_LONGPRESS_TIEM ) 
            {   //按键未松开,且保持一定的时间,则为长按.
                key->statemachine = LONG_CLICK;
            }

            break;
        case KEYUp:
            //统计第一次按下按键以后的时间(无符号溢出时自然环绕.不需要溢出判断)
            key->statetime = HAL_GetTick() - key->timebase;

            if( PS2KEY_PressDOWN == key->keystate && key->statetime < PS2_CLICK_TIME && key->statetime > PS2_KEYFILTER_TIME )
            {
                key->statemachine = WaitToRelease;
                return PS2KEYSTATE_DOUBLECLICK;
            }
            else if( key->statetime >= PS2_CLICK_TIME )
            {
                key->statemachine = WaitToRelease;
                return PS2KEYSTATE_SINGLECLICK;
            }
            break;
        case LONG_CLICK:
            key->statemachine = WaitToRelease;
            return PS2KEYSTATE_LONGCLICK;
        case WaitToRelease:
            //按键检测完毕,等用户松开按键后,再恢复状态机的状态
            if( PS2KEY_PressUP == key->keystate ) key->statemachine = WaitToPress;
            break;
        default:
            break;
    }

    return PS2KEYSTATE_NONE;
}

//直接反馈按键的状态值
uint8_t ps2_checkkeystate(uint8_t bit)
{
    return (ps2_KeyVal>>bit)&0x01;
}

////////////////// PS2按键检测函数 END ///////////////

///////////////////////////////////////////////// 解码函数细节 ////////////////////////////////////////////////////////
//标志位设置函数,用于辅助ps2手柄解码
static void ps2_set_bit(uint16_t* state,uint8_t state_bit,uint8_t bit)
{
	if(state_bit==1) //指定的位(bit)设置为1,其他位不变
	{
		*state |= (1U<<bit);
	}
	else //指定的位(bit)设置为0,其他位不变
	{
		*state &= ~(1U<<bit);
	}
}

//有线PS2手柄的数据解码
static void Wired_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0 ;
	
	ps2_info.LX = data[3];
	ps2_info.LY = data[4];
	ps2_info.RX = data[1];
	ps2_info.RY = data[2];
	
	tmp_bool = (data[6]>>4)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,0); //seltec key 选择按键
	
	tmp_bool = (data[6]>>6)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,1); //左摇杆按键
	
	tmp_bool = (data[6]>>7)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,2); //右摇杆按键
	
	tmp_bool = (data[6]>>5)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,3); //start
	
	tmp_bool = data[5]&0x0F;//取出低4位
	if(tmp_bool==0x0F)//没有任何按键按下
	{
		ps2_set_bit(&ps2_KeyVal,0,4); //↑
		ps2_set_bit(&ps2_KeyVal,0,5); //→
		ps2_set_bit(&ps2_KeyVal,0,6); //↓
		ps2_set_bit(&ps2_KeyVal,0,7); //←
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://↑
				ps2_set_bit(&ps2_KeyVal,1,4); //↑
				break;
			case 0x01://→
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x02://↓
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				break;
			case 0x03://←
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //首位为1,代表存在左盘2个按键按下的情况
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://↑→
				ps2_set_bit(&ps2_KeyVal,1,4);//↑
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x01://↓→
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x02://↓←
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			case 0x03://↑←
				ps2_set_bit(&ps2_KeyVal,1,4); //↑
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			default:
				break;
		}
	}
	
	tmp_bool = (data[6]>>2)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,8); //右扳机2号
	
	tmp_bool = (data[6]>>3)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,9); //右扳机2号
	
	tmp_bool = (data[6]>>0)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,10); //左扳机1号
	
	tmp_bool = (data[6]>>1)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,11); //右扳机1号
	
	tmp_bool = (data[5]>>4)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,12); //一号,绿色GREEN
	
	tmp_bool = (data[5]>>5)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,13); //二号,红色RED

	tmp_bool = (data[5]>>6)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,14); //三号,蓝牙BLUE
	
	tmp_bool = (data[5]>>7)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,15); //四号,粉色PINK
}

//无线安卓模式手柄数据解码
static void Wiredless_Android_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0 ;
	
	uint8_t rm_val = 0;
	if( data[6]==0&&data[7]==0 ) rm_val=128;
	else rm_val = data[6];
	ps2_info.LX = rm_val;
	
	if( data[8]==0&&data[9]==0  ) rm_val=128;
	else rm_val = data[8];
	ps2_info.LY = 255 - rm_val;
	
	if( data[10]==0&&data[11]==0  ) rm_val=128;
	else rm_val = data[10];
	ps2_info.RX = rm_val;
	
	if( data[12]==0&&data[13]==0  ) rm_val=128;
	else rm_val = data[12];
	ps2_info.RY = 255 - rm_val;
	
	//data[2]
	//Rm    Lm    select   start    →      ←       ↓        ↑
	//0		0		0		0		0		0		0		0
	tmp_bool = (data[2]>>0)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,4); //↑
	
	tmp_bool = (data[2]>>3)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,5); //→
	
	tmp_bool = (data[2]>>1)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,6); //↓
	
	tmp_bool = (data[2]>>2)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,7); //←
	
	tmp_bool = (data[2]>>5)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,0); //seltec key 选择按键	
	
	tmp_bool = (data[2]>>4)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,3); //start key 选择按键
	
	tmp_bool = (data[2]>>6)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,1); //左摇杆按键	
	
	tmp_bool = (data[2]>>7)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,2); //右摇杆按键
	
	tmp_bool = (data[3]>>0)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,10); //左扳机1号
	
	tmp_bool = (data[3]>>1)&0x01;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,11); //右扳机1号
	
	if(data[4]==0xff) tmp_bool=1;
	else tmp_bool=0;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,8); //右扳机2号
	
	if(data[5]==0xff) tmp_bool=1;
	else tmp_bool=0;
	ps2_set_bit(&ps2_KeyVal,tmp_bool,9); //右扳机2号
	
	tmp_bool = (data[3]>>4)&0x01;//BLUE
	ps2_set_bit(&ps2_KeyVal,tmp_bool,14);
	
	tmp_bool = (data[3]>>5)&0x01;//RED
	ps2_set_bit(&ps2_KeyVal,tmp_bool,13);
	
	tmp_bool = (data[3]>>6)&0x01;//PINK
	ps2_set_bit(&ps2_KeyVal,tmp_bool,15);
	
	tmp_bool = (data[3]>>7)&0x01;//GREEN
	ps2_set_bit(&ps2_KeyVal,tmp_bool,12);
}

//无线pc模式手柄数据解码
static void  Wiredless_PC_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0;
	
	ps2_info.LX = data[3];
	ps2_info.LY = data[4];
	ps2_info.RX = data[5];
	ps2_info.RY = data[6];
	
	tmp_bool = (data[1]>>0)&0x01;//select
	ps2_set_bit(&ps2_KeyVal,tmp_bool,0); //seltec key 选择按键	
	
	tmp_bool = (data[1]>>1)&0x01;//start
	ps2_set_bit(&ps2_KeyVal,tmp_bool,3); //start key 选择按键
	
	tmp_bool = (data[1]>>2)&0x01;//Lm
	ps2_set_bit(&ps2_KeyVal,tmp_bool,1); //左摇杆按键	
	
	tmp_bool = (data[1]>>3)&0x01;//Rm
	ps2_set_bit(&ps2_KeyVal,tmp_bool,2); //右摇杆按键
	
	tmp_bool = (data[0]>>4)&0x01;//L1
	ps2_set_bit(&ps2_KeyVal,tmp_bool,10); //左扳机1号
	
	tmp_bool = (data[0]>>5)&0x01;//R1
	ps2_set_bit(&ps2_KeyVal,tmp_bool,11); //右扳机1号
	
	tmp_bool = (data[0]>>6)&0x01;//L2
	ps2_set_bit(&ps2_KeyVal,tmp_bool,8); //右扳机2号
	
	tmp_bool = (data[0]>>7)&0x01;//R2
	ps2_set_bit(&ps2_KeyVal,tmp_bool,9); //右扳机2号
	
	tmp_bool = (data[0]>>0)&0x01;//GREEN
	ps2_set_bit(&ps2_KeyVal,tmp_bool,12);
	
	tmp_bool = (data[0]>>1)&0x01;//RED
	ps2_set_bit(&ps2_KeyVal,tmp_bool,13);
	
	tmp_bool = (data[0]>>2)&0x01;//BLUE
	ps2_set_bit(&ps2_KeyVal,tmp_bool,14);
	
	tmp_bool = (data[0]>>3)&0x01;//PINK
	ps2_set_bit(&ps2_KeyVal,tmp_bool,15);
	
	tmp_bool = data[2]&0x0F;//取出低4位
	if(tmp_bool==0x0F)//没有任何按键按下
	{
		ps2_set_bit(&ps2_KeyVal,0,4); //↑
		ps2_set_bit(&ps2_KeyVal,0,5); //→
		ps2_set_bit(&ps2_KeyVal,0,6); //↓
		ps2_set_bit(&ps2_KeyVal,0,7); //←
	}
	else if( (tmp_bool&0x01)==0 )
	{	
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://↑
				ps2_set_bit(&ps2_KeyVal,1,4); //↑
				break;
			case 0x01://→
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x02://↓
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				break;
			case 0x03://←
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			default:
				break;
		}
	}
	else if( (tmp_bool&0x01)==1 ) //首位为1,代表存在左盘2个按键按下的情况
	{
		switch ((tmp_bool>>1)&0x03)
		{
			case 0x00://↑→
				ps2_set_bit(&ps2_KeyVal,1,4);//↑
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x01://↓→
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				ps2_set_bit(&ps2_KeyVal,1,5); //→
				break;
			case 0x02://↓←
				ps2_set_bit(&ps2_KeyVal,1,6); //↓
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			case 0x03://↑←
				ps2_set_bit(&ps2_KeyVal,1,4); //↑
				ps2_set_bit(&ps2_KeyVal,1,7); //←
				break;
			default:
				break;
		}
	}	
}
///////////////////////////////////////////////// 解码函数细节 END ////////////////////////////////////////////////////////
