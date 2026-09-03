/**
 * @file    bsp_ps2.c
 * @brief   PS2 手柄 USB HID 驱动（支持有线/无线 PC/无线 Android 三种模式）
 * @note    基于 STM32 USB Host 库的 HID 类实现
 *          识别三种 PS2 设备: Wired PS2 (PID/VID), Wireless PC, Wireless Android
 *          16 按键 + 双摇杆 (LX, LY, RX, RY)，含按键状态机（单击/双击/长按识别）
 *          蜂鸣器提示 USB 插入/拔出
 */

#include "bsp_ps2.h"
#include "usbh_hid.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"

/* USB 插入/拔出提示定时器 -----------------------------------------------------*/
static TimerHandle_t TimerUSBinsert   = NULL;   /* USB 插入蜂鸣提示 */
static TimerHandle_t TimerUSBunplugged = NULL;   /* USB 拔出蜂鸣提示 */

/* PS2 手柄类型 ---------------------------------------------------------------*/
static PS2_TYPE_t ps2_type = UnKnown_Dev;        /* 当前识别的手柄类型 */

/* PS2 16 按键状态值（bit[N]=1 表示第 N 键按下） */
static uint16_t ps2_KeyVal = 0;

/* 外部按键检测函数（ps2_task.c 中实现） */
extern PS2KEY_State_t ps2_checkkey(uint8_t bit);
extern uint8_t ps2_checkkeystate(uint8_t bit);

/* PS2 手柄信息结构体（全局可访问） --------------------------------------------*/
PS2INFO_t ps2_info = {
	.LX = 127,                               /* 左摇杆 X 轴，默认中值 127 */
	.LY = 127,                               /* 左摇杆 Y 轴 */
	.RX = 127,                               /* 右摇杆 X 轴 */
	.RY = 127,                               /* 右摇杆 Y 轴 */
	.getKeyEvent = ps2_checkkey,             /* 按键事件检测（单击/双击/长按） */
	.getKeyState = ps2_checkkeystate         /* 按键实时状态检测 */
};

/* PS2 手柄默认值（手柄拔出时恢复此值） */
PS2INFO_t ps2_defaultVal = {
	.LX = 127, .LY = 127, .RX = 127, .RY = 127,
	.getKeyEvent = ps2_checkkey,
	.getKeyState = ps2_checkkeystate
};

/* HID 类回调函数前置声明 */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

/* 外部 PS2 初始化函数 */
extern USBH_StatusTypeDef USBH_HID_PS2Init(USBH_HandleTypeDef *phost);

/* PS2 有线手柄 HID 类注册 ----------------------------------------------------*/
USBH_ClassTypeDef PS2_HID_Class =
{
	.Name       = "HID",
	.ClassCode  = USB_HID_CLASS,             /* 标准 HID 类代码 */
	.Init       = USBH_HID_InterfaceInit,
	.DeInit     = USBH_HID_InterfaceDeInit,
	.Requests   = USBH_HID_ClassRequest,
	.BgndProcess = USBH_HID_Process,
	.SOFProcess  = USBH_HID_SOFProcess,
	.pData      = NULL,
};

/* PS2 无线手柄 HID 类注册（类代码 0xFF，兼容加密狗模式）-----------------------*/
USBH_ClassTypeDef WiredlessPS2_HID_Class =
{
	.Name       = "HID",
	.ClassCode  = 0xff,                      /* 加密狗模式下枚举返回 0xFF */
	.Init       = USBH_HID_InterfaceInit,
	.DeInit     = USBH_HID_InterfaceDeInit,
	.Requests   = USBH_HID_ClassRequest,
	.BgndProcess = USBH_HID_Process,
	.SOFProcess  = USBH_HID_SOFProcess,
	.pData      = NULL,
};

/**
 * @brief  USBH_HID_InterfaceInit - HID 类接口初始化
 * @param  phost: USB Host 句柄
 * @retval USBH_OK / USBH_FAIL
 * @note   匹配接口 → 根据 PID/VID 识别设备类型 → 分配 HID Handle → 打开 IN/OUT 管道
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

	/* 匹配任意接口（0xFF 表示匹配所有子类和协议） */
	interface = USBH_FindInterface(phost, 0xFFU, 0xFFU, 0xFFU);

	if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES))
	{
		USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
		return USBH_FAIL;
	}

	status = USBH_SelectInterface(phost, interface);
	if (status != USBH_OK) return USBH_FAIL;

	/* 分配 HID Handle 内存 */
	phost->pActiveClass->pData = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
	HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

	if (HID_Handle == NULL)
	{
		USBH_DbgLog("Cannot allocate memory for HID Handle");
		return USBH_FAIL;
	}

	/* 初始化 HID Handler */
	(void)USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));
	HID_Handle->state = USBH_HID_ERROR;

	/* 根据 PID/VID 识别 PS2 设备类型 */
	if (phost->device.DevDesc.idProduct == Wired_PS2_PID
	    && phost->device.DevDesc.idVendor == Wired_PS2_VID)
	{
		USBH_UsrLog("Wired PS2 device found!");
		ps2_type = Wired_PS2;
		HID_Handle->Init = USBH_HID_PS2Init;
	}
	else if (phost->device.DevDesc.idProduct == Wireless_PC_PS2_PID
	         && phost->device.DevDesc.idVendor == Wireless_PC_PS2_VID)
	{
		USBH_UsrLog("Wireless PC PS2 device found!");
		ps2_type = Wiredless_PC_PS2;
		HID_Handle->Init = USBH_HID_PS2Init;
	}
	else if (phost->device.DevDesc.idProduct == Wireless_Android_PS2_PID
	         && phost->device.DevDesc.idVendor == Wireless_Android_PS2_VID)
	{
		USBH_UsrLog("Wireless Android PS2 device found!");
		ps2_type = Wiredless_Android_PS2;
		HID_Handle->Init = USBH_HID_PS2Init;
	}
	else
	{
		ps2_type = UnKnown_Dev;
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

	/* 获取端点数（不超过最大值） */
	max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
	          phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);

	/* 解码并打开 IN/OUT 端点管道 */
	for (num = 0U; num < max_ep; num++)
	{
		if ((phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) != 0U)
		{
			/* IN 端点（设备 → 主机） */
			HID_Handle->InEp   = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
			HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);
			ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

			(void)USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
			                    phost->device.speed, USB_EP_TYPE_INTR, ep_mps);
			(void)USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
		}
		else
		{
			/* OUT 端点（主机 → 设备） */
			HID_Handle->OutEp   = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
			HID_Handle->OutPipe = USBH_AllocPipe(phost, HID_Handle->OutEp);
			ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

			(void)USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
			                    phost->device.speed, USB_EP_TYPE_INTR, ep_mps);
			(void)USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
		}
	}
	return USBH_OK;
}

/**
 * @brief  USBH_HID_InterfaceDeInit - HID 类接口反初始化
 * @param  phost: USB Host 句柄
 * @retval USBH_OK
 * @note   关闭管道、释放内存、恢复 PS2 手柄默认值
 */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

	if (HID_Handle->InPipe != 0x00U)
	{
		(void)USBH_ClosePipe(phost, HID_Handle->InPipe);
		(void)USBH_FreePipe(phost, HID_Handle->InPipe);
		HID_Handle->InPipe = 0U;
	}

	if (HID_Handle->OutPipe != 0x00U)
	{
		(void)USBH_ClosePipe(phost, HID_Handle->OutPipe);
		(void)USBH_FreePipe(phost, HID_Handle->OutPipe);
		HID_Handle->OutPipe = 0U;
	}

	if ((phost->pActiveClass->pData) != NULL)
	{
		USBH_free(phost->pActiveClass->pData);
		phost->pActiveClass->pData = 0U;
	}

	/* 设备拔出时恢复默认值 */
	memcpy(&ps2_info, &ps2_defaultVal, sizeof(PS2INFO_t));

	/* 复位手柄类型和按键值 */
	ps2_type  = UnKnown_Dev;
	ps2_KeyVal = 0;

	// xTimerStart(TimerUSBunplugged, portMAX_DELAY);

	return USBH_OK;
}

/**
 * @brief  USBH_HID_ClassRequest - HID 类标准请求处理
 * @param  phost: USB Host 句柄
 * @retval USBH 状态
 * @note   状态机: REQ_INIT → GET_HID_DESC → GET_REPORT_DESC → SET_IDLE → SET_PROTOCOL
 */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status         = USBH_BUSY;
	USBH_StatusTypeDef classReqStatus = USBH_BUSY;
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

	switch (HID_Handle->ctl_state)
	{
		case USBH_HID_REQ_INIT:
		case USBH_HID_REQ_GET_HID_DESC:
			USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.CfgDesc_Raw);
			HID_Handle->ctl_state = USBH_HID_REQ_GET_REPORT_DESC;
			break;

		case USBH_HID_REQ_GET_REPORT_DESC:
			classReqStatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->HID_Desc.wItemLength);
			if (classReqStatus == USBH_OK)
				HID_Handle->ctl_state = USBH_HID_REQ_SET_IDLE;
			else if (classReqStatus == USBH_NOT_SUPPORTED)
				status = USBH_FAIL;
			break;

		case USBH_HID_REQ_SET_IDLE:
			classReqStatus = USBH_HID_SetIdle(phost, 0U, 0U);
			if (classReqStatus == USBH_OK)
				HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
			else if (classReqStatus == USBH_NOT_SUPPORTED)
				HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
			break;

		case USBH_HID_REQ_SET_PROTOCOL:
			classReqStatus = USBH_HID_SetProtocol(phost, 0U);
			if (classReqStatus == USBH_OK)
			{
				HID_Handle->ctl_state = USBH_HID_REQ_IDLE;
				phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
				status = USBH_OK;
			}
			else if (classReqStatus == USBH_NOT_SUPPORTED)
				status = USBH_FAIL;
			break;

		case USBH_HID_REQ_IDLE:
		default:
			break;
	}

	return status;
}

/**
 * @brief  USBH_HID_Process - HID 数据传输状态机
 * @param  phost: USB Host 句柄
 * @retval USBH 状态
 * @note   状态: INIT → IDLE → SYNC → GET_DATA → POLL
 *         通过 GetReport 和 Interrupt IN 管道获取 HID 输入报告
 */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
	USBH_StatusTypeDef status = USBH_OK;
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;
	uint32_t XferSize;

	switch (HID_Handle->state)
	{
		case USBH_HID_INIT:
			status = HID_Handle->Init(phost);
			if (status == USBH_OK)
				HID_Handle->state = USBH_HID_IDLE;
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
			/* 发送 GetReport 请求获取 HID 输入报告 */
			status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length);
			if (status == USBH_OK)
				HID_Handle->state = USBH_HID_SYNC;
			else if (status == USBH_BUSY)
				HID_Handle->state = USBH_HID_IDLE, status = USBH_OK;
			else if (status == USBH_NOT_SUPPORTED)
				HID_Handle->state = USBH_HID_SYNC, status = USBH_OK;
			else
				HID_Handle->state = USBH_HID_ERROR, status = USBH_FAIL;
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
			/* 同步到偶数帧开始 */
			if ((phost->Timer & 1U) != 0U)
				HID_Handle->state = USBH_HID_GET_DATA;
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
			/* 启动中断 IN 传输接收 HID 数据 */
			(void)USBH_InterruptReceiveData(phost, HID_Handle->pData,
			                                (uint8_t)HID_Handle->length,
			                                HID_Handle->InPipe);
			HID_Handle->state     = USBH_HID_POLL;
			HID_Handle->timer     = phost->Timer;
			HID_Handle->DataReady = 0U;
			break;

		case USBH_HID_POLL:
			if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE)
			{
				XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);
				if ((HID_Handle->DataReady == 0U) && (XferSize != 0U) && (HID_Handle->fifo.buf != NULL))
				{
					/* 将接收到的数据写入 FIFO，供上层解析 */
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
				/* IN 端点停止，发送 ClearFeature 后重试 */
				if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL)
				{
					if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK)
						HID_Handle->state = USBH_HID_GET_DATA;
				}
			}
			break;

		default:
			break;
	}

	return status;
}

/**
 * @brief  USBH_HID_SOFProcess - SOF 帧同步处理
 * @param  phost: USB Host 句柄
 * @retval USBH_OK
 * @note   在 HID_POLL 状态下按设备轮询间隔定时触发数据获取
 */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

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
 * @brief  USBH_ParseHIDDesc - 解析 HID 描述符
 * @param  desc: 输出 HID 描述符结构体指针
 * @param  buf:  配置描述符原始数据缓冲区
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
				desc->bLength              = *(uint8_t *)((uint8_t *)pdesc + 0U);
				desc->bDescriptorType      = *(uint8_t *)((uint8_t *)pdesc + 1U);
				desc->bcdHID               = LE16((uint8_t *)pdesc + 2U);
				desc->bCountryCode         = *(uint8_t *)((uint8_t *)pdesc + 4U);
				desc->bNumDescriptors      = *(uint8_t *)((uint8_t *)pdesc + 5U);
				desc->bReportDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 6U);
				desc->wItemLength          = LE16((uint8_t *)pdesc + 7U);
				break;
			}
		}
	}
}

/* ========================= PS2 初始化和数据解码 ==============================*/

/* PS2 手柄 HID 输入报告缓冲区（64 字节） */
static uint8_t ps2_report_data[64] = { 0 };

/**
 * @brief  USB 插入蜂鸣提示回调（两次短鸣 + 一次长鸣）
 */
static void timer_usb_inset_callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t beep = &UserBuzzer;
	beep->on();  vTaskDelay(80);
	beep->off(); vTaskDelay(50);
	beep->on();  vTaskDelay(400);
	beep->off();
}

/**
 * @brief  USB 拔出蜂鸣提示回调（一次长鸣 + 一次短鸣）
 */
static void timer_usb_pull_callback(TimerHandle_t xTimer)
{
	pBuzzeInterface_t beep = &UserBuzzer;
	beep->on();  vTaskDelay(800);
	beep->off(); vTaskDelay(50);
	beep->on();  vTaskDelay(200);
	beep->off();
}

/**
 * @brief  PS2 HID 初始化（设备识别后调用）
 * @param  phost: USB Host 句柄
 * @retval USBH_OK / USBH_FAIL
 * @note   配置 HID_Handle 的 pData 和 FIFO
 *          创建插入/拔出提示定时器
 */
USBH_StatusTypeDef USBH_HID_PS2Init(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

	/* 限制接收长度不超过缓冲区大小 */
	if (HID_Handle->length > sizeof(ps2_report_data))
	{
		HID_Handle->length = (uint16_t)sizeof(ps2_report_data);
	}

	/* 设置 HID 数据缓冲区 */
	HID_Handle->pData = ps2_report_data;

	/* 检查 FIFO 所需内存是否超过 USB 数据缓冲区 */
	if ((HID_QUEUE_SIZE * sizeof(ps2_report_data)) > sizeof(phost->device.Data))
	{
		return USBH_FAIL;
	}
	else
	{
		/* 初始化 FIFO */
		USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data,
		                  (uint16_t)(HID_QUEUE_SIZE * sizeof(ps2_report_data)));
	}

	/* 创建插入/拔出提示定时器（首次调用时） */
	if (TimerUSBinsert == NULL)
		TimerUSBinsert = xTimerCreate("USBinTips", pdMS_TO_TICKS(10), pdFALSE, NULL, timer_usb_inset_callback);
	if (TimerUSBunplugged == NULL)
		TimerUSBunplugged = xTimerCreate("USBoutTips", pdMS_TO_TICKS(10), pdFALSE, NULL, timer_usb_pull_callback);
	xTimerStart(TimerUSBinsert, portMAX_DELAY);

	return USBH_OK;
}

/* 三种模式解码函数前置声明 */
static void Wired_PS2_Decode(const uint8_t *data);
static void Wiredless_PC_PS2_Decode(const uint8_t *data);
static void Wiredless_Android_PS2_Decode(const uint8_t *data);

/**
 * @brief  PS2 HID 数据解码入口
 * @param  phost: USB Host 句柄
 * @retval USBH_OK / USBH_FAIL
 * @note   从 FIFO 读取 HID 数据，根据手柄类型调用对应的解码函数
 */
USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *)phost->pActiveClass->pData;

	if (ps2_type == UnKnown_Dev) return USBH_FAIL;

	if ((HID_Handle->length == 0U) || (HID_Handle->fifo.buf == NULL))
	{
		return USBH_FAIL;
	}

	/* 从 FIFO 中读取 HID 数据 */
	if (USBH_HID_FifoRead(&HID_Handle->fifo, (uint8_t*)ps2_report_data, HID_Handle->length) == HID_Handle->length)
	{
		if (Wired_PS2 == ps2_type)
			Wired_PS2_Decode(ps2_report_data);
		else if (Wiredless_PC_PS2 == ps2_type)
			Wiredless_PC_PS2_Decode(ps2_report_data);
		else if (Wiredless_Android_PS2 == ps2_type)
			Wiredless_Android_PS2_Decode(ps2_report_data);

		return USBH_OK;
	}
	return USBH_FAIL;
}

/* ====================== PS2 按键检测状态机 ==================================*/

#define PS2_KEY_NUM 16                  /* 16 个按键 */

/* 按键识别时间阈值（ms） */
#define PS2_LONGPRESS_TIEM 1000         /* 长按判定时间 */
#define PS2_CLICK_TIME     400          /* 双击间隔判定时间 */
#define PS2_KEYFILTER_TIME 50           /* 按键去抖滤波时间 */

/* 按键检测状态 */
typedef enum {
	WaitToPress  = 0,                   /* 等待按键按下 */
	WaitToRelease,                      /* 等待按键释放 */
	KEYPress,                           /* 按键已按下 */
	KEYUp,                              /* 按键已弹起 */
	LONG_CLICK,                         /* 长按 */
} PS2KEY_CheckState;

/* 按键检测辅助结构体 */
typedef struct
{
	uint8_t  keystate;                  /* 当前键值: 0=弹起, 1=按下 */
	uint32_t timebase;                  /* 时间基准（ms） */
	uint32_t statetime;                 /* 状态持续时间 */
	PS2KEY_CheckState statemachine;     /* 检测状态机 */
} PS2_CheckKey_t;

PS2_CheckKey_t ps2key[PS2_KEY_NUM] = { 0 };

/**
 * @brief  PS2 按键事件检测（单击/双击/长按）
 * @param  bit: 按键位号（0~15）
 * @retval PS2KEYSTATE_NONE / SINGLECLICK / DOUBLECLICK / LONGCLICK
 * @note   状态机: WaitToPress → KEYPress → (LONG_CLICK | KEYUp → SINGLE/DOUBLE)
 */
PS2KEY_State_t ps2_checkkey(uint8_t bit)
{
	PS2_CheckKey_t* key = &ps2key[bit];

	/* 读取当前按键状态 */
	key->keystate = (ps2_KeyVal >> bit) & 0x01;

	switch (key->statemachine)
	{
		case WaitToPress:
			if (PS2KEY_PressDOWN == key->keystate)
			{
				key->timebase = HAL_GetTick();
				key->statemachine = KEYPress;
			}
			break;
		case KEYPress:
			key->statetime = HAL_GetTick() - key->timebase;

			if (PS2KEY_PressUP == key->keystate)
			{
				/* 按下的时间过短，视为抖动忽略 */
				if (key->statetime < PS2_KEYFILTER_TIME)
					key->statemachine = WaitToPress;
				else
				{
					key->timebase = HAL_GetTick(); /* 重新计时 */
					key->statemachine = KEYUp;     /* 进入弹起状态 */
				}
			}
			else if (key->statetime > PS2_LONGPRESS_TIEM)
			{
				/* 持续按下超过 1 秒 → 长按 */
				key->statemachine = LONG_CLICK;
			}
			break;
		case KEYUp:
			key->statetime = HAL_GetTick() - key->timebase;

			if (PS2KEY_PressDOWN == key->keystate
			    && key->statetime < PS2_CLICK_TIME
			    && key->statetime > PS2_KEYFILTER_TIME)
			{
				key->statemachine = WaitToRelease;
				return PS2KEYSTATE_DOUBLECLICK;
			}
			else if (key->statetime >= PS2_CLICK_TIME)
			{
				key->statemachine = WaitToRelease;
				return PS2KEYSTATE_SINGLECLICK;
			}
			break;
		case LONG_CLICK:
			key->statemachine = WaitToRelease;
			return PS2KEYSTATE_LONGCLICK;
		case WaitToRelease:
			/* 等待按键释放后回到初始状态 */
			if (PS2KEY_PressUP == key->keystate) key->statemachine = WaitToPress;
			break;
		default:
			break;
	}

	return PS2KEYSTATE_NONE;
}

/**
 * @brief  直接返回按键实时状态（无消抖）
 * @param  bit: 按键位号（0~15）
 * @retval 0=弹起, 1=按下
 */
uint8_t ps2_checkkeystate(uint8_t bit)
{
	return (ps2_KeyVal >> bit) & 0x01;
}

/* ====================== 位操作辅助函数 ======================================*/

/**
 * @brief  设置/清除 ps2_KeyVal 的指定位
 * @param  state:     指向 ps2_KeyVal 的指针
 * @param  state_bit: 0=清零指定位, 非0=置位指定位
 * @param  bit:       位索引（0~15）
 */
static void ps2_set_bit(uint16_t* state, uint8_t state_bit, uint8_t bit)
{
	if (state_bit == 1)
		*state |= (1U << bit);           /* 置位 */
	else
		*state &= ~(1U << bit);          /* 清零 */
}

/* ====================== 三种 PS2 模式数据解码 ===============================*/

/**
 * @brief  有线 PS2 手柄数据解码
 * @param  data: HID 报告数据（8 字节）
 * @note   摇杆范围: 0~255, 中值 127
 *         按键映射: bit0=SELECT, bit1=L3, bit2=R3, bit3=START
 *                   bit4~7=方向键, bit8=L2, bit9=R2, bit10=L1, bit11=R1
 *                   bit12=△, bit13=○, bit14=×, bit15=□
 */
static void Wired_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0;

	ps2_info.LX = data[3];               /* 左摇杆 X */
	ps2_info.LY = data[4];               /* 左摇杆 Y */
	ps2_info.RX = data[1];               /* 右摇杆 X */
	ps2_info.RY = data[2];               /* 右摇杆 Y */

	tmp_bool = (data[6] >> 4) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 0);  /* SELECT */
	tmp_bool = (data[6] >> 6) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 1);  /* L3 (左摇杆按下) */
	tmp_bool = (data[6] >> 7) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 2);  /* R3 (右摇杆按下) */
	tmp_bool = (data[6] >> 5) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 3);  /* START */

	/* 方向键解析（4 位编码） */
	tmp_bool = data[5] & 0x0F;
	if (tmp_bool == 0x0F)
	{
		/* 无方向键按下 */
		ps2_set_bit(&ps2_KeyVal, 0, 4); ps2_set_bit(&ps2_KeyVal, 0, 5);
		ps2_set_bit(&ps2_KeyVal, 0, 6); ps2_set_bit(&ps2_KeyVal, 0, 7);
	}
	else if ((tmp_bool & 0x01) == 0)
	{
		switch ((tmp_bool >> 1) & 0x03)
		{
			case 0x00: ps2_set_bit(&ps2_KeyVal, 1, 4); break; /* 上 */
			case 0x01: ps2_set_bit(&ps2_KeyVal, 1, 5); break; /* 下 */
			case 0x02: ps2_set_bit(&ps2_KeyVal, 1, 6); break; /* 左 */
			case 0x03: ps2_set_bit(&ps2_KeyVal, 1, 7); break; /* 右 */
		}
	}
	else
	{
		/* 对角线方向 */
		switch ((tmp_bool >> 1) & 0x03)
		{
			case 0x00: /* 左上 */ ps2_set_bit(&ps2_KeyVal, 1, 4); ps2_set_bit(&ps2_KeyVal, 1, 6); break;
			case 0x01: /* 左下 */ ps2_set_bit(&ps2_KeyVal, 1, 5); ps2_set_bit(&ps2_KeyVal, 1, 6); break;
			case 0x02: /* 右下 */ ps2_set_bit(&ps2_KeyVal, 1, 5); ps2_set_bit(&ps2_KeyVal, 1, 7); break;
			case 0x03: /* 右上 */ ps2_set_bit(&ps2_KeyVal, 1, 4); ps2_set_bit(&ps2_KeyVal, 1, 7); break;
		}
	}

	tmp_bool = (data[6] >> 2) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 8);  /* L2 */
	tmp_bool = (data[6] >> 3) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 9);  /* R2 */
	tmp_bool = (data[6] >> 0) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 10); /* L1 */
	tmp_bool = (data[6] >> 1) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 11); /* R1 */
	tmp_bool = (data[5] >> 4) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 12); /* △ GREEN */
	tmp_bool = (data[5] >> 5) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 13); /* ○ RED */
	tmp_bool = (data[5] >> 6) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 14); /* × BLUE */
	tmp_bool = (data[5] >> 7) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 15); /* □ PINK */
}

/**
 * @brief  无线 Android 模式 PS2 手柄数据解码
 * @param  data: HID 报告数据
 */
static void Wiredless_Android_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0;
	uint8_t rm_val = 0;

	/* 摇杆值（含零值保护：0 表示中值 128） */
	if (data[6] == 0 && data[7] == 0) rm_val = 128;
	else rm_val = data[6];
	ps2_info.LX = rm_val;

	if (data[8] == 0 && data[9] == 0) rm_val = 128;
	else rm_val = data[8];
	ps2_info.LY = 255 - rm_val;

	if (data[10] == 0 && data[11] == 0) rm_val = 128;
	else rm_val = data[10];
	ps2_info.RX = rm_val;

	if (data[12] == 0 && data[13] == 0) rm_val = 128;
	else rm_val = data[12];
	ps2_info.RY = 255 - rm_val;

	/* 按键: data[2] 位映射 */
	tmp_bool = (data[2] >> 0) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 4);  /* 上 */
	tmp_bool = (data[2] >> 3) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 5);  /* 下 */
	tmp_bool = (data[2] >> 1) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 6);  /* 左 */
	tmp_bool = (data[2] >> 2) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 7);  /* 右 */
	tmp_bool = (data[2] >> 5) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 0);  /* SELECT */
	tmp_bool = (data[2] >> 4) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 3);  /* START */
	tmp_bool = (data[2] >> 6) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 1);  /* L3 */
	tmp_bool = (data[2] >> 7) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 2);  /* R3 */
	tmp_bool = (data[3] >> 0) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 10); /* L1 */
	tmp_bool = (data[3] >> 1) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 11); /* R1 */

	if (data[4] == 0xff) tmp_bool = 1;
	else tmp_bool = 0;                   ps2_set_bit(&ps2_KeyVal, tmp_bool, 8);  /* L2 */
	if (data[5] == 0xff) tmp_bool = 1;
	else tmp_bool = 0;                   ps2_set_bit(&ps2_KeyVal, tmp_bool, 9);  /* R2 */

	tmp_bool = (data[3] >> 4) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 14); /* × BLUE */
	tmp_bool = (data[3] >> 5) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 13); /* ○ RED */
	tmp_bool = (data[3] >> 6) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 15); /* □ PINK */
	tmp_bool = (data[3] >> 7) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 12); /* △ GREEN */
}

/**
 * @brief  无线 PC 模式 PS2 手柄数据解码
 * @param  data: HID 报告数据
 */
static void Wiredless_PC_PS2_Decode(const uint8_t *data)
{
	uint8_t tmp_bool = 0;

	ps2_info.LX = data[3];
	ps2_info.LY = data[4];
	ps2_info.RX = data[5];
	ps2_info.RY = data[6];

	tmp_bool = (data[1] >> 0) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 0);  /* SELECT */
	tmp_bool = (data[1] >> 1) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 3);  /* START */
	tmp_bool = (data[1] >> 2) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 1);  /* L3 */
	tmp_bool = (data[1] >> 3) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 2);  /* R3 */
	tmp_bool = (data[0] >> 4) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 10); /* L1 */
	tmp_bool = (data[0] >> 5) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 11); /* R1 */
	tmp_bool = (data[0] >> 6) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 8);  /* L2 */
	tmp_bool = (data[0] >> 7) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 9);  /* R2 */
	tmp_bool = (data[0] >> 0) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 12); /* △ GREEN */
	tmp_bool = (data[0] >> 1) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 13); /* ○ RED */
	tmp_bool = (data[0] >> 2) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 14); /* × BLUE */
	tmp_bool = (data[0] >> 3) & 0x01;    ps2_set_bit(&ps2_KeyVal, tmp_bool, 15); /* □ PINK */

	/* 方向键 */
	tmp_bool = data[2] & 0x0F;
	if (tmp_bool == 0x0F)
	{
		ps2_set_bit(&ps2_KeyVal, 0, 4); ps2_set_bit(&ps2_KeyVal, 0, 5);
		ps2_set_bit(&ps2_KeyVal, 0, 6); ps2_set_bit(&ps2_KeyVal, 0, 7);
	}
	else if ((tmp_bool & 0x01) == 0)
	{
		switch ((tmp_bool >> 1) & 0x03)
		{
			case 0x00: ps2_set_bit(&ps2_KeyVal, 1, 4); break; /* 上 */
			case 0x01: ps2_set_bit(&ps2_KeyVal, 1, 5); break; /* 下 */
			case 0x02: ps2_set_bit(&ps2_KeyVal, 1, 6); break; /* 左 */
			case 0x03: ps2_set_bit(&ps2_KeyVal, 1, 7); break; /* 右 */
		}
	}
	else
	{
		switch ((tmp_bool >> 1) & 0x03)
		{
			case 0x00: ps2_set_bit(&ps2_KeyVal, 1, 4); ps2_set_bit(&ps2_KeyVal, 1, 6); break;
			case 0x01: ps2_set_bit(&ps2_KeyVal, 1, 5); ps2_set_bit(&ps2_KeyVal, 1, 6); break;
			case 0x02: ps2_set_bit(&ps2_KeyVal, 1, 5); ps2_set_bit(&ps2_KeyVal, 1, 7); break;
			case 0x03: ps2_set_bit(&ps2_KeyVal, 1, 4); ps2_set_bit(&ps2_KeyVal, 1, 7); break;
		}
	}
}
