/*
 * driver_user_if.c
 *
 *  Created on: 2019年12月11日
 *      Author: Jingtai_Wu
 */

#include "main.h"
#include "usbd_customhid.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern CAN_HandleTypeDef hcan;
extern TIM_HandleTypeDef htim3;
extern uint8_t USBD_CUSTOM_HID_SendReport(USBD_HandleTypeDef *pdev,
		uint8_t *report, uint16_t len);

usb_message_t usb_tx_buf;
usb_message_t usb_rx_buf;
can_message_t can_tx_buf;
can_message_t can_rx_buf;
queue_t Q;

volatile bool g_usb_rx_complete;
volatile bool g_can_rx_complete;
volatile static uint8_t can_tx_complete;
volatile static uint16_t tim3_cnt;

static CAN_FilterTypeDef sFilterConfig;
static CAN_TxHeaderTypeDef can_tx_hd;
static CAN_RxHeaderTypeDef can_rx_hd;
static uint32_t messagebox;
static uint8_t pre_queue[64];
static uint8_t *pre_q_ptr;
static uint8_t pre_q_cnt;
static uint8_t match_idx = 0xFF;
/*FUNCTION**********************************************************************
 *
 * Function Name : message_buffer_init
 * Description   : pointer to TX RX buffers
 * Implements    :
 *END**************************************************************************/

void message_buffer_init(void)
{
	usb_rx_buf.msg.pdata = usb_rx_buf.msg.payload;
	can_tx_buf.pdata = usb_rx_buf.msg.payload;
	usb_tx_buf.msg.pdata = usb_tx_buf.msg.payload;
	can_rx_buf.pdata = usb_tx_buf.msg.payload;

	pre_q_ptr = pre_queue;
	pre_q_cnt = 0;
	Q.front = 0;
	Q.rear = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : Queue operation
 * Description   : Q FIFO push-pop
 * Implements    :
 *END**************************************************************************/
void enqueue(queue_t *Q, uint8_t *tx_msg)
{
	if ((Q->rear + 1) % Q_MAX_SIZE == Q->front)
	{
		//printf("Queue is full!\n");
		//memory run out
		__BKPT(0);
	}
	else
	{
		Q->rear = (Q->rear + 1) % Q_MAX_SIZE;
		memcpy(Q->data[Q->rear], tx_msg, USB_PACKET_SIZE);
	}
}

q_status dequeue(queue_t *Q, usb_message_t *tx_msg)
{

	if (Q->front == Q->rear)
	{
		/*
		 * when Q is emepty, we need to check if data
		 * has been queued in the pre_queue over a period of time.
		 * send them out and reset the pre_q_cnt
		 */
		if ((tim3_cnt > PRE_Q_TIMEOUT) && (pre_q_cnt != 0))
		{
			//stop & clear Timer
			TIM32_Stop();

			//To grab data from pre_queue
#if 0
			memcpy(tx_msg->packet.payload, pre_queue, USB_PACKET_SIZE);
			tx_msg->packet.pk_length = pre_q_cnt * PRE_Q_LENGTH;
#else
			memset(tx_msg->packet.payload, 0, USB_PACKET_SIZE);
			memcpy(tx_msg->packet.payload, pre_queue, pre_q_cnt * PRE_Q_LENGTH);
			tx_msg->packet.pk_length = USB_PACKET_SIZE;
#endif
			//reset pre_q
			pre_q_ptr = pre_queue;
			pre_q_cnt = 0;

			return q_full;
		}
		//printf("Queue is empty!\n");
		return q_empty;
	}
	else
	{
		//stop & clear Timer
		TIM32_Stop();

		Q->front = (Q->front + 1) % Q_MAX_SIZE;
		memcpy(tx_msg->packet.payload, Q->data[Q->front], USB_PACKET_SIZE);
		tx_msg->packet.pk_length = USB_PACKET_SIZE;
		return q_full;
	}
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TIM32_DelayMS
 * Description   : 1ms-unit delay
 * Implements    :
 *END**************************************************************************/
#if 0
void TIM32_DelayMS(unsigned int ms)
{
    /* timer start*/
    HAL_TIM_Base_Start_IT(&htim3);
    tim3_cnt = 0;
    while(tim3_cnt < ms);

    /* timer stop and reset */
    while(HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK);
    htim3.Instance->EGR = 0x0001;
    htim3.Instance->CNT = 0;
}
#endif

void TIM32_Stop(void)
{
	/* timer stop and reset */
	while (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK)
		;
	htim3.Instance->EGR = 0x0001;
	htim3.Instance->CNT = 0;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CAN_Filter_Init
 * Description   : Filter Init and start
 * Implements    :
 *END**************************************************************************/

void CAN_Filter_Init(void)
{
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	//HAL_CAN_Start(&hcan);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : CAN_Send
 * Description   : CAN message send
 * Implements    :
 *END**************************************************************************/
uint8_t CAN_Send(can_message_t *message)
{

	uint32_t tickstart = 0U;

	//can_tx_hd.StdId = 0x321;
	can_tx_hd.ExtId = message->id;
	can_tx_hd.RTR = CAN_RTR_DATA;
	can_tx_hd.IDE = CAN_ID_EXT;
	can_tx_hd.DLC = message->length;
	can_tx_hd.TransmitGlobalTime = DISABLE;

	tickstart = HAL_GetTick();
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	{
		if ((HAL_GetTick() - tickstart) > COMM_TIMEOUT)
		{
			// fail to get mailbox
			return COMM_FAIL;
		}
	}

	can_tx_complete = 0;
	tickstart = HAL_GetTick();

	HAL_CAN_AddTxMessage(&hcan, &can_tx_hd, message->pdata, &messagebox);

	while (can_tx_complete == 0)
	{
		if ((HAL_GetTick() - tickstart) > COMM_TIMEOUT)
		{
			//fail to complete send
			return COMM_FAIL;
		}
	}
	return COMM_OK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : USB_Send
 * Description   : USB message send
 * Implements    :
 *END**************************************************************************/
uint8_t USB_Send(usb_message_t *message)
{

	uint32_t tickstart = 0U;

	tickstart = HAL_GetTick();

	while (USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,
			(uint8_t*) message->packet.payload, message->packet.pk_length)
			!= USBD_OK)
	{
		if ((HAL_GetTick() - tickstart) > COMM_TIMEOUT)
		{
			//fail to send
			return COMM_FAIL;
		}
	}
	return COMM_OK;
}

/**************************INTERRUPT CALLBACK *********************************/
/*FUNCTION**********************************************************************
 *
 * Function Name :
 * Description   :
 * Implements    :
 *END**************************************************************************/

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	can_tx_complete = 1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_hd, can_rx_buf.pdata);
	usb_tx_buf.msg.mode = txdata;
	usb_tx_buf.msg.cmd = can_rx_hd.ExtId;
	usb_tx_buf.msg.length = can_rx_hd.DLC;
	match_idx = can_rx_hd.FilterMatchIndex;
	memcpy(pre_q_ptr, usb_tx_buf.packet.payload, PRE_Q_LENGTH);

	pre_q_ptr += PRE_Q_LENGTH;
	pre_q_cnt++;
	if (pre_q_cnt == 1)
	{
		tim3_cnt = 0;
		HAL_TIM_Base_Start_IT(&htim3);
	}
	if (pre_q_cnt == PRE_Q_NUMBER)
	{
		enqueue(&Q, (uint8_t*) &pre_queue);
		pre_q_ptr = pre_queue;
		pre_q_cnt = 0;
	}

	g_can_rx_complete = true;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Error_Handler();
}

void USB_Receive_Callback(uint8_t event_idx, uint8_t state)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);

	USBD_CUSTOM_HID_HandleTypeDef *hhid = hUsbDeviceFS.pClassData;
	memcpy(usb_rx_buf.packet.payload, hhid->Report_buf, 64);
	g_usb_rx_complete = true;
#ifdef __DEBUG_PRINTF__
	printf("usb data received!\n");
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		tim3_cnt++;
	}
}

