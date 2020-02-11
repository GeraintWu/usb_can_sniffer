/*
 * sniffer_config.c
 *
 *  Created on: Jan 14, 2020
 *      Author: Jingtai_Wu
 */
#include "main.h"
#include "usbd_customhid.h"

extern uint8_t sniffer_mode;
extern CAN_HandleTypeDef hcan;
extern usb_message_t usb_rx_buf;

static CAN_FilterTypeDef sFilterConfig;

static void config_bitrate(usb_message_t cmd);
static void config_filter(can_filter_config_t value);
//static void can_config(usb_message_t cmd, void(*config_mode)(usb_message_t));

const can_bitrate_config_t g_can_speed[] =
{
{ .speed = (uint8_t*) "1000", .prescaler = (uint32_t) 3, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "500", .prescaler = (uint32_t) 6, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "250", .prescaler = (uint32_t) 12, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "125", .prescaler = (uint32_t) 24, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "100", .prescaler = (uint32_t) 30, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "50", .prescaler = (uint32_t) 60, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
{ .speed = (uint8_t*) "10", .prescaler = (uint32_t) 300, .seg1 =
		(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ }, };

static void config_bitrate(usb_message_t cmd)
{
	HAL_CAN_DeInit(&hcan);

	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;

	hcan.Init.Prescaler = g_can_speed[cmd.msg.payload[0]].prescaler;

	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;

	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}
}

static void config_filter(can_filter_config_t value)
{

	sFilterConfig.FilterBank = value.bank_number;

	if (value.en == 1)
	{
		if (value.id_mode == 0)
			sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		else
			sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;

		sFilterConfig.FilterIdHigh = (value.id1_filter[0] << 8) | value.id1_filter[1];
		sFilterConfig.FilterIdLow  = (value.id1_filter[2] << 8) | value.id1_filter[3];

		sFilterConfig.FilterMaskIdHigh = (value.id2_mask_filter[0] << 8) | value.id2_mask_filter[1];
		sFilterConfig.FilterMaskIdLow  = (value.id2_mask_filter[2] << 8) | value.id2_mask_filter[3];

		sFilterConfig.FilterActivation = ENABLE;
	}

	else
	{
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterActivation = DISABLE;

	}

	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	//sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14; // meaningless

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

//	if (HAL_CAN_ActivateNotification(&hcan,
//	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
//	{
//		/* Notification Error */
//		Error_Handler();
//	}
}

//static void can_config(usb_message_t cmd, void(*config_mode)(usb_message_t))
//{
//	HAL_CAN_Stop(&hcan);
//config CAN bitrate | filter
//	(*config_mode)(cmd);
//}

void can_sniffer_config(void)
{
	usb_message_t config_cmd;
	can_filter_config_t fval; //filter value

	//g_usb_rx_complete = false;
	//memset(usb_rx_buf.packet.payload, 0, 64);

	while (1)
	{

		/* wait for a config command from PC tool */
		if (g_usb_rx_complete == true)
		{
			g_usb_rx_complete = false;

			/* copy the received data to new buffer to prevent buffer override when new packet comes in */
			memcpy(config_cmd.packet.payload, usb_rx_buf.packet.payload, 64);

			HAL_CAN_Stop(&hcan);

			switch (config_cmd.msg.mode)
			{
			case 0x1:
				config_bitrate(config_cmd);
				break;

			case 0x2:
				memcpy((uint8_t*) &fval, (uint8_t*) &config_cmd, sizeof(can_filter_config_t));
				config_filter(fval);
				break;

			default:
				break;
			}

			sniffer_mode = CAPTURE_MODE;
			break;

		}
	} // End of while(1)

}
