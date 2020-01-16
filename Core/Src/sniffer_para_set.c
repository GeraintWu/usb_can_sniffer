/*
 * sniffer_para_set.c
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

static void config_bitrate(uint8_t speed_select);
static void config_filter(void);
static void can_config(usb_message_t usb_config,
		         void(*can_speed)(uint8_t speed),
				 void(*can_filter)(void));



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
			(uint32_t) CAN_BS1_13TQ, .seg2 = (uint32_t) CAN_BS2_2TQ },
};


static void config_bitrate(uint8_t speed_select)
{

	hcan.Instance = CAN;
	hcan.Init.Mode = CAN_MODE_NORMAL;


	hcan.Init.Prescaler = g_can_speed[speed_select].prescaler;

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

static void config_filter(void)
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
}


static void can_config(usb_message_t cmd,
		         void(*can_speed)(uint8_t speed),
				 void(*can_filter)(void))
{
	HAL_CAN_Stop(&hcan);
	HAL_CAN_DeInit(&hcan);

	//config CAN bitrate
	can_speed(cmd.packet.payload[4]);


	//config CAN filter
	can_filter();
}


void can_sniffer_config(void)
{
	usb_message_t config_cmd;

	g_usb_rx_complete = false;
	memset(usb_rx_buf.packet.payload, 0, 64);

	while (1)
	{

		/* wait for a config command from PC tool */
		if (g_usb_rx_complete == true)
		{
			g_usb_rx_complete = false;

			/* copy the received data to prevent buffer override */
			memcpy(config_cmd.packet.payload, usb_rx_buf.packet.payload, 64);

			if ((config_cmd.packet.payload[1] == 's')
					&& (config_cmd.packet.payload[2] == 'e')
					&& (config_cmd.packet.payload[3] == 't'))
			{
				can_config(config_cmd, config_bitrate, config_filter);

				sniffer_mode = CAPTURE_MODE;
				break;
			}
		}
	}

}
