/*
 * usb_user_if.h
 *
 *  Created on: 2019年12月11日
 *      Author: Jingtai_Wu
 */

#ifndef INC_DRIVER_USER_IF_H_
#define INC_DRIVER_USER_IF_H_

#include <stdbool.h>

#define COMM_TIMEOUT    (1000)
#define USB_PACKET_SIZE (64)
#define Q_MAX_SIZE      (4)
#define PRE_Q_NUMBER    (4)
#define PRE_Q_LENGTH    (USB_PACKET_SIZE/PRE_Q_NUMBER)
#define PRE_Q_TIMEOUT    (100)  //ms


typedef enum
{
	COMM_OK = 0x00U, COMM_FAIL = 0x01U
} comm_status;

typedef enum
{
	txdata = 0x00U, txstatus = 0x01U
}usb_tx_mode;

typedef enum
{
	q_empty = 0x00U,
	q_num1 = 0x01U,
	q_num2 = 0x02U,
	q_num3 = 0x03U,
	q_full = 0x04U,
} q_status;

typedef struct
{
	uint8_t mode;
	uint8_t reserved[2];
	uint8_t length;
	uint32_t cmd;   // ID
	uint8_t payload[56];
	uint8_t *pdata;
} usb_msg_t;

typedef struct
{
	uint8_t config_mode;
	uint8_t bank_number;
	uint8_t id_mode;           // mask or list mode
	uint8_t en;
	uint8_t reserved[4];
	uint8_t id1_filter[4];
	uint8_t id2_mask_filter[4];
} can_filter_config_t;

typedef struct
{
	uint8_t payload[USB_PACKET_SIZE];
	uint8_t pk_length;
} usb_packet_info;

typedef union
{
	usb_msg_t msg;
	usb_packet_info packet;
} usb_message_t;

typedef struct
{
	uint32_t cs; /*!< Code and Status*/
	uint32_t id; /*!< ID of the message */
	//uint8_t data[USB_PACKET_SIZE];  /*!< Data bytes of the CAN message*/
	uint8_t *pdata;
	uint8_t length; /*!< Length of payload in bytes */
} can_message_t;

typedef struct
{
	uint8_t data[Q_MAX_SIZE][USB_PACKET_SIZE];
	uint8_t front;
	uint8_t rear;
} queue_t;

typedef struct
{
	uint8_t *speed;
	uint32_t prescaler;
	uint32_t seg1;
	uint32_t seg2;
} can_bitrate_config_t ;


void message_buffer_init(void);
uint8_t USB_Send(usb_message_t *message);
void USB_Receive_Callback(uint8_t event_idx, uint8_t state);
void CAN_Filter_Init(void);
uint8_t CAN_Send(can_message_t *message);
void enqueue(queue_t*, uint8_t *tx_msg);
q_status dequeue(queue_t*, usb_message_t *tx_msg);
//void TIM32_DelayMS(unsigned int ms);
void TIM32_Stop(void);

extern usb_message_t usb_tx_buf;
extern usb_message_t usb_rx_buf;
extern queue_t Q;
extern can_message_t can_tx_buf;
extern can_message_t can_rx_buf;
extern volatile bool g_usb_rx_complete;
extern volatile bool g_can_rx_complete;

#endif /* INC_DRIVER_USER_IF_H_ */
