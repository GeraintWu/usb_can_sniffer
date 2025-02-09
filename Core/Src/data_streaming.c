/*
 * data_streaming.c
 *
 *  Created on: 2019年12月12日
 *      Author: Jingtai_Wu
 */
#include "main.h"
#include "usbd_customhid.h"

//static comm_status message_transport(void);
//static void comm_error(void);

usb_message_t usb_q_buf;
volatile uint32_t tx_cnt = 0;
void can_data_logger(void)
{
	//uint32_t i;
	q_status status = q_empty;

	while (1)
	{
		//g_can_rx_complete = false;

		status = dequeue(&Q, &usb_q_buf);

		if (status != q_empty)
		{
			//while(USB_Send(&usb_q_buf) != COMM_OK);
			USB_Send(&usb_q_buf);
			tx_cnt++;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		}

//         HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//         for(i=0;i<1600000;i++)
//            if(g_can_rx_complete == true) break;
	}
}

#if 0
void download_app(void)
{
	uint32_t i;
	comm_status ret;

	while(1)
	{
		if(g_usb_rx_complete == true)
		{
			g_usb_rx_complete = false;

			ret = message_transport();

#ifdef __DEBUG_PRINTF__
	       printf("ERROR CODE:%d\n", ret);
#endif

	       if(ret == COMM_FAIL)
	       {
	    	   comm_error();
	       }
		} // end of if(g_usb_rx_complete == true)

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for(i=0;i<1600000;i++)
        	if(g_usb_rx_complete == true) break;

	} //end of while(1)
}

static comm_status message_transport(void)
{
	comm_status status;

    uint32_t tickstart = 0U;

	can_tx_buf.id = usb_rx_buf.msg.cmd;
	can_tx_buf.length = (uint8_t) usb_rx_buf.msg.length;
	can_rx_buf.id = 0; //clear CAN rx_cmd

	status = CAN_Send(&can_tx_buf);

	if(status != COMM_OK)
		return COMM_FAIL;

    tickstart = HAL_GetTick();

	while(can_rx_buf.id != (usb_rx_buf.msg.cmd+1)) // wait ACK from loader
	{
        if((HAL_GetTick()-tickstart) > COMM_TIMEOUT)
        {
            // non-acknowledge
            return COMM_FAIL;
        }
    }


	usb_tx_buf.msg.cmd = can_rx_buf.id;
	usb_tx_buf.msg.length = (uint32_t) can_rx_buf.length;
	status = USB_Send(&usb_tx_buf);

	if(status != COMM_OK)
		return COMM_FAIL;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

	return COMM_OK;
}

static void comm_error(void)
{
	memset(usb_tx_buf.packet, 0, sizeof(usb_tx_buf.packet));
	USB_Send(&usb_tx_buf);
}
#endif

