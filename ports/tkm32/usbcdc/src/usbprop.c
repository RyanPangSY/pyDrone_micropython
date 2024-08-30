/**
******************************************************************************
* @file    usbprop.c
* @author  IC Applications Department
* @version  V0.8
* @date  2019_08_02
* @brief   This file provides all the usbprop firmware functions.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, HOLOCENE SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2016 HOLOCENE</center></h2>
*/ 

#include "usbprop.h"
#include "usb.h"
#include "HAL_device.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "py/runtime.h"
#include "py/stream.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "shared/runtime/interrupt_char.h"
#include "shared/runtime/mpirq.h"
#include "uart.h"
#include "irq.h"
#include "pendsv.h"
#include "systick.h"

#include "usb_cdc_port.h"
#include "buffer.h"

unsigned char *pucClassDrcData;

static uint8_t ep1_Tx_dataBuf[TX_BUFF_SIZE];			//USB ���ͻ�����,���Զ����С

unsigned int ui_curTxLen = 0;				//ʵʱ���ͳ���
unsigned int ep1_Tx_dataLen = 0;			//���������ܳ���(С���Զ��建������С)
unsigned char ep1_tx_flag = 0;				//���ͱ�־

//���⴮�ڲ���
LINE_CODING linecoding =
{
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
};

/********************************************************************************************************
**������Ϣ ��USBCDC_senddata(unsigned char *data,unsigned short length)
**�������� ������ת��
**������� ��unsigned char *data(ת��������ָ��),unsigned short length(ת�����ݳ���)
**������� ��
**��    ע ��
********************************************************************************************************/
void USBCDC_senddata(unsigned char *data, unsigned short length)
{
    if(linecoding.datatype == 7)
    {
        while(length)
        {
					*data &= 0x7f;
					length--;
					data++;
        }
    }
    else if(linecoding.datatype == 8)							//������
    {
        while(length)
        {

        }
    }
}

/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������GET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
void Class_Get_Line_Coding(void)      							// bRequest = 0x21
{
    int count;
    switch( usb_running_ctrl_state )
    {
        case    USB_CTRL_SETUP :
            if(req_dir == USB_REQ_DIR_IN)
            {
                pucClassDrcData = (uint8_t*)&linecoding;
                usb_running_ctrl_state = USB_CTRL_IN;
            }
            else
            {
                usb_running_ctrl_state = USB_IDLE;
            }
            break ;

        case    USB_CTRL_IN :

            while(USB->rEP0_CTRL & 0x80);
            for( count = 0; count < 7; count++ )
            {
                USB->rEP0_FIFO = *pucClassDrcData;
                pucClassDrcData ++;
            }
            USB->rEP0_CTRL = 0x87;
						
            usb_running_ctrl_state = USB_IDLE;
            break ;

        default :
            //		usb_running_ctrl_state = USB_IDLE ;
            break ;
    }
}
unsigned char baud_read [64];
/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������SET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
void Class_Set_Line_Coding(void)  // bRequest = 0x20
{
    int count, i;

    switch( usb_running_ctrl_state )
    {
        case    USB_CTRL_SETUP :
            if(req_dir == USB_REQ_DIR_OUT)
            {
                usb_running_ctrl_state = USB_CTRL_OUT;
								
            }
            else
            {
                usb_running_ctrl_state = USB_IDLE;
            }
            break ;

        case    USB_CTRL_OUT :
            count = USB->rEP0_AVIL;
            for(i = 0; i < count ; i++)
            {
                baud_read[i] = USB->rEP0_FIFO;
            }
            linecoding.bitrate = ((uint32_t)baud_read[0] << 0) | ((uint32_t)baud_read[1] << 8) | ((uint32_t)baud_read[2] << 16) | ((uint32_t)baud_read[3] << 24);
						//UartInit(UART1,linecoding.bitrate);
            usb_running_ctrl_state = USB_CTRL_STATUS;
            break;

        default :
            //			usb_running_ctrl_state = USB_IDLE ;
            break ;
    }
}
/********************************************************************************************************
**������Ϣ ��Class_Get_Line_Coding(void)
**�������� ��CDC Abstract Control Model(���⴮��) ������SET_LINE_CODING
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/

void Class_Set_Control_Line_State(void)      				// bRequest = 0x22
{
    int count, i;
    unsigned char state_temp = 0;
    state_temp = state_temp;
		
		
    switch( usb_running_ctrl_state )
    {
			
			
        case    USB_CTRL_SETUP :							//���ƴ���״̬�׶�
            if(req_dir == USB_REQ_DIR_OUT)
            {

                usb_running_ctrl_state = USB_CTRL_OUT;
            }
            else
            {

                usb_running_ctrl_state = USB_IDLE;
            }
            break ;
        case    USB_CTRL_OUT :
            count = USB->rEP0_AVIL;
            for(i = 0; i < count ; i++)
            {
                state_temp = USB->rEP0_FIFO;				//�˴�Ϊ������(���ƴ������ݽ׶�)
								
            }
            usb_running_ctrl_state = USB_CTRL_STATUS;
	
            break;

        default :
            {

                //usb_running_ctrl_state = USB_IDLE ;
                break ;
            }
    }
}


/********************************************************************************************************
**������Ϣ ��EP1_USB_IN_Data(void)
**�������� ��USB�ӷ��ͻ�����������ͨ��IN�˵㷢��
**������� ��
**������� ��
**��    ע ��USB���ݷ���ʱ�����ݳ������˵�sizeʱ����Ҫ�������
********************************************************************************************************/
void EP1_USB_IN_Data(void)      			// bRequest = 0x22
{
	int count, i;

	if(ep1_tx_flag != 0)						//tx_buf�ǿ�
	{
		while(USB->rEP1_CTRL & 0x80);
		if(ep1_Tx_dataLen > 64)				//USB���ݲ�ַ���
		{
			count = 64;
			ep1_Tx_dataLen -= 64;
		}
		
		else
		{
			count = ep1_Tx_dataLen;
			ep1_Tx_dataLen = 0;
		}
		for(i = 0; i < count; i++)
		{
			USB->rEP1_FIFO = *(ep1_Tx_dataBuf + ui_curTxLen + i);
		}
		USB->rEP1_CTRL = 0x80 | count;
		if(ep1_Tx_dataLen == 0)
		{
			ep1_tx_flag = 0;
			ui_curTxLen = 0;
			usb_running_ctrl_state = USB_IDLE;
		}
		else
		{
			ui_curTxLen += count;
		}
	}

}

/********************************************************************************************************
**������Ϣ ��UsbVcomSend(unsigned char*Info,unsigned int *infoLenth)
**�������� ����Ҫ���͵�����д��USB���ͻ�����
**������� ��unsigned char*Info(��������ָ��),unsigned int *infoLenth(�������ݳ���ָ��)
**������� ��return status(0��ʾ�������,1��ʾ���ڷ���);
**��    ע ��
********************************************************************************************************/

void send_delay(uint32_t mdelay)
{
	uint32_t j=0;
	while(mdelay--)
	{
		for(j=0;j<1000000;j++);
	}
}
int UsbVcomSend(uint8_t *Info, unsigned int infoLenth)
{
    unsigned int i, status = 0;
	if(connect_state == USBD_CDC_CONNECT_CONNECTED)
	{
		uint32_t t0 = HAL_GetTick();
		while (ep1_tx_flag) {
			if (HAL_GetTick() - t0 >= 50) {
				break;
			}
		}
    if((ep1_tx_flag == 0) && (infoLenth != 0))
    {
        ep1_Tx_dataLen = infoLenth;
        for(i = 0; i < ep1_Tx_dataLen; i++)
        {
            ep1_Tx_dataBuf[i] = *(Info + i);
        }
        ep1_tx_flag = 1;
    }
    else
    {
        status = 1;
    }
	}
  return status;
}

Buffer_t buffer;
unsigned char ep3_rx_flag = 0;				//�˵�3���ձ�־
/********************************************************************************************************
**������Ϣ ��EP3_USB_OUT_Data(void)
**�������� ��USB_OUT�˵���յ���Ч����,���ձ�־λ��1
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
static uint8_t rx_flag = 0;

void EP3_USB_OUT_Data(void)      			// bRequest = 0x22
{
	
	uint8_t usb_buf[CDC_MAX_PACKET_SIZE];
	bool buf_len = 1;
	unsigned int i, temp_len=0;
	
	temp_len = USB->rEP3_AVIL & 0x7f;

	if(temp_len)					//����buf�ǿ�
	{
		int data;
		uint16_t cont_len = 0;					
		for(i = 0; i < temp_len; i++)
		{
			data = USB->rEP3_FIFO;
			if (data == mp_interrupt_char) {
				pendsv_kbd_intr();
			} else {
				usb_buf[cont_len++] = data;
			}
		}

		buf_len = Buffer_Puts(&buffer, usb_buf, cont_len);
		if(!buf_len)	
		{
			Buffer_Clear(&buffer);
			Buffer_Puts(&buffer, usb_buf, cont_len);
		}
		ep3_rx_flag = 0;
		rx_flag = 1;
	}
	
	usb_running_ctrl_state = USB_IDLE;
}

/********************************************************************************************************
**������Ϣ ��UsbVcomRec(unsigned char*rxInfo)
**�������� ����USB device�� OUT�˵��������
**������� ��unsigned char*rxInfo(������Ϣbuf)
**������� ��
**��    ע ��
********************************************************************************************************/
unsigned int UsbVcomRec(unsigned char*rxInfo)
{
    unsigned int i, temp_len;
    if(ep3_rx_flag == 1)					//����buf�ǿ�
    {
			pendsv_kbd_intr();
			temp_len = USB->rEP3_AVIL & 0x7f;
			for(i = 0; i < temp_len; i++)
			{
					*(rxInfo + i) = USB->rEP3_FIFO;
			}
			ep3_rx_flag = 0;
    }
    else
    {
        return 0;
    }
    return temp_len;
}
//--------------------------------------------------------
int usb_vcom_recv(uint8_t *rxbuf , uint32_t len)
{
	//enable_irq(irq_state);
	if(rx_flag)
	{
		if(!Buffer_Gets(&buffer, rxbuf, len))
		{
			rx_flag = 0;
		}else return 1;
	}

	return 0;
}

//--------------------------------------------------------
int usb_cdc_tx_is_empty(void) {
  return ep1_tx_flag ? 0 : 1;
}
int usb_cdc_rx_is_empty(void) {
	int start  =  ep3_rx_flag;
	ep3_rx_flag = 0;
  return start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
bool usb_vcp_is_enabled(void) {
    return 1;
}
void usb_fifo_init(void)
{
    uint8_t* buff = m_new(uint8_t, RX_BUFF_SIZE);
    Buffer_Init(&buffer, buff, RX_BUFF_SIZE);
	//	mp_hal_set_interrupt_char(3);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*-------------------------(C) COPYRIGHT 2016 HOLOCENE ----------------------*/
