/**
******************************************************************************
* @file    usbreg.c
* @author  IC Applications Department
* @version  V0.8
* @date  2019_08_02
* @brief   This file provides all the usbreg firmware functions.
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
#include "usb.h"
#include "HAL_device.h"
#include "stdio.h"

#include "usbprop.h"
volatile uint8_t USB_EP_STALL[5];
volatile uint8_t USB_EP_IN_STALL[5];
volatile uint8_t USB_EP_OUT_STALL[5];
//volatile uint8_t usb_setup_flag;
//volatile uint8_t usb_out_flag;
//volatile uint8_t usb_in_flag;
volatile uint8_t USB_FINISH_Flag[5];
//volatile uint8_t usb_reset_flag;
//volatile uint8_t usb_suspend_flag;
//volatile uint8_t usb_resume_flag;
//volatile uint8_t usb_state_suspend = 0;
//volatile uint8_t usb_state_resume = 0;
volatile USB_STATE_t usb_state;
volatile uint8_t usb_ep_flag;
//
volatile uint8_t USB_SEND_OPEN_STALL[5];
volatile uint32_t irq_state = 0;
/********************************************************************************************************
**������Ϣ ��USB_HP_CAN1_TX_IRQHandler(void)
**�������� ��USB�жϴ�����
**������� ��
**������� ��
**��    ע ��
********************************************************************************************************/
#include "py/mphal.h"
void USB_CDC_IRQHandler(void)
{
    uint32_t temp;
    uint32_t temp_ep;
    uint32_t temp_epn;
    temp = USB->rINT_STATE ;				//��ȡUSB�ж�״̬
    USB->rINT_STATE = temp;					//��USB�ж�״̬,д1����
    if(temp & USB_INT_STATE_EPINTF)			//�˵��ж�
    {
        temp_ep = USB->rEP_INT_STATE;			//��ȡ�˵��жϺ� 0x1,0x2,0x4,0x8,0x10�ֱ��Ӧ0,1,2,3,4�˵��ж�

        USB->rEP_INT_STATE = temp_ep;		//��˵���ж�
        if(temp_ep & EP_INT_STATE_EP0F )		//�˵�0�ж�
        {
            //GPIO_SetBits(GPIOA, GPIO_Pin_5);//���ڴ���
            usb_ep_flag = USB_EP0;			//��¼�˵��
            temp_epn = USB->rEP0_INT_STATE;	//��ȡ�˵�0�ж�״̬λ

            USB->rEP0_INT_STATE = temp_epn; //��˵�0�ж�

            if(temp_epn & EPn_INT_STATE_SETUP) //SETUP�ж�
            {
              usb_setup_handle();			//setup������,������ƴ���setup�׶�
            }
            if(temp_epn & EPn_INT_STATE_OUTACK)	//�˵�0 OUT��Ӧ���жϣ��յ�����
            {
              usb_out_handle();				//OUT������
            }
            if(temp_epn & EPn_INT_STATE_INNACK)	//IN����Ӧ���жϣ�׼��д������
            {
              usb_in_handle();				//IN������
              USB->rEP0_INT_STATE |= EPn_INT_STATE_INNACK;//��˵��ж�
            }

            if(temp_epn & (EPn_INT_EN_OUTSTALLIE | EPn_INT_EN_INSTALLIE)) //�˵�رգ����Լ����Զ��庯��
            {

                USB_EP_STALL[0] = 1;
            }
            if(temp_epn & EPn_INT_STATE_INSTALL)
            {
                USB_EP_IN_STALL[0] = 1;
            }
            if(temp_epn & EPn_INT_STATE_OUTSTALL)
            {
                USB_EP_OUT_STALL[0] = 1;
            }
        }
#if ((EPOUT!=EPIN))	 							//if epin != epout,
        if(temp_ep & EPn_INT(EPOUT))				//����˵���ж�,ͬ����˵�0�ж�,�ж��Ƿ�Ϊ����˵�(�������ö˵�3Ϊ����˵�)�ж�
        {
            usb_ep_flag = USB_EPn(EPOUT);

            temp_epn = USB->rEP3_INT_STATE;
            USB->rEP3_INT_STATE = temp_epn;
            if(temp_epn & EPn_INT_STATE_OUTACK)	//OUT��Ӧ���ж�
            {
							//irq_state = disable_irq();
							usb_out_handle();
						//	printf("q\r\n");
							//printf("EPn_INT_STATE_OUTACK\r\n");
							//EP3_USB_OUT_Data();
            }
            if(temp_epn & EPn_INT_STATE_END)		//��������ж�
            {
                USB_FINISH_Flag[EPOUT] = 1;
            }
            if(temp_epn & (EPn_INT_EN_OUTSTALLIE | EPn_INT_EN_INSTALLIE))
            {
                USB_EP_STALL[EPOUT] = 1;
            }
            if(temp_epn & EPn_INT_STATE_INSTALL)
            {
                USB_EP_IN_STALL[EPOUT] = 1;
            }
            if(temp_epn & EPn_INT_STATE_OUTSTALL)
            {
                USB_EP_OUT_STALL[EPOUT] = 1;
            }
        }
#endif
        if(temp_ep & EPn_INT(EPIN))
        {
            usb_ep_flag = USB_EPn(EPIN);
            temp_epn = USB->rEP1_INT_STATE;
            USB->rEP1_INT_STATE = temp_epn;
            if(temp_epn & EPn_INT_STATE_INNACK)	//IN����Ӧ���ж�
            {
                usb_in_handle();
            }
            if(temp_epn & EPn_INT_STATE_END)
                USB_FINISH_Flag[EPIN] = 1;
            if(temp_epn & (EPn_INT_EN_OUTSTALLIE | EPn_INT_EN_INSTALLIE))
            {
                USB_EP_STALL[EPIN] = 1;
            }
            if(temp_epn & EPn_INT_STATE_INSTALL)
            {
                USB_EP_IN_STALL[EPIN] = 1;
            }
            if(temp_epn & EPn_INT_STATE_OUTSTALL)
            {
                USB_EP_OUT_STALL[EPIN] = 1;
            }
        }
    }
    else if(temp & USB_INT_STATE_RSTF)			//USB��λ�ж�
    {
        usb_reset_handle();
    }
    else if(temp & USB_INT_STATE_SUSPENDF )	//USB�����ж�,�Զ����д
    {
        usb_suspend_handle();
    }
    else if(temp & USB_INT_STATE_RESUMF  )		//USB�ָ��ж�,�Զ����д
    {
        usb_resume_handle();
    }
}



/********************************************************************************************************
**������Ϣ ��Read_Mreg32( uint32_t mreg)
**�������� ��ָ����ַ��32bit����
**������� ��uint32_t mreg(��ַ)
**������� ��return(*(volatile uint32_t *)mreg);����
**��    ע ��
********************************************************************************************************/
uint32_t Read_Mreg32( uint32_t mreg)
{
    return(*(volatile uint32_t *)mreg);
}
/********************************************************************************************************
**������Ϣ ��Read_Mreg32( uint32_t mreg)
**�������� ��ָ����ַд32bit����
**������� ��uint32_t mreg(��ַ), uint32_t val(����)
**������� ��
**��    ע ��
********************************************************************************************************/
void Write_Mreg32( uint32_t mreg, uint32_t val)
{
    *(volatile uint32_t *)mreg = (uint32_t)val;
}

/********************************************************************************************************
**������Ϣ ��Read_Mreg32( uint32_t mreg)
**�������� ��ָ����ַ��8bit����
**������� ��uint32_t mreg(��ַ)
**������� ��
**��    ע ��
********************************************************************************************************/
uint8_t read_mreg8( uint32_t mreg)
{
    return(*(volatile uint8_t *)mreg);
}
/********************************************************************************************************
**������Ϣ ��write_mreg8( uint32_t mreg, uint8_t val)
**�������� ��ָ����ַд8bit����
**������� ��uint32_t mreg(��ַ), uint8_t val(����)
**������� ��
**��    ע ��
********************************************************************************************************/
void write_mreg8( uint32_t mreg, uint8_t val)
{
    *(volatile uint8_t *)mreg = (uint8_t)val;
}
/********************************************************************************************************
**������Ϣ ��usb_delay1ms(uint32_t dly)
**�������� ��usb�ڲ���ʱ����ʼ�������Ѳ���
**������� ��uint32_t dly(��ʱ����)
**������� ��
**��    ע ��
********************************************************************************************************/
void usb_delay1ms(uint32_t dly)
{
    uint32_t cnt, i, j, k;

    for(cnt = 0; cnt < dly; cnt++)
    {
        for(i = 0; i < 24; i++)
        {
            for(j = 0; j < 2; j++)
            {
                for(k = 0; k < 100; k++);
            }
        }
    }

}


/*-------------------------(C) COPYRIGHT 2016 HOLOCENE ----------------------*/
