/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R							(PH0)
#define LED_Y							(PH1)
#define LED_G							(PH2)

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) ((DATA) * (VREF) / ADC_DIGITAL_SCALE())

uint16_t aADCxConvertedData[4] = {0};

uint32_t Vgap = 0;
uint32_t Vtemp = 0;
uint32_t Vbat = 0;


enum
{
	ADC0_CH0 = 0 ,
	ADC0_CH1 ,
	ADC0_CH2 , 
	ADC0_CH3 , 
	ADC0_CH4 ,
	ADC0_CH5 , 
	ADC0_CH6 , 
	ADC0_CH7 ,
	ADC0_CH8 , 
	ADC0_CH9 , 
	ADC0_CH10 , 
	ADC0_CH11 ,
	ADC0_CH12 , 
	ADC0_CH13 , 
	ADC0_CH14 ,
	ADC0_CH15 , 
	
	ADC0_CH16_BAND_GAP_VOLT , 
	ADC0_CH17_TEMP_SENSOR ,
	ADC0_CH18_VBAT , 
	
	ADC_CH_DEFAULT 	
}ADC_CH_TypeDef;

typedef enum{

	flag_ADC_Data_Ready ,	
	flag_ADC_Channel_Change ,
	
	flag_DEFAULT	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))

/*****************************************************************************/

void EADC00_IRQHandler(void)
{
    set_flag(flag_ADC_Data_Ready , ENABLE);
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

void EADC02_IRQHandler(void)
{
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
}

void ADC_Read_Int_Channel(void)
{
	
   /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 16 external sampling time to 0xF */
    EADC_SetExtendSampleTime(EADC, ADC0_CH16_BAND_GAP_VOLT, 0x3F);
    EADC_SetExtendSampleTime(EADC, ADC0_CH17_TEMP_SENSOR, 0x3F);

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF2_Msk);
    EADC_ENABLE_INT(EADC, (BIT0 << 2));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH16_BAND_GAP_VOLT));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH17_TEMP_SENSOR));
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 2, (BIT0 << ADC0_CH18_VBAT));
	
    NVIC_EnableIRQ(EADC02_IRQn);

    EADC_START_CONV(EADC, (BIT0 << ADC0_CH16_BAND_GAP_VOLT) | (BIT0 << ADC0_CH17_TEMP_SENSOR)| (BIT0 << ADC0_CH18_VBAT));

    while(EADC_GET_DATA_VALID_FLAG(EADC, (BIT16|BIT17|BIT18)) != (BIT16|BIT17|BIT18));

    Vgap = EADC_GET_CONV_DATA(EADC, ADC0_CH16_BAND_GAP_VOLT);
    Vtemp = EADC_GET_CONV_DATA(EADC, ADC0_CH17_TEMP_SENSOR);
    Vbat = EADC_GET_CONV_DATA(EADC, ADC0_CH18_VBAT);
	
    EADC_DISABLE_INT(EADC, (BIT0 << 2));
 	NVIC_DisableIRQ(EADC02_IRQn);
	
}

/*
	ADC0_CH7
	ADC0_CH8
	ADC0_CH9
	ADC0_CH10
*/
void ADC_Convert_Ext_Channel(uint8_t ch)
{
	uint8_t i = 0;
	uint32_t ModuleMask = BIT0 ;		// use bit 0 as module ask
	uint32_t ModuleNum = 0 ;		// use ch 0 as module num
	
    /* Set input mode as single-end, and Single mode*/
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    EADC_SetExtendSampleTime(EADC, ch, 0x3F);
	
	EADC_ConfigSampleModule(EADC, ModuleNum, EADC_ADINT0_TRIGGER, ch);

    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);
	
    EADC_ENABLE_INT(EADC, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, (ModuleMask));
    NVIC_EnableIRQ(EADC00_IRQn);

	switch(ch)
	{
		case ADC0_CH7: 

			set_flag(flag_ADC_Data_Ready , DISABLE);				
			EADC_START_CONV(EADC, (ModuleMask));				
//			while(is_flag_set(flag_ADC_Data_Ready) == DISABLE);		
			while(EADC_GET_DATA_VALID_FLAG(EADC, (ModuleMask)) != (ModuleMask));
			
			aADCxConvertedData[0] = EADC_GET_CONV_DATA(EADC, ModuleNum);

			break;

		case ADC0_CH8: 

			set_flag(flag_ADC_Data_Ready , DISABLE);				
			EADC_START_CONV(EADC, (ModuleMask));						
//			while(is_flag_set(flag_ADC_Data_Ready) == DISABLE);		
			while(EADC_GET_DATA_VALID_FLAG(EADC, (ModuleMask)) != (ModuleMask));
			
			aADCxConvertedData[1] = EADC_GET_CONV_DATA(EADC, ModuleNum);

			break;

		case ADC0_CH9: 

			set_flag(flag_ADC_Data_Ready , DISABLE);				
			EADC_START_CONV(EADC, (ModuleMask));						
//			while(is_flag_set(flag_ADC_Data_Ready) == DISABLE);		
			while(EADC_GET_DATA_VALID_FLAG(EADC, (ModuleMask)) != (ModuleMask));
			
			aADCxConvertedData[2] = EADC_GET_CONV_DATA(EADC, ModuleNum);

			break;

		case ADC0_CH10: 

			set_flag(flag_ADC_Data_Ready , DISABLE);				
			EADC_START_CONV(EADC, (ModuleMask));						
//			while(is_flag_set(flag_ADC_Data_Ready) == DISABLE);
			while(EADC_GET_DATA_VALID_FLAG(EADC, (ModuleMask)) != (ModuleMask));
			
			aADCxConvertedData[3] = EADC_GET_CONV_DATA(EADC, ModuleNum);

			break;		
	}
	
	for (i = 0 ; i < 4; i++)
	{
		printf("(CH:%2d)0x%3X,%4dmv" , ch , aADCxConvertedData[i] , ADC_CALC_DATA_TO_VOLTAGE(aADCxConvertedData[i],3300));
	}
//	printf(",0x%3X,0x%3X,0x%3X" , Vgap , Vtemp, Vbat);
   
	printf("\r\n");

    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, (ModuleMask));
    EADC_DISABLE_INT(EADC, BIT0);
 	NVIC_DisableIRQ(EADC00_IRQn);
	
}


void TMR1_IRQHandler(void)
{
	static uint16_t CNT = 0;	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT++ > 1000)
		{		
			CNT = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
//			ADC_Convert_Ext_Channel(ADC0_CH7);

			set_flag(flag_ADC_Channel_Change , ENABLE);

			LED_R ^= 1;
		}
    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}


void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

//    CLK_EnableModuleClock(PDMA_MODULE);

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);

	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    PB->MODE &= ~(GPIO_MODE_MODE7_Msk | GPIO_MODE_MODE8_Msk | GPIO_MODE_MODE9_Msk | GPIO_MODE_MODE10_Msk);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk );
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB7MFP_EADC0_CH7);

    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk | SYS_GPB_MFPH_PB10MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB8MFP_EADC0_CH8 | SYS_GPB_MFPH_PB9MFP_EADC0_CH9 | SYS_GPB_MFPH_PB10MFP_EADC0_CH10);

    /* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT10|BIT9|BIT8|BIT7);

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

    /* Set reference voltage to external pin (3.3V) */
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	uint8_t state = ADC0_CH7;
	
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());


	LED_Init();
	TIMER1_Init();
//	ADC_Read_Int_Channel();

    /* Got no where to go, just loop forever */
    while(1)
    {
//		TIMER0_Polling(1000);
		LED_Y ^= 1;

		ADC_Convert_Ext_Channel(state);
		
		if (is_flag_set(flag_ADC_Channel_Change))
		{
			set_flag(flag_ADC_Channel_Change ,DISABLE);					
			state = (state >= ADC0_CH10) ? (ADC0_CH7) : (state+1) ;
		}	
    }

}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
