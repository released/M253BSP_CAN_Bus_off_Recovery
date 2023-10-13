/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "misc_config.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TIMER_PERIOD_SPECIFIC                 (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_TIMER_PERIOD_BUS_OFF    				(flag_PROJ_CTL.bit2)
#define FLAG_PROJ_BUS_OFF_TRIGGER                       (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned int counter_systick = 0;
volatile uint32_t counter_tick = 0;

#define BAUDRATE_MIN        500000
#define BAUDRATE_MAX        1000000
#define BAUDRATE_OFFSET     50000
#define CAN_MON_DISABLE     0
#define CAN_MON_ENABLE      1

CANFD_FD_MSG_T      g_sRxMsgFrame;

uint32_t u32CanBitRate = BAUDRATE_MIN;
uint32_t u32RealCanBitRate = 0;
CANFD_FD_MSG_T      sRxFrame;
CANFD_FD_MSG_T      sTxMsgFrame;

uint8_t canbus_INT_flag = 0x00;

#define CAN_BOFF_INT        5

uint32_t u32TimeOutCount = __HSI;

uint8_t canbus_BO_COUNT = 0;


#define CAN_BO_FAST_RESP        (1)
#define CAN_BO_SLOW_RESP        (2)
#define CAN_BO_DEFAULT_RESP     (3)

uint8_t canbus_off_state = CAN_BO_FAST_RESP;

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/
void CAN_ShowMsg(CANFD_FD_MSG_T * sRxMsg);
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len);

unsigned int get_systick(void)
{
	return (counter_systick);
}

void set_systick(unsigned int t)
{
	counter_systick = t;
}

void systick_counter(void)
{
	counter_systick++;
}

void SysTick_Handler(void)
{

    systick_counter();

    if (get_systick() >= 0xFFFFFFFF)
    {
        set_systick(0);      
    }

    // if ((get_systick() % 1000) == 0)
    // {
       
    // }

    #if defined (ENABLE_TICK_EVENT)
    TickCheckTickEvent();
    #endif    
}

void SysTick_delay(unsigned int delay)
{  
    
    unsigned int tickstart = get_systick(); 
    unsigned int wait = delay; 

    while((get_systick() - tickstart) < wait) 
    { 
    } 

}

void SysTick_enable(unsigned int ticks_per_second)
{
    set_systick(0);
    if (SysTick_Config(SystemCoreClock / ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while (1);
    }

    #if defined (ENABLE_TICK_EVENT)
    TickInitTickEvent();
    #endif
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

// void delay_ms(uint16_t ms)
// {
// 	TIMER_Delay(TIMER0, 1000*ms);
// }

/*
    Fast : send per 50 ms , loop until 1 sec
    Slow : after 1 sec , send per 1 sec

    Normal : 200 ms
*/

uint8_t Get_CAN_PSC_BO(void)
{
    /*
        0 = The CAN FD controller is not Bus_Off. 
        1 = The CAN FD controller is in Bus_Off state. 
    */
    uint8_t res = 0;

    res = CANFD0->PSR & 0x80 ? TRUE : FALSE ;

    return res;
}

void Bus_off_recovery(void)
{
    // uint8_t result = 0;
    
    if (FLAG_PROJ_TIMER_PERIOD_BUS_OFF && 
        FLAG_PROJ_BUS_OFF_TRIGGER ) // 50 ms
    {
        FLAG_PROJ_TIMER_PERIOD_BUS_OFF = 0;

        if (canbus_BO_COUNT >= 20)
        {        
            if (((canbus_BO_COUNT) % 20) == 0)
            {
                canbus_off_state = CAN_BO_SLOW_RESP;
                CAN_SendMessage(&sTxMsgFrame,eCANFD_SID,0x666,8);
                FLAG_PROJ_BUS_OFF_TRIGGER = 0;
            }	
        }
        else
        {
            if (canbus_off_state == CAN_BO_FAST_RESP)
            {
                CAN_SendMessage(&sTxMsgFrame,eCANFD_SID,0x555,8);
                printf("FAST :%d\r\n",canbus_BO_COUNT);
                FLAG_PROJ_BUS_OFF_TRIGGER = 0;
            }
        }
        canbus_BO_COUNT++;

        if ( (CANFD0->PSR & 0x05) == 0x00)
        {
            FLAG_PROJ_BUS_OFF_TRIGGER = 0;            
            canbus_off_state = CAN_BO_FAST_RESP;
            canbus_BO_COUNT = 0;

            printf("BOFF:clr\r\n");
        }

    }    

	if (canbus_INT_flag == CAN_BOFF_INT)
    {
        // printf("BOFF:entry\r\n");
        canbus_INT_flag = 0;

        /*   bus-off recovery check*/
        #if 1
        CANFD0->CCCR |= ( CANFD_CCCR_INIT_Msk);

        // while((CANFD0->PSR&0x07)!=0x00);
        // prevent stuck in loop
        // u32TimeOutCount = 0xFFFF;   //__HSI;
        // while ( (CANFD0->PSR & 0x05) == 0x05)
        // {
        //     // printf("BOFF:101 = Bit0Error(1) \r\n");
        //     if (--u32TimeOutCount == 0)
        //     {
        //         // printf("BOFF:101 = Bit0Error(2) - TIMEOUT\r\n");
        //         break;
        //     }
        // }
        
        CANFD0->CCCR &= ~( CANFD_CCCR_INIT_Msk);
        CANFD_RunToNormal(CANFD0, TRUE);  


        #else
        CANFD0->CCCR &= (~(CANFD_CCCR_INIT_Msk | CANFD_CCCR_CCE_Msk));
        while(CANFD0->CCCR & CANFD_CCCR_INIT_Msk);
        while((CANFD0->PSR & CANFD_PSR_LEC_Msk) == 0x05) 
        {
            if(CANFD0->ECR ==0) 
            {
                CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_DRXE_Msk | CANFD_IE_BOE_Msk  |CANFD_IE_PEAE_Msk ), 0, 0, 0);
                
                canbus_INT_flag = 0x00;
                break;
            }
        }
        #endif
	}	
}

/*---------------------------------------------------------------------------------------------------------*/
/*  ISR to handle CAN FD Line 0 interrupt event                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD0_IRQ0_IRQHandler(void)
{
//    printf("IR =0x%08X \n", CANFD0->IR);

    // if(CANFD0->IR  & CANFD_IR_TC_Msk)
    // {
    //     printf("Transmission Completed\n") ;
    //     CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TC_Msk);
    // }

    if(CANFD0->IR & CANFD_IR_BO_Msk)
    {
        // printf("BOFF INT\n") ;
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_BO_Msk);
        canbus_INT_flag = CAN_BOFF_INT;

        FLAG_PROJ_BUS_OFF_TRIGGER = 1;
    }

    // if(CANFD0->IR  & CANFD_IR_EW_Msk)
    // {
    //     // printf("EWARN INT\n") ;
    //     CANFD_ClearStatusFlag(CANFD0, CANFD_IR_EW_Msk);
    // }

    /* Protocol Error in Arbitration Phase */
    // if(CANFD0->IR & CANFD_IR_PEA_Msk)
    // {
    //     // printf(" Protocol error INT\n");
    //     CANFD_ClearStatusFlag(CANFD0, CANFD_IR_PEA_Msk);
    // }

    // if(CANFD0->IR & CANFD_IR_DRX_Msk )
    // {
    //     /*Clear the Interrupt flag */
    //     CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_DRX_Msk);

    //     /*Receive the Rx buffer message(Standard ID) */
    //     if(CANFD_ReadRxBufMsg(CANFD0, 0, &g_sRxMsgFrame) == 1)
    //     {
    //         CAN_ShowMsg(&g_sRxMsgFrame);
    //     }
    //     /*Receive the Rx buffer message(Standard ID) */
    //     if(CANFD_ReadRxBufMsg(CANFD0, 1, &g_sRxMsgFrame) == 1)
    //     {
    //         CAN_ShowMsg(&g_sRxMsgFrame);
    //     }
    // }
}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(CANFD_FD_MSG_T * sRxMsg)
{
   uint8_t u8Cnt;
   /* Show the message information */
   if(sRxMsg->eIdType == eCANFD_SID)
   printf("Rx buf 0: ID = 0x%08X(11-bit),DLC = %d\n", sRxMsg->u32Id,sRxMsg->u32DLC);
   else
   printf("Rx buf 1: ID = 0x%08X(29-bit),DLC = %d\n", sRxMsg->u32Id,sRxMsg->u32DLC);

   printf("Message Data : ");
   for (u8Cnt = 0; u8Cnt < sRxMsg->u32DLC; u8Cnt++)
   {
     printf("%02X ,", sRxMsg->au8Data[u8Cnt]);
   }
    printf("\n\n");
}
/*---------------------------------------------------------------------------*/
/*  Get the CAN interface bit rate Function                                  */
/*---------------------------------------------------------------------------*/
uint32_t Get_CAN_BitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;
    if(CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos) )
     u32CanClk = CLK_GetHCLKFreq();
    else
     u32CanClk = CLK_GetHXTFreq();

    u32CanDiv = ((CLK->CLKDIV4 & CLK_CLKDIV4_CANFD0DIV_Msk) >> CLK_CLKDIV4_CANFD0DIV_Pos) + 1;
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->NBTP & CANFD_NBTP_NBRP_Msk) >> CANFD_NBTP_NBRP_Pos)+1 ;
    u8NtSeg1 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG1_Msk) >> CANFD_NBTP_NTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG2_Msk) >> CANFD_NBTP_NTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1+u8NtSeg2+3); 

    return u32BitRate;
}
/*---------------------------------------------------------------------------*/
/*  Send CAN bus Message Function                                            */
/*---------------------------------------------------------------------------*/
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;
    uint8_t resp;
    
    psTxMsg->u32Id = u32Id;
    psTxMsg->eIdType = eIdType;
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    psTxMsg->bBitRateSwitch = 0;
    psTxMsg->u32DLC = u8Len;

    /* use message buffer 0 */
    if (eIdType == eCANFD_SID)
        printf("Send ID = 0x%08x (11-bit),DLC = %d\n", psTxMsg->u32Id,psTxMsg->u32DLC);
    else
        printf("Send ID = 0x%08x (29-bit),DLC = %d\n", psTxMsg->u32Id,psTxMsg->u32DLC);

    printf("Message Data : ");
    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) 
    {
      psTxMsg->au8Data[u8Cnt] = u8Cnt;
      printf("%02X ,", psTxMsg->au8Data[u8Cnt]);
    }
    // printf("\n\n");
    resp = CANFD_TransmitTxMsg(CANFD0, 0, psTxMsg);
    if (resp == 1)
    {
        printf("transmit message OK\n");
    }
    else
    {
        printf("Failed to transmit message (0x%2X)\n",resp);
    }
}


void CAN_Init(uint32_t u32BitRate,uint8_t u8EnMonMode)
{
    CANFD_FD_T sCANFD_Config;

    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = u32BitRate;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    CANFD_Open(CANFD0, &sCANFD_Config);
    printf("Set baud-rate value(bps): %d\n", sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate);
    printf("Real baud-rate value(bps): %d\n\n", Get_CAN_BitRate(CANFD0));
    /* receive 0x111 in CAN rx message buffer 0 by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));

    /* receive 0x222 (29-bit id) in CAN rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));

    // /*Enable the Bus Monitoring Mode */
    // if(u8EnMonMode == 1)
    // CANFD0->CCCR |= CANFD_CCCR_MON_Msk;
    // else
    // CANFD0->CCCR &= ~CANFD_CCCR_MON_Msk;

    /* Enable RX buffer new message ,Timeout,Protocol Error,Bus_Off interrupt using interrupt line 0. */
    // CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_DRXE_Msk | CANFD_IE_BOE_Msk  |CANFD_IE_PEAE_Msk ), 0, 0, 0);    
    CANFD_EnableInt(CANFD0, CANFD_IE_BOE_Msk , 0, 0, 0);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


void TMR1_IRQHandler(void)
{	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
		tick_counter();

		if ((get_tick() % 1000) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_1000MS = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}
		if ((get_tick() % 200) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 1;//set_flag(flag_timer_period_1000ms ,ENABLE);
		}

		if ((get_tick() % 50) == 0)
		{
            FLAG_PROJ_TIMER_PERIOD_BUS_OFF = 1;
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

void loop(void)
{
	// static uint32_t LOG1 = 0;
	// static uint32_t LOG2 = 0;
	// static uint32_t counter = 0;
    static uint8_t flag_ctrl = 0;

    if ((get_systick() % 1000) == 0)
    {
        // printf("%s(systick) : %4d\r\n",__FUNCTION__,LOG2++);    
    }

    if (FLAG_PROJ_TIMER_PERIOD_1000MS)//(is_flag_set(flag_timer_period_1000ms))
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 0;//set_flag(flag_timer_period_1000ms ,DISABLE);

        // printf("%s(timer) : %4d\r\n",__FUNCTION__,LOG1++);
        PB0 ^= 1;        
    }

    if ((FLAG_PROJ_TIMER_PERIOD_SPECIFIC) && 
        (!FLAG_PROJ_BUS_OFF_TRIGGER) && 
        ( Get_CAN_PSC_BO() == 0x00) )
    {
        FLAG_PROJ_TIMER_PERIOD_SPECIFIC = 0;

        if(canbus_INT_flag != CAN_BOFF_INT) 
        {
            if (flag_ctrl)
            {
                flag_ctrl = 0;
                CAN_SendMessage(&sTxMsgFrame,eCANFD_SID,0x333,8);
            }
            else 
            {
                flag_ctrl = 1;
                CAN_SendMessage(&sTxMsgFrame,eCANFD_XID,0x444,8);

            }
        }

    }

    Bus_off_recovery();
}

void UARTx_Process(void)
{
	uint8_t res = 0;
	res = UART_READ(UART4);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		printf("press : %c\r\n" , res);
		switch(res)
		{
			case '1':
				break;

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
                SYS_UnlockReg();
				// NVIC_SystemReset();	// Reset I/O and peripherals , only check BS(FMC_ISPCTL[1])
                // SYS_ResetCPU();     // Not reset I/O and peripherals
                SYS_ResetChip();    // Reset I/O and peripherals ,  BS(FMC_ISPCTL[1]) reload from CONFIG setting (CBS)	
				break;
		}
	}
}

void UART4_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(UART4, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART4) == 0)
        {
			UARTx_Process();
        }
    }

    if(UART4->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART4, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }	
}

void UART4_Init(void)
{
    SYS_ResetModule(UART4_RST);

    /* Configure UART4 and set UART4 baud rate */
    UART_Open(UART4, 115200);
    UART_EnableInt(UART4, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
    NVIC_EnableIRQ(UART4_IRQn);
	
	#if (_debug_log_UART_ == 1)	//debug
	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHCLKFreq : %8d\r\n",CLK_GetHCLKFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
	#endif	

    #if 0
    printf("FLAG_PROJ_TIMER_PERIOD_1000MS : 0x%2X\r\n",FLAG_PROJ_TIMER_PERIOD_1000MS);
    printf("FLAG_PROJ_REVERSE1 : 0x%2X\r\n",FLAG_PROJ_REVERSE1);
    printf("FLAG_PROJ_REVERSE2 : 0x%2X\r\n",FLAG_PROJ_REVERSE2);
    printf("FLAG_PROJ_REVERSE3 : 0x%2X\r\n",FLAG_PROJ_REVERSE3);
    printf("FLAG_PROJ_REVERSE4 : 0x%2X\r\n",FLAG_PROJ_REVERSE4);
    printf("FLAG_PROJ_REVERSE5 : 0x%2X\r\n",FLAG_PROJ_REVERSE5);
    printf("FLAG_PROJ_REVERSE6 : 0x%2X\r\n",FLAG_PROJ_REVERSE6);
    printf("FLAG_PROJ_REVERSE7 : 0x%2X\r\n",FLAG_PROJ_REVERSE7);
    #endif

}

void GPIO_Init (void)
{
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | (SYS_GPB_MFPL_PB0MFP_GPIO);

    GPIO_SetMode(PB, BIT0, GPIO_MODE_OUTPUT);

}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
    
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    CLK_EnableModuleClock(GPB_MODULE);

    CLK_EnableModuleClock(TMR1_MODULE);
  	CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HXT, 0);

    
    CLK_EnableModuleClock(TMR3_MODULE);
  	CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3SEL_HXT, 0);

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HXT, CLK_CLKDIV4_CANFD0(1));
    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    // SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB12MFP_Msk)) | SYS_GPB_MFPH_PB12MFP_CAN0_RXD;
    // SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB13MFP_Msk)) | SYS_GPB_MFPH_PB13MFP_CAN0_TXD;

    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC4MFP_Msk)) | SYS_GPC_MFPL_PC4MFP_CAN0_RXD;
    SYS->GPC_MFPL = (SYS->GPC_MFPL & (~SYS_GPC_MFPL_PC5MFP_Msk)) | SYS_GPC_MFPL_PC5MFP_CAN0_TXD;

    /* Debug UART clock setting */
    // UartDebugCLK();
    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HXT, CLK_CLKDIV4_UART4(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART4_MODULE);    

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UartDebugMFP();


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M251 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART4 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    SYS_Init();

	GPIO_Init();
	UART4_Init();
	TIMER1_Init();

    SysTick_enable(1000);
    #if defined (ENABLE_TICK_EVENT)
    TickSetTickEvent(1000, TickCallback_processA);  // 1000 ms
    TickSetTickEvent(5000, TickCallback_processB);  // 5000 ms
    #endif

    /* CAN interface initialization*/
    CAN_Init(u32CanBitRate,CAN_MON_DISABLE);


    /* Got no where to go, just loop forever */
    while(1)
    {
        loop();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
