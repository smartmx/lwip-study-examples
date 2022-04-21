/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*******************************************************************************/
#include "debug.h"
#include "tiny-macro-os.h"
#include "lwip_task.h"
#include "main.h"
#include "tcpecho.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    Delay_Init();
    USART_Printf_Init(115200);

    /* setup timer for tiny-macro-os and lwip sys_now */
    NVIC_SetPriority(SysTicK_IRQn,0xff);
    NVIC_EnableIRQ(SysTicK_IRQn);
    SysTick->CTLR= 0;
    SysTick->SR  = 0;
    SysTick->CNT = 0;
    SysTick->CMP = SystemCoreClock / OS_SEC_TICKS;
    SysTick->CTLR= 0xf;

    OS_INIT_TASKS();

    printf("Enter main loop.\n");
    while(1)
    {
        OS_RUN_TASK(os_lwip);
    }
}

/* dhcp分配成功回调，用户在此增加关于网络进程的初始化函数 */
void lwip_dhcp_success_callback()
{
    TCP_Echo_Init();
}
