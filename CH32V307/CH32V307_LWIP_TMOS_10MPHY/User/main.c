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
#include "lwip/apps/lwiperf.h"

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
    OS_TASK_EXIT_ANOTHER(os_lwip_timeouts);     /* 超时任务等待lwip主任务启动 */

    printf("Enter main loop.\n");
    while(1)
    {
        OS_RUN_TASK(os_lwip);
        OS_RUN_TASK(os_lwip_timeouts);
    }
}

/* IP分配成功回调，用户在此增加关于网络进程的初始化函数 */
void lwip_init_success_callback(ip_addr_t *ip)
{
    printf("本地IP地址是:%ld.%ld.%ld.%ld\n\n",  \
        ((ip->addr)&0x000000ff),       \
        (((ip->addr)&0x0000ff00)>>8),  \
        (((ip->addr)&0x00ff0000)>>16), \
        ((ip->addr)&0xff000000)>>24);
    TCP_Echo_Init();
    lwiperf_start_tcp_server(ip, 9527, NULL, NULL);
}
