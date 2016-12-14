#ifndef __SCI_QUEUE_H
#define __SCI_QUEUE_H

#define QUEUE_MAX_SIZE 200//串口接收缓存长度
#define CMD_MAX_BUFFER 30//一条指令的最大长度
#define _UARTa 0x01  //因为STM32原来的程序里定义过USART，为了避免重定义，所以前后加横杠
#define _UARTb 0x02  //
#define WAIT_FRAME1  0
#define WAIT_OVER    1
#define WAIT_DATA    2
#define GET_CMD_OK   3

typedef struct _QUEUE
{
    unsigned char _head;  //队首
    unsigned char _tail;  //队尾
    unsigned char _data[QUEUE_MAX_SIZE];  //队列空间
} QUEUE;//串口数据接收队列结构体

typedef struct _SERIALCMD
{
	  unsigned char  cmd_status;    //接收指令状态标志位
    unsigned char  cmd_buffer_pointer;  //cmd数组的计数指针，因为不一定调用一次取CMD函数就能取出一帧指令，所以这个变量必须是全局变量，在取出一帧后，清0.
    unsigned char  cmd_buffer_R[CMD_MAX_BUFFER];//串口接收指令数组
    unsigned char  cmd_buffer_T[CMD_MAX_BUFFER];//串口指令发送数组
} SERIALCMD;//串口指令结构体

extern QUEUE UARTa_queue;    //声明串口1队列
extern QUEUE UARTb_queue;    //声明串口1队列
extern SERIALCMD UARTa_cmd ;//声明串口1的指令结构体
extern SERIALCMD UARTb_cmd ;//声明串口1的指令结构体

void Uart_Queue_Init(QUEUE *UARTX_queue);
void Uart_Queue_pushAchar(QUEUE *UARTX_queue,unsigned char data);
void Uart_Queue_popAchar(QUEUE *UARTX_queue,unsigned char *data);
unsigned char Uart_Queue_size(QUEUE *UARTX_queue);
unsigned char Uart_Queue_getAcmd(SERIALCMD *UARTX_cmd,QUEUE *UARTX_queue);
#endif
