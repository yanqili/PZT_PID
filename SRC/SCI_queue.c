#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "SCI_queue.h"

QUEUE UARTa_queue = {0,0,0}; //定义串口1队列
QUEUE UARTb_queue = {0,0,0}; //定义串口1队列
SERIALCMD UARTa_cmd = {0,0,0,0};//定义串口1的指令结构体
SERIALCMD UARTb_cmd = {0,0,0,0};//定义串口1的指令结构体

/********************************
*名    称： Uart＿Queue_Init
*功    能： 串口队列初始化
*入口参数： 串口队列
*出口参数： 无
*********************************/
void Uart_Queue_Init(QUEUE *UARTX_queue)
{
   UARTX_queue->_head = UARTX_queue->_tail = 0;//清空队列
}

/********************************
*名    称： Uart＿Queue＿pushAchar
*功    能： 存入串口队列一个数据
*入口参数： 串口队列,数据
*出口参数： 无
*********************************/
void Uart_Queue_pushAchar(QUEUE *UARTX_queue,unsigned char data)
{
   UARTX_queue->_tail = (UARTX_queue->_tail+1)%QUEUE_MAX_SIZE;//将UARTX＿queue.tail限制在队列的长度范围内
   if(UARTX_queue->_tail!=UARTX_queue->_head)//非满状态
      UARTX_queue->_data[(UARTX_queue->_tail)%QUEUE_MAX_SIZE] = data;
}

/********************************
*名    称： Uart＿Queue＿popAchar
*功    能： 取出串口队列一个数据
*入口参数： 串口队列
*出口参数： 无
*********************************/
void Uart_Queue_popAchar(QUEUE *UARTX_queue,unsigned char *data)
{

   if(UARTX_queue->_tail!=UARTX_queue->_head)//非空状态
   {
    UARTX_queue->_head = (UARTX_queue->_head+1)%QUEUE_MAX_SIZE;
    *data = UARTX_queue->_data[UARTX_queue->_head];

    }
 }

/********************************
*名    称： Uart＿Queue＿size
*功    能： 算出串口队列的长度
*入口参数： 串口队列
*出口参数： 队列长度
*********************************/
unsigned char Uart_Queue_size(QUEUE *UARTX_queue)
{
   return ((UARTX_queue->_tail+QUEUE_MAX_SIZE-UARTX_queue->_head)%QUEUE_MAX_SIZE);
}


/********************************
*名    称： Uart＿Queue＿getAcmd
*功    能： 获取一帧完整的指令
*入口参数： 接收指令数组，串口队列
*出口参数： 0成功1失败
*********************************/

unsigned char Uart_Queue_getAcmd(SERIALCMD *UARTX_cmd,QUEUE *UARTX_queue)
{
    unsigned char cmd_data;

  while(Uart_Queue_size(UARTX_queue)>0)
    {
        //取一个数据
        Uart_Queue_popAchar(UARTX_queue,&cmd_data);

        switch(UARTX_cmd->cmd_status)
				{
					case WAIT_FRAME1:                  //等待帧首1  0x5A
						 if(cmd_data==0x5A)
						 {
						    UARTX_cmd->cmd_status = WAIT_DATA;
						 }

					break;

				    case WAIT_DATA:                  //找到了帧首，存入后面的数据
						  if(UARTX_cmd->cmd_buffer_pointer<6)
							{
                	UARTX_cmd->cmd_buffer_R[UARTX_cmd->cmd_buffer_pointer]=cmd_data;
               		UARTX_cmd->cmd_buffer_pointer++;
							}
						  if(UARTX_cmd->cmd_buffer_pointer==6)
						  {
							  UARTX_cmd->cmd_status = WAIT_OVER;

						  }
				    break;

				    case WAIT_OVER:

							  if(cmd_data==0x23)
							  {
								  UARTX_cmd->cmd_buffer_pointer=0;
								  UARTX_cmd->cmd_status=0;
								  return 1;   //得到了一个完整的帧
							  }
							  else
							  {
								 UARTX_cmd->cmd_status = WAIT_FRAME1;
								 UARTX_cmd->cmd_buffer_pointer=0;
							  }

					break;
				}
     }
		 return 0;//没有得到完整的帧
}
