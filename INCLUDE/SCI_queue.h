#ifndef __SCI_QUEUE_H
#define __SCI_QUEUE_H

#define QUEUE_MAX_SIZE 200//���ڽ��ջ��泤��
#define CMD_MAX_BUFFER 30//һ��ָ�����󳤶�
#define _UART1 0x01  //��ΪSTM32ԭ���ĳ����ﶨ���USART��Ϊ�˱����ض��壬����ǰ��Ӻ��
#define _UART2 0x02  //
#define WAIT_FRAME1  0
#define WAIT_FRAME2  1
#define WAIT_DATA    2
#define GET_CMD_OK   3

typedef struct _QUEUE
{
    unsigned char _head;  //����
    unsigned char _tail;  //��β
    unsigned char _data[QUEUE_MAX_SIZE];  //���пռ�
} QUEUE;//�������ݽ��ն��нṹ��

typedef struct _SERIALCMD
{
	  unsigned char  cmd_status;    //����ָ��״̬��־λ
    unsigned char  cmd_buffer_pointer;  //cmd����ļ���ָ�룬��Ϊ��һ������һ��ȡCMD��������ȡ��һָ֡������������������ȫ�ֱ�������ȡ��һ֡����0.
    unsigned char  cmd_buffer_R[CMD_MAX_BUFFER];//���ڽ���ָ������
    unsigned char  cmd_buffer_T[CMD_MAX_BUFFER];//����ָ�������
} SERIALCMD;//����ָ��ṹ��

extern QUEUE UART1_queue;    //��������1����
extern QUEUE UART2_queue;    //��������1����
extern SERIALCMD UART1_cmd ;//��������1��ָ��ṹ��
extern SERIALCMD UART2_cmd ;//��������1��ָ��ṹ��

void Uart_Queue_Init(QUEUE *UARTX_queue);
void Uart_Queue_pushAchar(QUEUE *UARTX_queue,unsigned char data);
void Uart_Queue_popAchar(QUEUE *UARTX_queue,unsigned char *data);
unsigned char Uart_Queue_size(QUEUE *UARTX_queue);
unsigned char Uart_Queue_getAcmd(SERIALCMD *UARTX_cmd,QUEUE *UARTX_queue);
#endif