
#ifndef __VOFA_H_
#define __VOFA_H_

#define  VOFA_MAX 255 
#define  UART_PutChar(byte)		wireless_uart_send_buff( byte, 4);

typedef union     
{
    float fdata;
    unsigned long ldata;
}FloatLongType1;


extern float vofa_send_data[VOFA_MAX];
void vodka_JustFloat_send(float *data_str,uint16 num);





//  @brief      vofa调参 vofa调参 vofa调参 vofa调参 vofa调参
extern uint8 RxBuffer[1];//串口接收缓冲
extern uint16 RxLine;//指令长度
extern uint8 DataBuff[200];//指令内容
// * 解析出DataBuff中的数据
float Get_Data(void);

//根据串口信息进行PID调参
void USART_PID_Adjust();
//接收缓冲区
extern float data_Get;

#endif

