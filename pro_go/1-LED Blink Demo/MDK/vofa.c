#include "headfile.h"

float vofa_send_data[VOFA_MAX] = {0};

// 把浮点数据转为4个八位数据，存到数组中
static void Float_to_Byte(float f, uint8 byte[]) 
{
    FloatLongType1 fl;
    fl.fdata = f;
    byte[0] = (unsigned char)fl.ldata;
    byte[1] = (unsigned char)(fl.ldata >> 8);
    byte[2] = (unsigned char)(fl.ldata >> 16);
    byte[3] = (unsigned char)(fl.ldata >> 24);
}


// vofa+上位机协议-Justfloat
//-------------------------------------------------------------------------------------------------------------------
//  @brief      示波器函数
//  @param      data_str            存储的数据地址
//  @param      num             			发送的数据数量，
//		 vofa_send_data[0] = 1;
//		 vofa_send_data[1] = 2;
//			vofa_send_data[3] = dat;
//		 vofa_send_data[2] = 3;
//		 vofa_send_data[7] = 4;
//		 vodka_JustFloat_send(vofa_send_data, 8);
//  Sample usage:  发送vofa_send_data数组的元素到vofa上  
//-------------------------------------------------------------------------------------------------------------------
void vodka_JustFloat_send(float *data_str,uint16 num) 
{
    uint8 i = 0;
    uint8 byte[4] = {0};
    uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f};	
    // 循环遍历数据并发送
    for (i = 0; i < num; i++) 
    {
        Float_to_Byte(data_str[i], byte);
        UART_PutChar(byte);
    }
    
    UART_PutChar(tail);// 发送帧尾
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      vofa调参 vofa调参 vofa调参 vofa调参 vofa调参
//-------------------------------------------------------------------------------------------------------------------

uint8 RxBuffer[1];//串口接收缓冲
uint16 RxLine = 0;//指令长度
uint8 DataBuff[200];//指令内容
float data_Get=0;//接收缓冲区


///*
// * 解析出DataBuff中的数据
// * 返回解析得到的数据
// */
float Get_Data(void)
{
    uint8 data_Start_Num = 0;  // 记录数据位开始的地方
    uint8 data_End_Num = 0;    // 记录数据位结束的地方
    uint8 data_Num = 0;        // 记录数据位数
		uint8 minus_Flag = 0; 		 // 判断是不是负数
		float data_return = 0; 			 // 解析得到的数据
		uint8 i;
    for( i=0;i<200;i++)   // 查找等号和感叹号的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
    return data_return;
}



/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust()
{
     data_Get = Get_Data(); // 存放接收到的数据
	
//    printf("data=%.2f\r\n",data_Get);
//    if(mode == 1)//循迹环
//    {
	
				#if 0
            if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            Turn_PID.kp = data_Get;
            else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            Turn_PID.ki = data_Get;
            else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            Turn_PID.kd = data_Get;
		    else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
				{
					R_pid.Target = data_Get; 
					L_pid.Target = data_Get; 
				}
                else if((DataBuff[0]=='p' && DataBuff[1]=='t'))
                Turn_PID.kp1 = data_Get;

				#endif
	
				#if 1
        if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            R_pid.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            R_pid.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            R_pid.kd = data_Get;
		    else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
            R_pid.Target = data_Get;
				
				
//				    r_target_speed = data_Get;

				#endif
				
				#if 0
				if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
						ang_pid.kp = data_Get;
				else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
						ang_pid.ki = data_Get;
				else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
						ang_pid.kd = data_Get;
				else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度              
						ang_pid.Target = data_Get;           
				else if((DataBuff[0]=='s' && DataBuff[1]=='g'))
						pwm_state = data_Get;

				#endif

				
					#if 0
        if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
           LEFT_WEIGHT = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
           RIGHT_WEIGHT = data_Get;


				#endif
				
}

