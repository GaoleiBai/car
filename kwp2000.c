/*
 ******************************************************************
 *  公司名称:佳盟科技有限公司
 *  公司地址:广东省东莞市高埗镇草墩工业区
 *      作者:江洪波
 *  文件名称:KWP2000.C
 *  功能描述:基于KWP2000通行协议OBDII软件包
 
*******************************************************************/
#include"KWP2000.h"

#if BAULT_SET_METHOD==0x01
static INT8U  Time0_Value;                    //Time0_Value用于保存计算波特率时,存储定时器0的值,定义为静态全局变量对其他文件屏蔽
static INT8U  Int_Number=0;                   /***Int_Numbe用于保存外部中断的次数,ECU发送给诊断仪的同步字符信号为55H,刚好产生5次中断
                                                当产生5次中断时,定时器0的值即接受55H八位长度的时间****/
#endif

extern INT8U *Net_Data_Point[3];                          //定义一个指向有效数据,可以直接运算的指针数组
extern INT8U Negative_Code;
extern INT8U PID_Support_Byte_Mode1[3][12];
extern INT8U PID_Support_Byte_Mode[3][12]; 
extern const INT8U PID_Byte_Length_Respond_1_2[91];
extern const Request_Byte_Length_KWP2000[];
extern INT8U  P2_Flag;              /*变量k用作发送地址初始化数据时,记算发送的位数.P2变量用
                                作诊断仪记录诊断仪成功发送请求报文,与ECU成功发送应答报文
						          的最大时间间隔,P3为ECU成功发送应答报文到诊断仪发出新请求
						           报文的最小时间间隔***/ 
                             
						   
extern INT8U Rx_Erro;        /**接受错误计数器变量,当错误计数器达到一定值时即RX_ERRO_MAX时,因重新初始化,重新建立握手信号****/
extern volatile INT8U Commu_Method;
extern Time0_Over_Flag;
extern Repeat_Check_Num;
extern INT8U  Rx_Buff[RXC_BUFF_SIZE];               /***接受缓冲器****/
extern INT8U  Rx_Data_Number;                     /***接受当前应答信息的字节数****/
extern INT8U Respond_Message_Num;                  /***接受当前应答信息的帧数目***/
extern INT8U ECU_Addr[3],ECU_Addr_System[3];  /***ECU_Addr[]当前发送应答信息的ECU地址,ECU_Addr_System整个系统的ECU地址***/
extern INT8U ECU_Num;  
extern INT8U Frame_Num; 
extern O2_Num;  /**氧气传感器位置编号量,用于模式五: /* Request oxygen sensor monitoring test results****/
extern Init_Temp_Flag;
/***************************************************************************
函数名称:void Calculate_Set_Baud(void) 
函数功能:根据ECU返回的同步字符55H，即定时器的值,计算并设置通信波特率,该函数时候适合于波特率大于4000bit/s的应用
入口参数:Time0_Value为全局变量,在使用该函数时必须定义该全局变量
返回参数:无
函数说明:该函数仅在BAULT_SET_METHOD==1,即通过接受到的同步字符计算波特率方式时才启用,在直接设置波特率模式下,该函数不参与编译
          在移植该函数时,注意不同的CPU可能波特率设置方式不同,但是Time0_Value为定时器n分频因子下的计数值,分频因子n在外部中断函数设置,
		   定时器的时间即为接受8位数据所用的时间.
          ****************************************************************************/
#if BAULT_SET_METHOD==0x01
void Calculate_Set_Baud(void)
{ 
  INT16U Temp;
  Temp=(Time0_Value<<5)-(1<<7);
  UBRR1H=Temp>>15;
  UBRR1L=Temp>>7;             //计算波特率计数器的低8位值,Time0_Vaue为定时器的初值，
      /*32为定时器的分频因数,8为不包含起始位,校验位,停止位的数据位个数,16波特率计算得分频因子***/
//  Temp _L=(32*Time0_Value-8*16)/(8*16);             //计算波特率计数器的低8位值
//  Temp_H=(32*Time0_Value-8*16)/(8*16*256);
}
#endif
/***************************************************************************
函数名称:void Delay_Us(int us) 
函数功能:微妙级别的延时，延时时间为2*us
入口参数:int ms 延时时间参数
返回参数:无
函数说明:在应用到不同的目标板对象,或者其他的CPU当中,调试函数必须调试到2*us的时间
****************************************************************************/
void Delay_Us(INT16U us)
{
  while(us--)
  {
   NOP();
  }
}

/***************************************************************************
函数名称:void Delay_Ms(INT16U Ms)
函数功能:毫秒级别的延时，延时时间为Ms毫秒
入口参数:int ms 延时时间参数
返回参数:无
函数说明:在应用到不同的目标板对象,或者其他的CPU当中,调试函数必须调试到Ms的时间
****************************************************************************/

void Delay_Ms(INT16U Ms)
{
  INT16U i;
  for(;Ms>0;Ms--)
    for(i=677;i>0;i--);
}

/***************************************************************************
函数名称:void USART_Init(void)
函数功能:基于ISO9141-2正常通行,USART1的初始化
入口参数:无
返回参数:无
函数说明:不同的CPU与OBD2,K,L线连接的串口必须初始化为1起始位+8个数据位+无校验位+1停止位,
            正常通信模式,波特率设置成10.4k,或者使用定时器,捕获同步字符的时间,计算出波特率,
			 在针对特定CPU设置。
****************************************************************************/
void USART_Init(void)
{ 
 UCSR1B = 0x00;          /*初始化设置时，关闭USART1.保证寄存器数值的确定性***/
  
 UCSR1A = 0x00;                      /***设置USART1为双机通行,正常模式***/

 UCSR1C =(1<<UCSZ11)|(1<<UCSZ10);   /*USART1模式设置为1起始位+8个数据位+无校验位+1停止位*/
 /*#if BAULT_SET_METHOD==0x00 
  UBRR1H=(FOSC/(16*BAUD1)-1)/256;    /***基于ISO9141-2正常通信时,直接将波特率设置为10.4k/s****************/
  /*UBRR1L=(FOSC/(16*BAUD1)-1)%256;
 #endif*/
 #if BAULT_SET_METHOD==0x00 
  UBRR1H=((FOSC/(BAUD1<<4)-1)>>8);                 /***基于ISO9141-2正常通信时,直接将
                                            波特率设置为10.4k/s****************/
  UBRR1L=(FOSC/(BAUD1<<4)-1);					
 #else 
  Calculate_Set_Baud();            //根据接受到的同步字符,所用的时间计算,并设置波特率寄存器
 #endif
 }

/***************************************************************************
函数名称:void USART_Init(void)
函数功能:基于ISO9141-2正常通行,USART1的初始化
入口参数:无
返回参数:无
函数说明:不同的CPU与OBD2,K,L线连接的串口必须初始化为1起始位+8个数据位+无校验位+1停止位,
            正常通信模式,波特率设置成10.4k,或者使用定时器,捕获同步字符的时间,计算出波特率,
			 在针对特定CPU设置。
****************************************************************************/
void USART_Disabe(void)
{
  /***关闭串口功能**/
   UCSR1B = 0x00;     //禁止接受和发送,关闭中断使能                         
   UCSR1C =0x00;   
}

/***************************************************************************
函数名称:void INT2_Time0_Init(void)
函数功能:外部中断（串行的所属接口)初始化,用于接受同步字符,定时器1初始化,用于计录接受同步字符的时间
入口参数:无
返回参数:无
****************************************************************************/
void INT2_Time0_Init(void)
{
 TCCR0 = 0x00;                 												 //停止定时器计数
 TCNT0 = 0x00;                 												 //定时计数器清零
 PORTD|=(1<<PD2);
 DDRD&=~(1<<PD2);             												 //设置PD2即INT2口设置为带上拉电阻输入
 CLEAR_I;
 EICRA = 0x30;                												 //INT2口设置为上升沿触发
 EIFR |=(1<<INTF2);
 EIMSK |=(1<<INT2);                												 //打开外部中断
 SET_I;                                                                 //re-enable interrupts
}

/***************************************************************************
函数名称:void INT2_Time0_Disable(void)
函数功能:外部中断和定时器禁止,在接受完同步字符,或者接受同步字符错误情况下调用
入口参数:无
返回参数:无
****************************************************************************/
void INT2_Time0_Disable(void)
{
  TCCR0 = 0x00;
  EIFR |=(1<<INTF2);         												  //清除外部中断标志位
  EIMSK &=~(1<<INT2);                										 //关闭外部中断
                                                                            //re-enable interrupts
}

/*************************************************************************
函数名称:P2_Time_Start(void)
函数功能:启动P2计时,使能溢出中断
入口参数:无
返回参数:无
函数说明:P2时间的最大时间,设置为50ms根据ECU返回不同的关键字决定,同时在移植到不同CPU时,定时器的初值,分频因子由P2时间决定，
           定时器必须初始为溢出中断,普通工作模式。
****************************************************************************/

void P2_Time_Start(void)
{
  TCCR0 = 0x00; 							//stop
  TCNT0 = 0x39; 							//set count
  CLEAR_I;                                    //禁止全局中断
  TIMSK |= (1<<TOIE0); 							//timer interrupt sources
  TIFR|=(1<<TOV0);                  //清除T0溢出标志位
  TCCR0 = 0x07; 							//1024预分频启动计时器
  SET_I; 
}

/***************************************************************************
函数名称:void P2_Time_Stop(void) 
函数功能:停止P2计时,清除溢出标志位
入口参数:无
返回参数:无
****************************************************************************/

void P2_Time_Stop(void)
{
  TCCR0 = 0x00;                     //停止T0定时器
 // TIFR|=0x01;//(1<<TOV0);                  //清除T0溢出标志位
} 
/***************************************************************************
函数名称:void P3_Time_Start(void)
函数功能:启动P3计时,使能溢出中断 4.5s
入口参数:无
返回参数:无
****************************************************************************/

void P3_Time_Start(void)
{
 CLEAR_I;
 TCCR3B = 0x00; 					 //stop
// TCNT3H = 0xE1;                    //setup
// TCNT3L = 0x7C;
 //TCNT3H = 0xC2;                    //setup
 //TCNT3L = 0xF7;
 TCNT3H = 0xBB;                    //setup
 TCNT3L = 0x56; 
 TCCR3A = 0x00;
 ETIMSK|=(1<<TOIE3);        		 //使能T3溢出中断
 TCCR3B = 0x05;          			 //以1024预分频启动定时器3
 SET_I; 
} 

/***************************************************************************
函数名称:void P3_Time_Stop(void) 
函数功能:停止P3计时,清除溢出标志位
入口参数:无
返回参数:无
****************************************************************************/

void P3_Time_Stop(void)
{
  TCCR3B = 0x00; //stop
  ETIFR|=(1<<TOV3); // 清除溢出标志位
}      
/***************************************************************************
函数名称:void Timer_Start_Ms(INT16U Ms) 
函数功能:启动Ms秒的记时，采用定时器1，以clk/256分频启动定时器1
入口参数:无
返回参数:无
****************************************************************************/
void Timer_Start_Ms(INT16U Ms)
{
   INT16U Temp;
   TCCR1B = 0x00;                                 //stop
   Temp=(((FOSC_K>>5)*Ms)>>3);                              
   TCNT1=65535-Temp+1; 
                              
   TCCR1A=0X00;                                 /****选择T1工作的普通模式**/
   TIFR|=(1<<TOV1);                             //写入1清楚溢出标志位
   TCCR1B=0X04;                                 /*****以clk/256分频启动定时器1*/ 
}
/***************************************************************************
函数名称:void Timer_Start_S(INT8U S) 
函数功能:启动S秒的记时，采用定时器1，以clk/1024分频启动定时器1,最小记时时间为4S,最大记时时间为16S，且S应该为4的倍数，否则会产生误差
入口参数:无
返回参数:无
****************************************************************************/
void Timer_Start_S(INT8U S)
{
   TCCR1B = 0x00;                                 //stop
   S=S>>2;                       
   TCNT1=65535-FOSC_B*S+1;                              
   TCCR1A=0X00;                                 /****选择T1工作的普通模式**/
   TIFR|=(1<<TOV1);                             //写入1清楚溢出标志位
   TCCR1B = 0x05;                              /*****以clk/1024分频启动定时器1*/ 
}

void Timer_Stop(void)
{
     TCCR1B = 0x00; 
     TIFR|=(1<<TOV1);                             //写入1清除溢出标志位
}


/***************************************************************************  
函数名称:INT8U Read_ECU_Init_Data(void)
函数功能:读取诊断仪与ECU握手信号过程中,ECU发送给诊断仪的数据,或者发送数据失败
入口参数:无
返回参数:ECU发送给诊断仪的握手信号数据,或者握手失败信号
****************************************************************************/
INT8U Read_ECU_Init_Data(void)
{
 INT8U Data;
 while((UCSR1A&(1<<RXC1))==0)              /***判断是否接收到数据,如没有接收,继续查询
	                                                直到接收到数据,或者定时器溢出****/
   {
	 if(TIFR&(1<<TOV1))
	  {
	   Timer_Stop(); 
	   USART_Disabe();           /***关闭USART1口*********/
	   return ERROR;                             /****返回通信错误信号*****/
	  }
	}
	Data=UDR1;
	return Data;
} 

/***************************************************************************
函数名称:BOOLEAN KWP2000_5_Bauld_Init(void) 
函数功能:基于ISO9142-2通信协议时,必须有一个波特率为5Bit/s,地址33H的初始化信号,
         Atmega64串口的波特率无法设置那么低，该函数采用IO口模拟与定时器结合的方式。
入口参数:无
返回参数:BOOLEAN,如果初始化成功返回TRUE,初始化失败返回FALSE
****************************************************************************/
BOOLEAN KWP2000_5_Baud_Init(INT8U *Key_Word)
{
 INT16U Init_Word=INIT_ADDR_WORD;            /*将要通过I/O模拟发送的10位数据(1起始位+8数据位+1停止位)赋给变量Init_Word*/
 INT8U k,Data0,i; 
 USART_Disabe();
 K_CONFIGE_OUT_H;
 Delay_Ms(300);                  /*???????????????????*/
 for(k=10;k>0;k--)
 {
    Timer_Start_Ms(200);                   //启动200ms定时器
    WAITE_TIMER_OVER;                     //等待定时器溢出
	/****发送初始化地址变量Init_Word的最低位*************/
	if(Init_Word&0x01)
	   {
	    K_OUT_H;
	    LED_OK_ON;
	   }
	else
	  {
	   K_OUT_L;
	   LED_OK_OFF;
	   }
	 Init_Word= Init_Word>>1;
  }
#if BAULT_SET_METHOD==0x01
  INT2_Time0_Init();                   //外部中断与定时器0初始化
  Timer_Start_Ms(300);                   //启动300ms定时器,W1时间的最大值,如果超过该时间,ECU还未反应,表示该种方式通信失败
  while(Int_Number!=5)            //等待同步字符接受完成,并判断设计的300ms定时是否溢出
  {
    if(TIFR&(1<<TOV1))                 //如果定时器溢出
	   {
	     Timer_Stop();
		 Int_Number=0;
	     INT2_Time0_Disable();          
	     return FALSE;                             /****返回通信错误信号*****/
	   }
  }
  INT2_Time0_Disable();     
  Int_Number=0;
#endif
  USART_Init();                        /**KWP2000通信接口即USART1初始化*/ 
  ON_RX1;  
  for(i=0;i<2;i++)
	{
	  Timer_Start_Ms(20); 
	  *Key_Word++=Read_ECU_Init_Data();
    }
 //  Timer_Stop();                         /****定时器1停止工作*****/
   Timer_Start_Ms(35);                   //启动35ms定时器
   WAITE_TIMER_OVER;                     //等待定时器溢出
   OFF_RX1;                       /*因为接受与发送共用一个K线,避免在发送时,产生误接受,串口在发送数据时必须关闭接受器***/
   ON_TX1;  
   Key_Word--;                      //使能发送器
   UDR1=(*Key_Word)^0xff;           //异或取反                   //启动35ms定时器
   WAITE_TX_OVER;
   Timer_Start_Ms(50);
   Delay_Ms(1);                    //?????? 
   ON_RX1;
   Data0=Read_ECU_Init_Data();
   Timer_Stop(); 
   if(Data0!=0xcc)
     return ERROR;
   P3_Time_Start();
   return TRUE;
}  

/***************************************************************************
函数名称:BOOLEAN KWP2000_Fast_Init(void)
函数功能:诊断仪与ECU快速初始化函数。
入口参数:无
返回参数:INT8U类型,当诊断仪与ECU初始化成功,返回TRUE,否则返回FALSE
****************************************************************************/
BOOLEAN KWP2000_Fast_Init(INT8U *Key_Word)
{ 
 struct MESSAGE Message1={
	                   START_COMMUNICATION,
					   PID00,
					   START_REQUEST_BYTE_LENGTH_KWP2000
	                   };
	 INT8U Recive_Staue,i;
	 USART_Disabe();
     K_CONFIGE_OUT_H;
     Delay_Ms(400);                  /*???????????????????*/
	 K_OUT_L;
	 Timer_Start_Ms(25);                   //启动25ms定时器
	 WAITE_TIMER_OVER;                     //等待定时器溢出
	 K_OUT_H;
	 Timer_Start_Ms(25);                   //启动200ms定时器  
	 USART_Init();                   //等待定时器溢出
	 WAITE_TIMER_OVER;  
	 Timer_Stop();
	 Applicaiton_Send_Request_Message(&Message1);  
	 Recive_Staue=Net_Receive_Respond_Message(&Message1,00,Rx_Buff);
	  P3_Time_Start();
	 if(Recive_Staue==MESSAGE_END)
	  {
	     for(i=0;i<2;i++)
	       {
		   *Key_Word=*Net_Data_Point[0]++;
		    Key_Word++;
		   }
		 return TRUE;
	  }
	 else
	    return FALSE;
}


BOOLEAN KWP2000_Init(INT8U *Key_Word)
{
   INT8U KWP_Init_Statu;
   P3_Time_Stop();
   P2_Time_Stop();
   if(Commu_Method!=KWP2000_FAST_INIT)
     {
	  KWP_Init_Statu=KWP2000_5_Baud_Init(Key_Word);
      if(KWP_Init_Statu==TRUE)
	    {
		Commu_Method=KWP2000_5_BAUD_INIT;
	    return TRUE;
		}
	 }
    if(Commu_Method!=KWP2000_5_BAUD_INIT)
	 {
	  KWP_Init_Statu=KWP2000_Fast_Init(Key_Word);
	  if(KWP_Init_Statu==TRUE)
	   {
	     Commu_Method=KWP2000_FAST_INIT;  
		 return TRUE;
	   }
     }
	 return FALSE;
}
/***************************************************************************
函数名称:void Physical_Send_Message(INT8U *P,INT8U Length)
函数功能:通过串口发送网络层传送给物理层的数据
入口参数:INT8U *P 指向发送数据组的的指针，INT8U Length1 发送数据长度
返回参数:无
****************************************************************************/
void Physical_Send_Message(INT8U *P_Tx,INT8U Length_Sum)
{
   INT8U i;
   Timer_Stop();				 		  		   
   /***发送网络层传递过来的数据**********/
   for(i=0;i<Length_Sum;i++)
   {
       /****设置定时器大约为10ms,确保请求信号内部字节之间最小有5ms****/
	   while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
           UDR0=*P_Tx; 
       while(!(UCSR1A&(1<<UDRE1)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
           UDR1=*P_Tx++; 
	  Timer_Start_Ms(8);                   //启动10ms定时器
	   WAITE_TIMER_OVER;                     //等待定时器溢出
   }
   Timer_Stop();
   WAITE_TX_OVER;              //等待发送结束
   Delay_Ms(1);                             //?????为了确接受器打开时,确保数据从I/O口已完全发出,需要延时,具体看调试
   CLEAR_I;           
   RX_INT_ENABLE;
   ON_RX1; 
   SET_I;
   P2_Time_Start();                             //启动P2计时
}

/***************************************************************************
函数名称:void NetWork_Send_Message(INT8U Mode0,INT8U Identify0,INT8U Length0)
函数功能:根据应用层传递过来的数据类型发送命令,根据ISO9141-2通信协议,组合数据,计算校验字节,在传递给物理层
入口参数:INT8U Mode0 应用层传递给网络层发送的请求信息诊断模式,INT8U Identify0 诊断模式的编号参数:PID值,
         TID值,InfoType值,INT8U Length0 发送数据长度
返回参数:无
****************************************************************************/
void NetWork_Send_Message(struct MESSAGE *P1)
{
  INT8U i;                        
  INT8U Txc_Buff[TXC_BUFF_SIZE];         //定义一个发送缓冲区
  INT8U *P_Txc_Buff=Txc_Buff;
  P3_Time_Stop();                             //P3定时器停止计时
  *P_Txc_Buff++=(0xc0|P1->Length);                 //功能地址与长度字节为头字节的第一字节
  *P_Txc_Buff++=FUNCTIONAL_ADDR;                     //第二字节为功能目的地址0x33               
  *P_Txc_Buff++=OBD_ADDR;                     //第三字节为OBD的源地址0x0xf1  
  *(P_Txc_Buff+P1->Length)=0;                    //校验字节先确定为0
  *P_Txc_Buff++=P1->Mode;
  /*****各种模式的格式不同,如模式2额外需要一个冻结帧序号,模式3,4,7不需要PID值
     模式5,额外加了一个测试氧气传感器的编号,模式8数据长度固定不变,无效字节
	 用0X00填充***/
  switch(P1->Mode)
  {
     case MODE1:
	 case MODE6:*P_Txc_Buff++=P1->Identify;break;
     case MODE2:
     {
      *P_Txc_Buff++=P1->Identify;
   	  *P_Txc_Buff++=Frame_Num;        //冻结帧编号0,可能有多个冻结编好,是故障代码有几个，相应的冻结帧就有几个吗？？？发送不同的冻结帧编号,都能收到应答信息
     };break;
     case MODE5:
     {
     *P_Txc_Buff++=P1->Identify;
	 *P_Txc_Buff++=O2_Num;    //测试氧气传感器的编号,即所在的地址
     }break;      
     case MODE8:
   		{
		*P_Txc_Buff++=P1->Identify;
		/****其他字节用0x00填充*****/
		for(i=0;i<(P1->Length-1);i++)
		  {
		   *P_Txc_Buff++=0x00;
		  } 
	    }break;
     case MODE9:*P_Txc_Buff++=P1->Identify;break; 
	 case KEEP_COMMUNICATION:*P_Txc_Buff++=RSPREQ;break;              //ECU需回复信息,该模式用于保持双方的通信
    default:break;
   }
   P_Txc_Buff=Txc_Buff;
  /*****计算最后的校验字节*************/
   for(i=0;i<(P1->Length+3);i++)
    Txc_Buff[P1->Length+3]+=*P_Txc_Buff++;
   /***将网络层组织好的数据,及要发送的数量传送给物理层发送*****/
   Physical_Send_Message(Txc_Buff,P1->Length+4);
}


BOOLEAN Applicaiton_Send_Request_Message(struct MESSAGE *P)  
{
   INT8U Temp,i;
   OFF_RX1;     			
   ON_TX1;
   Respond_Message_Num=0;
   Rx_Data_Number=0;
   Negative_Code=0;
/**判断发送请求消息的模式,并判断是否支持该模式下的PID******/
      if((P->Mode==MODE3)||(P->Mode==KEEP_COMMUNICATION)||(P->Mode==MODE7)||(P->Mode==START_COMMUNICATION)||(P->Mode==STOP_COMMUNICATION))
	  { 
	    NetWork_Send_Message(P);
	    return TRUE;
	  }
	 else
	 { 
        if((P->Identify==0)||(P->Identify==0x20)||(P->Identify==0x40))
	                                          // 下是否支持的PID号,注意一字节的高位开始||((1<<(8-Identify%8)&PID_Support_Byte_Mode1[0][(Identify-1)/8])||((1<<(8-Identify%8))&PID_Support_Byte_Mode1[1][(Identify-1)/8])||((1<<(8-Identify%8))&PID_Support_Byte_Mode1[2][(Identify-1)/8]))*/
	    {
	      NetWork_Send_Message(P);
	      return TRUE;
	    }
	    else
	     {
	         if(P->Identify%8)
		        Temp=(1<<(8-P->Identify%8));
		     else
		       Temp=0x01;
	        if(P->Mode==MODE1)
           {
            /**模式一获取与排放系统相关的动态诊断数据,因为模式一系统使用频率高为了避免
                 频繁读取模式ECU支持的PID值,故采取全局变量,将首次读取支持的PID值存储到全局变量中
	              以后就不需要再读了**Mode_Pid_Support[Mode-1][Pid/8]保存Mode模式*****/
	         if((Temp&PID_Support_Byte_Mode1[0][(P->Identify-1)/8])||(Temp&PID_Support_Byte_Mode1[1][(P->Identify-1)/8])||(Temp&PID_Support_Byte_Mode1[2][(P->Identify-1)/8]))
		       {
			     NetWork_Send_Message(P);
			     return TRUE;
			   }
		       else
		          return NO_SUPPORT;
            }
		   else
		   {
			    if((Temp&PID_Support_Byte_Mode[0][(P->Identify-1)/8])||(Temp&PID_Support_Byte_Mode[1][(P->Identify-1)/8])||(Temp&PID_Support_Byte_Mode[2][(P->Identify-1)/8]))
		       {
			     NetWork_Send_Message(P);
			     return TRUE;
			   }
		       else
		         return NO_SUPPORT;
		    }
        }
	 } 
  
}

/***************************************************************************
函数名称:INT8U P1_Time_Hand(INT8U temp)
函数功能:启动P1计时,检测P1是否超时,P1字节为应答信息字节内部的最大时间间隔20ms
入口参数:接受的数据字节个数
返回参数:TRUE 表示P1时间为超时， P1_TIME_ERRO 表示P1时间超时
****************************************************************************/

INT8U P1_Time_Hand(INT8U temp)
{
   Timer_Start_Ms(20);
   while(temp==Rx_Data_Number)
   {
     if(TIFR&(1<<TOV1))
	 {
	   Timer_Stop();
	   return P1_TIME_ERRO;
	 }
   }
   Timer_Stop();
   return TRUE;
}

/***************************************************************************
函数名称:INT8U Net_Receive_Respond_Message(INT8U Mode,INT8U Idenify,INT8U i,INT8U *P)
函数功能:网络层接收来自物理层的数据,并判断读取数据是否正确，及是否出现差错。同时校验接收到得数据
           物理层接收数据流采用中断方式，网络层接收数据才有查询的方式，查询物理层是否接收了新的字节数。
入口参数:MESSAGE *Message2 请求结构数据，i 网络层接收来自物理层的字节数，*P当前网络指向接收来自物理层得数据缓冲地址
全局变量:ECU_Addr[]保存当前发送应答信息的ECU地址,Negative_Code：如果没有ECU消极响应应答信息，
         Net_Data_Point[Respond_Message_Num]:表式第Respond_Message_Num有效数据地址;
返回参数:TRUE 该帧数据接收成功，FALSE表示帧数据接收失败，Negative表示ECU返回消极应答信号。Negative_Code保存消极响应代码
****************************************************************************/
INT8U Net_Receive_Respond_Message(struct MESSAGE *Message2,INT8U i,INT8U *P)
{
   INT8U j,Check_Test=0,Data_Num; 
   INT8U Respond_Staue,P1_Time_Staue;
   while(i==Rx_Data_Number)
   {
   	 if(P2_Flag==P2_TIME_OVER)
	  {
	    P2_Flag=0;
	    P2_Time_Stop();
	    if(Respond_Message_Num==0)
		  {
		   if(Negative_Code==0)
			  return NO_RESPPOND;
		   else
		     return Negative_Code;
		  }
		else
		 {
		   P2_Time_Stop();
		   Rx_Data_Number=0;                      //接受缓冲器字节数清零,等待接受下一个应答信息
		   if(Negative_Code==0)
		     return MESSAGE_END;
		   else 
		     return Negative_Code;
		 }
	  }
   }
   P2_Time_Stop();
   while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
     UDR0=*P;
	Data_Num=0x3f&(*P);
	Check_Test+=*P++;
	i++;
    P1_Time_Staue=P1_Time_Hand(i);
    if(P1_Time_Staue!=TRUE)
	  return P1_TIME_ERRO;
    while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
      UDR0=*P;
    if(*P!=OBD_ADDR)
      return STRUCTURE_ERRO; 
    Check_Test+=*P++;
    i++; 
	P1_Time_Staue=P1_Time_Hand(i);
    if(P1_Time_Staue!=TRUE)
	  return P1_TIME_ERRO;
	while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
      UDR0=*P;
    ECU_Addr[Respond_Message_Num]=*P;
    Check_Test+=*P++;
    i++;	
    P1_Time_Staue=P1_Time_Hand(i);
    if(P1_Time_Staue!=TRUE)
	  return P1_TIME_ERRO;
    while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
      UDR0=*P;
    Data_Num--;
   if(*P==((Message2->Mode)|0X40))
   {
      Check_Test+=*P++;
	  i++;
	if(((Message2->Mode)!=KEEP_COMMUNICATION)&&((Message2->Mode)!=MODE3)&&((Message2->Mode)!=MODE4)&&((Message2->Mode)!=MODE7)&&((Message2->Mode)!=START_COMMUNICATION)&&((Message2->Mode)!=STOP_COMMUNICATION))     //如果为模式4，模式7，模式3则无Idenify，故跳过Idenify的检验
     { 
      P1_Time_Staue=P1_Time_Hand(i);
       if(P1_Time_Staue!=TRUE)
	     return P1_TIME_ERRO;
	   while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
        UDR0=*P;
      if(*P!=(Message2->Identify))
       return STRUCTURE_ERRO;
	  Check_Test+=*P++;
	  i++;
	  Data_Num--;
     }  
	 Net_Data_Point[Respond_Message_Num]=P;
	 for(j=0;j<Data_Num;j++)
	 {
	    P1_Time_Staue=P1_Time_Hand(i);
        if(P1_Time_Staue!=TRUE)
	      return P1_TIME_ERRO;
	    while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
        UDR0=*P;
	   Check_Test+=*P++;
	   i++;
	 }
	Respond_Message_Num++;
   }
   else
     {
	  if(*P==0x7F)
      {
       Check_Test+=*P++;
       i++;	
	   P1_Time_Staue=P1_Time_Hand(i);
       if(P1_Time_Staue!=TRUE)
	    return P1_TIME_ERRO;
	   while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
       UDR0=*P;
	   if(*P!=Message2->Mode)
	     return STRUCTURE_ERRO;  
       Check_Test+=*P++;
       i++;	
	   P1_Time_Staue=P1_Time_Hand(i);
       if(P1_Time_Staue!=TRUE)
	     return P1_TIME_ERRO;
	   while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
       UDR0=*P;
	   Negative_Code=*P;
	   Check_Test+=*P++;
       i++;	
       }
	   else
	     return STRUCTURE_ERRO;
     }
    P1_Time_Staue=P1_Time_Hand(i);
    if(P1_Time_Staue!=TRUE)
	  return P1_TIME_ERRO;
	while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
     UDR0=*P;
    while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
     UDR0=Check_Test;
   if(Check_Test!=*P)
    return CHECKSUM_ERRO;	
   P2_Time_Start();
   P++;
   i++;
  //  Display();//??????????????
   Respond_Staue=Net_Receive_Respond_Message(Message2,i,P);	
   return Respond_Staue;
}

/***************************************************************************
函数名称:void Get_ECU_Addr(void)
函数功能:获取车辆OBD诊断系统中ECU个数,ECU分配的地址
全局变量:ECU_Addr_System[]保存系统的ECU地址,ECU_Num保存系统ECU数量
入口参数:无
返回参数:无
****************************************************************************/
void Get_ECU_Addr(void)
{
   INT8U Respond_Staue,m;
   struct MESSAGE Message={
                    MODE1,
					PID00,
					MODE1_REQUEST_BYTE_LENGTH_KWP2000
                   };
   Applicaiton_Send_Request_Message(&Message);
   Respond_Staue=Net_Receive_Respond_Message(&Message,0X00,Rx_Buff);
   P3_Time_Start();
   if(Respond_Staue==MESSAGE_END)
     {
	  for(m=0;m<Respond_Message_Num;m++)
      {
        ECU_Addr_System[m]=ECU_Addr[m];
      }
      ECU_Num=Respond_Message_Num;
	 }
}

/****下面定义应用系统各功能函数*****/

/***************************************************************************
函数名称:INT8U Read_Support_Mode(INT8U Mode_1)
函数功能:读取该模式下支持的Identify参数。如果为模式1，保存都PID_Support_Byte_Mode1[][]固定全局变量数组中，
         其他模式保存在动态的PID_Support_Byte_Mode[][]全局变量数组中 
入口参数:INT8U Mode_1 读取Identify的模式变量
返回参数：ECU所支持的PID范围,0 表示该模式下不支持任何Identify,1表示支持PID00的范围,2表示PID00-PID20,3表示PID00-PID40
****************************************************************************/
/****读各模式下支持的PID,或TID,或Infytype****/
INT8U Read_Support_Mode(INT8U Mode_1)
{
   struct MESSAGE R_Message;
   INT8U i,j,k,Net_Receive_Staue;
   R_Message.Mode=Mode_1;
   R_Message.Length=Request_Byte_Length_KWP2000[Mode_1-1];
  if(Mode_1!=MODE1)
   {
     for(i=0;i<3;i++)
	   for(j=0;j<12;j++)
	 PID_Support_Byte_Mode[i][j]=0;
   }
     for(i=0;i<3;i++)
	  {
	   R_Message.Identify=0x20*i;
	   Applicaiton_Send_Request_Message(&R_Message);
	   Net_Receive_Staue=Net_Receive_Respond_Message(&R_Message,00,Rx_Buff); 
	    P3_Time_Start();
	   /*****如果正确接收则将数据保存到自定义的数组中****/
	   if(Net_Receive_Staue==MESSAGE_END)
	    {
		  for(j=0;j<Respond_Message_Num;j++)
		   {
			    if(Mode_1==MODE1)
				 { 
				 for(k=0;k<4;k++)
				   PID_Support_Byte_Mode1[j][R_Message.Identify/8+k]=*(Net_Data_Point[j]++);
				 }
				else
				 {
				    switch(Mode_1)
				     {
				       case MODE2:Frame_Num=*(Net_Data_Point[j]++);break;
					   case MODE5:O2_Num=*(Net_Data_Point[j]++);break;
					   case MODE6:
					   case MODE8:
					   case MODE9: Net_Data_Point[j]++;break;
					   default:break;
				     }
				    for(k=0;k<4;k++)
				    {
					  PID_Support_Byte_Mode[j][R_Message.Identify/8+k]=*(Net_Data_Point[j]++);
					  if(R_Message.Mode==MODE9)
				         {
                         while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
		                  UDR0=PID_Support_Byte_Mode[j][R_Message.Identify/8+k];
				        }
                    }
  			     } 
		    }
		   continue;
		}
		else 
		  return  i;
	  }
     return i;
}

void Respond_Statue_Hand(INT8U Respond_Message_Staue,struct MESSAGE *P_Message,INT8U Repeat_Check_Num)
{
  INT8U Tx_Staue;
  switch(Respond_Message_Staue)
  {
    case TRUE:
		 	  {
			    if(Rx_Erro>0)
				  Rx_Erro--;
			  }break;//Display();
	case NO_RESPPOND:  Repeat_Check_Num=1;//如果没有回应应答信息,只让系统重发一次
	case STRUCTURE_ERRO:
	case CHECKSUM_ERRO:
	                  {
					   P3_Time_Stop();
					  /********启动3s的定时时间,即请求信息每3.5S发送一次*注意此时P3定时器是否打开*/
					   Timer_Start_Ms(3000);
		               WAITE_TIMER_OVER;                   //等待定时器溢出
			           Timer_Stop(); //stop
					   Rx_Erro++; 
					   if(Rx_Erro>RX_ERRO_MAX)
					     {
						   INT8U Key_Word[2];
						  Delay_Ms(2000);
	                      KWP2000_Init(Key_Word);
			              P3_Time_Start();
						  Rx_Erro=0;
					     }
					   /****定义一个发送状态变量****/
					  
		               Tx_Staue=Applicaiton_Send_Request_Message(P_Message);
					   if(Tx_Staue!=TRUE)
					     break;
	                   Respond_Message_Staue=Net_Receive_Respond_Message(P_Message,00,Rx_Buff);
					   P3_Time_Start();
					   Repeat_Check_Num--;
					   if(Repeat_Check_Num)
					     Respond_Statue_Hand(Respond_Message_Staue,P_Message,Repeat_Check_Num);
					  }break;
	  case BRR:
	  {
	  INT8U Key_Word[2];
	  Delay_Ms(50000);
	  KWP2000_Init(Key_Word);
	  }break;
	  case RCR_RP:
	  {
	  LED_UP_ON;
      CLEAR_I;           
      RX_INT_ENABLE;
      ON_RX1; 
      SET_I;
	  P3_Time_Stop();
	  while((Respond_Message_Staue==NO_RESPPOND)||(Respond_Message_Staue==RCR_RP))
	  {
	   if(Respond_Message_Staue==RCR_RP)
		{
		  Timer_Start_S(8);
		  Rx_Data_Number=0;
		 }
	   if(TIFR&(1<<TOV1))
	    {
		  INT8U Key_Word[2];
	      Timer_Stop();
	      KWP2000_Init(Key_Word);
		  break;
	    }
	   Respond_Message_Num=0;
       Negative_Code=0; 
	    P2_Time_Start(); 
	   Respond_Message_Staue=Net_Receive_Respond_Message(P_Message,0,Rx_Buff);
	  }
	   P3_Time_Start();
	    Timer_Stop();
	  }break;
	  default:break;
  }
}
#pragma interrupt_handler timer0_ovf_isr:17
void timer0_ovf_isr(void)
{
  P2_Flag=P2_TIME_OVER;
  Init_Temp_Flag=TIFR;
}
/*利用定时最小间隔为4s产生保持总线激活的信号:模式1,PID00,的功能请求信号*/
#pragma interrupt_handler timer3_ovf_isr:30
void timer3_ovf_isr(void)
{
   struct MESSAGE Keep_Message={
                          KEEP_COMMUNICATION,
						  PID00,
						  KEEP_REQUEST_BYTE_LENGTH_KWP2000
                        };
   Applicaiton_Send_Request_Message(&Keep_Message);  //发送MODE1,PID00的保持通信信号
   Net_Receive_Respond_Message(&Keep_Message,00,Rx_Buff);
   P3_Time_Start();
   LED_CANCEL_ON;
}
#if BAULT_SET_METHOD==0x01
#pragma interrupt_handler int2_isr:4
void int2_isr(void)
{
  unsigned char sreg;
  if(Int_Number==0)      //如果第一次进入外部中断,即刚开始接受到同步字节的第一位
  {
    TCCR0 = 0x03;             //启动定时器计时
	Timer_Stop();            //W1记时关闭
  }
  Int_Number++;
  if(Int_Number==5)
  {
    LED_OK_ON;
    TCCR0 = 0x00;
	sreg=SREG;
	CLI();
    Time0_Value=TCNT0;
    while(!(UCSR0A&(1<<UDRE0)));        /**检查发送数据寄存器是否为空,并等待发送数据缓冲器为空**/
       UDR0=Time0_Value;
    EICRA = 0x00;                //清除触发方式
    EIMSK = 0x00;                //关闭外部中断
	sreg=SREG;
	SEI();
  }
}
#endif
#pragma interrupt_handler uart1_rx_isr:31
void uart1_rx_isr(void)
{
   Rx_Buff[Rx_Data_Number]=UDR1;
   Rx_Data_Number++;
}
