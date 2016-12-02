#ifndef _KWP2000_h
#define _KWP2000_h
#include<macros.h>
#include<iom64v.h>
#include<ASSERT.H>
typedef unsigned char BOOLEAN;
typedef unsigned char INT8U;
typedef signed char INT8S;
typedef unsigned int INT16U;
typedef int INT16S;
typedef unsigned long INT32U;
typedef long INT32S;
struct MESSAGE
{
   INT8U Mode;
   INT8U Identify;
   INT8U Length;
};
      

/***typedef struct
       {
	     INT8U Header[3];
		 INT8U Data_By;te;
		 INT8U Length;
	   }Respond_Message;	   
***/
#define SET_I  		            SEI()
#define CLEAR_I     			CLI()
#define ON_RX1					UCSR1B |=(1<<RXEN1)
#define OFF_RX1     			UCSR1B &=~(1<<RXEN1)   
#define ON_TX1					UCSR1B |=(1<<TXEN1)
#define OFF_TX1					UCSR1B &=~(1<<TXEN1) 
#define RX_INT_ENABLE           UCSR1B |=(1<<RXCIE1)                 
#define INIT_ADDR_BYTE       	0x33
#define INIT_ADDR_WORD       	0x266

/****为了统一函数中语句格式,宏定义中最后一条语句分号特意省略在函数中写,而且不能用括号()把所用语句括起***/
#define K_CONFIGE_OUT_H         PORTD|=(1<<PD3);DDRD|=(1<<PD3)   
#define K_OUT_H              	PORTD|=(1<<PD3)
#define K_OUT_L              	PORTD&=~(1<<PD3)


#define WAITE_TIMER_OVER     while(!(TIFR&(1<<TOV1)))
#define WAITE_TX_OVER        while(!(UCSR1A&(1<<TXC1)))


/****系统晶振4MHZ*****/
#define FOSC 4000000     

#define FOSC_K 4000 

/****如果为晶振为4000000的整数倍,则只需把一下宏定义15625乘以倍数即口，如果非4000000的倍数，则需重新设计Time_Start_S函数***/
#define FOSC_B 15625
/***地址初始化波特率为5Bit/s****/           
#define BAUD0 5   
/***正常通信波特率为10.4K/s*****/                   
#define BAUD1 10400  
/***接受缓冲区大小为  **********/               
#define RXC_BUFF_SIZE  55
/***发送缓冲区大小为  **********/                
#define TXC_BUFF_SIZE  11
/***地址初始化信号为0X33*******/               
#define INIT_ADDR    0x33   
/***ECU发送的关键字值可能为0x08或者0x94****/                
#define KEY_WORD_0   0x08
#define KEY_WORD_1   0x94
/*P2K_MIN_08,P2K_MAX_08为关键字为0x08时诊断仪成功发送请求报文,
        与ECU成功发送应答报文的最小,最大时间间隔,单位ms**/
#define P2K_MIN_08   20
#define P2K_MAX_08   50
/*P2K_MIN_94,P2K_MAX_94为关键字为0x94时诊断仪成功发送请求报文,
        与ECU成功发送应答报文的最小,最大时间间隔,单位ms**/
#define P2K_MIN_94   0
#define P2K_MAX_94   50

/*P3K_MIN,P3K_MAX为ECU成功发送应答报文到诊断仪发出新请求报文的最小,最大时间间隔,单位ms**/
#define P3K_MIN   55
#define P3K_MAX   5000

#define RX_ERRO_MAX  10

/*HEADER_F,HEADER_S,HEADER_T,是头三字节的值宏定义**/
#define HEADER_F  0x68
#define HEADER_S  0x6A
#define HEADER_T  0xF1
#define HEADER_CHECK (HEADER_F+HEADER_S+HEADER_T)

#define TRUE      0x00
#define FALSE     0X01
#define ERROR     0X02
#define NO_SUPPORT  0X03
#define NO_RESPPOND 0x04
#define MESSAGE_END 0x00
#define STRUCTURE_ERRO 0X10
#define CHECKSUM_ERRO  0x20
#define P1_TIME_ERRO   0x40
#define P2_TIME_OVER   0x80
/**当BAULT_SET_METHOD==0x00,ISO9141-2正常通信的波特率,直接设置为10.4k/s,
   当BAULT_SET_METHOD==0x01,ISO9141-2正常通信的波特率,根据ECU发送给诊断仪
   55H所用的时间计算得出****************************************/ 
#define BAULT_SET_METHOD 0X00
/****应答信息头字节宏定义******/
#define HEADER_BYTE1  0X48
#define HEADER_BYTE2  0X6B
/******各模式下的请求信息的字节长度宏定义***********/    
#define MODE1_REQUEST_BYTE_LENGTH_KWP2000 		2
#define MODE2_REQUEST_BYTE_LENGTH_KWP2000       3 
#define MODE3_REQUEST_BYTE_LENGTH_KWP2000 		1
#define MODE4_REQUEST_BYTE_LENGTH_KWP2000 		1
#define MODE5_REQUEST_BYTE_LENGTH_KWP2000 		3
#define MODE6_REQUEST_BYTE_LENGTH_KWP2000 		2 
#define MODE7_REQUEST_BYTE_LENGTH_KWP2000 		1
#define MODE8_REQUEST_BYTE_LENGTH_KWP2000 		7 
#define MODE9_REQUEST_BYTE_LENGTH_KWP2000 		2 
#define START_REQUEST_BYTE_LENGTH_KWP2000       1
#define STOP_REQUEST_BYTE_LENGTH_KWP2000        1  
#define KEEP_REQUEST_BYTE_LENGTH_KWP2000        2
#define LED_DOWN_ON 		  					PORTE&=~(1<<PE4)
#define LED_DOWN_OFF 		  					PORTE|=(1<<PE4)
#define LED_UP_ON 	  						    PORTE&=~(1<<PE5)
#define LED_UP_OFF 	  						    PORTE|=(1<<PE5)
#define LED_CANCEL_ON  	     	  				PORTE&=~(1<<PE6)
#define LED_CANCEL_OFF 		  					PORTE|=(1<<PE6)
#define LED_OK_ON   	  						PORTE&=~(1<<PE7)
#define LED_OK_OFF  	  						PORTE|=(1<<PE7)

  /******定义应答信息字节长度固定模式的应答信息字节长度不包括PID，MODE字节 ***/
#define MODE3_RESPOND_BYTE_LENGTH 			    6
#define MODE4_RESPOND_BYTE_LENGTH 				0
#define MODE5_RESPOND_BYTE_LENGTH_1 			2
#define MODE5_RESPOND_BYTE_LENGTH_2 			4
#define MODE6_RESPOND_BYTE_LENGTH 				5
#define MODE7_RESPOND_BYTE_LENGTH 				6
#define MODE8_RESPOND_BYTE_LENGTH 				5  

/************模式9,InfoType为偶数时为固定字节：1字节MessageCount+4字节有效数据*/ 
#define MODE9_RESPOND_BYTE_LENGTH_EVEN          5        
/**定义地址初始化发送的10位数据,1起始位+8数据位(33H)+1停止位 **/
#define INIT_WORD    0x266

#ifndef OBD_IDENTIFY
#define OBD_IDENTIFY
#define PID00 		 	  						0x00
#define PID01 									0x01
#define PID02 									0x02
#define PID03 									0x03
#define PID04 									0x04
#define PID05 									0x05
#define PID06 									0X06
#define PID07 									0x07
#define PID08 									0x08
#define PID09 									0x09
#define PID0A 									0x0a
#define PID0B 									0x0B
#define PID0C 									0x0C
#define PID0D 									0X0D
#define PID0E 									0x0E
#define PID0F 									0x0F
#define PID10 		 	  						0x10
#define PID11 									0x11
#define PID12 									0x12
#define PID13 									0x13
#define PID14 									0x14
#define PID15 									0x15
#define PID16 									0X16
#define PID17 									0x17
#define PID18 									0x18
#define PID19 									0x09
#define PID1A 									0x1A
#define PID1B 									0x1B
#define PID1C 									0x1C
#define PID1D 									0X1D
#define PID1E 									0x1E
#define PID1F 									0x1F
#define PID20 									0x20
#define PID40 									0x40

#define INFOTYPE00  							0x00
#define INFOTYPE01  							0x01
#define INFOTYPE02  							0x02
#define INFOTYPE03  							0x03
#define INFOTYPE04  							0x04
#define INFOTYPE05  							0x05
#define INFOTYPE06  							0x06
#define INFOTYPE07  							0x07
#define INFOTYPE08  							0x08
#define INFOTYPE09  							0x09

#define TID00 									0x00
#define TID01 									0x01
#define TID02 									0x02
#define TID03 									0x03 
#define TID04 									0x04
#define TID05 									0x05
#define TID06 									0x06
#define TID07 									0x07 
#define TID08 									0x08
#define TID09 									0x09
#define TID0A 									0x0A
#define TID0B 									0x0B 
#define TID0C 									0x0C
#define TID0D 									0x0D
#define TID0E 									0x0E
#define TID0F 									0x0F 
#define TID10 	   								0x10
#define TID11 									0x11
#define TID12 									0x12
#define TID13 									0x13 
#define TID14 									0x14
#define TID15 	  								0x15
#define TID16 									0x16
#define TID17 									0x17 
#define TID18 									0x18
#define TID19 									0x19
#define TID1A 									0x1A
#define TID1B 									0x1B 
#define TID1C 									0x1C
#define TID1D 									0x1D
#define TID1E 									0x1E
#define TID1F 									0x1F 
#define TID20 									0x20 

#define MODE1 									0X01
#define MODE2 									0X02
#define MODE3 									0x03
#define MODE4 									0X04
#define MODE5 									0X05
#define MODE6 									0X06
#define MODE7 									0x07
#define MODE8 									0X08
#define MODE9 									0X09

#endif

#define START_COMMUNICATION                     0X81
#define STOP_COMMUNICATION                      0X82
#define KEEP_COMMUNICATION                      0X3E

#define RSPREQ                                  0X01

#define FORMATE_BYTE                            0XC0
#define FUNCTIONAL_ADDR                         0x33
#define OBD_ADDR                                0xf1
#define REPEAT_SEND_NUM                           6

#define KWP2000_5_BAUD_INIT                      0X01
#define KWP2000_FAST_INIT                        0X02

#define RCR_RP                                 0x78
#define BRR                                    0x21

void Calculate_Set_Baud(void);
void Delay_Us(INT16U us);
void Delay_Ms(INT16U Ms);
void USART_Init(void);
void INT2_Time0_Init(void);
void P2_Time_Start(void);
void P2_Time_Stop(void);
void P3_Time_Start(void);
void P3_Time_Stop(void);
void Timer_Start_Ms(INT16U Ms);
void Timer_Start_S(INT8U S);
void Timer_Stop(void);

INT8U Read_ECU_Init_Data(void);
BOOLEAN KWP2000_5_Baud_Init(INT8U *Key_Word);
BOOLEAN KWP2000_Fast_Init(INT8U *Key_Word);
BOOLEAN KWP2000_Init(INT8U *Key_Word);
void Physical_Send_Message(INT8U *P,INT8U Length1);
void NetWork_Send_Message(struct MESSAGE *P1);
BOOLEAN Applicaiton_Send_Request_Message(struct MESSAGE *P);
INT8U Net_Receive_Respond_Message(struct MESSAGE *Message2,INT8U i,INT8U *P);
INT8U P1_Time_Hand(INT8U temp);
INT8U Net_Receive_Respond_Message(struct MESSAGE *Message2,INT8U i,INT8U *P);
void Respond_Statue_Hand(INT8U Respond_Message_Staue,struct MESSAGE *P_Message,INT8U Repeat_Check_Num);
void timer0_ovf_isr(void);
void int2_isr(void);
void uart1_rx_isr(void);
void Get_ECU_Addr(void);
BOOLEAN Read_Support_Mode(INT8U Mode_1);
#endif
