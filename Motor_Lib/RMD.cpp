#include "RMD.h"
/*************************************  RMD  ************************************************/

/*
Order_Id :  0x30, 读取当前电机的PID
            0x33，读取加速度参数
            0x90,读取编码器的当前位置
            0x19，写入当前位置到ROM作为电机零点，多次使用会影响芯片寿命
						0x92,读取多圈角度
						0x94,读取单圈角度
						0x95，清楚电机角度
						0x9A,读取电机状态1和错误标志
            0x9B,清除电机错误标志
						0x9C,读取电机状态2
						0x9D,读取电机状态3
            0x80,电机关闭
						0x81,电机停止
            0x88,电机运行
*/
HAL_StatusTypeDef USER_CAN::RMD_Read_and_Write_Things(uint16_t Id,uint16_t Order_Id)
{
	uint8_t pData[8];	
	pData[0]=Order_Id;
	pData[1] =0x00;
	pData[2] =0x00; 
	pData[3] =0x00;
	pData[4] =0x00;
	pData[5] =0x00; 
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);
}

/*
Order_Id: 0x31,写入PID参数到RAM，断电后参数失效
          0x32,写入PID参数到ROM，断电后仍然有效
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_PID(uint16_t Id,uint16_t Order_Id,uint16_t anglePidKp,uint16_t anglePidKi,uint16_t speedPidKp,uint16_t speedPidKi,uint16_t iqPidKp,uint16_t iqPidKi)
{
	uint8_t pData[8];	
	pData[0] =Order_Id;
  pData[1] =0x00;
	pData[2] =anglePidKp;
	pData[3] =anglePidKi;
	pData[4] =speedPidKp;
	pData[5] =speedPidKi; 
	pData[6] =iqPidKp;
	pData[7] =iqPidKi;
	return this->Send(Id,pData);	
}
/*
0x34,写入加速度到RAM
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_ACCLE_to_RAM(uint16_t Id,int32_t Accel)
{
	uint8_t pData[8];	
	pData[0] = 0x34;
  pData[1] =0x00;
	pData[2] =0x00; 
	pData[3] =0x00;
	pData[4] =Accel;
	pData[5] =(Accel>>8); 
	pData[6] =(Accel>>16);
	pData[7] =(Accel>>24);
	return this->Send(Id,pData);	
}

/*
0x91，写入编码器值到ROM作为电机零点
*/
HAL_StatusTypeDef USER_CAN::RMD_Write_EncoderOffset_to_ROM(uint16_t Id,uint16_t EncoderOffset)
{
	uint8_t pData[8];	
	pData[0] = 0x91;
  pData[1] =0x00;
	pData[2] =0x00; 
	pData[3] =0x00;
	pData[4] =0x00;
	pData[5] =0x00; 
	pData[6] =EncoderOffset;
	pData[7] =EncoderOffset>>8;
	return this->Send(Id,pData);	
}

/*
0xA1,转矩闭环控制
*/
HAL_StatusTypeDef USER_CAN::RMD_Iqcontrol_Motor(uint16_t Id,int16_t iqControl)
{
	uint8_t pData[8];	
	pData[0] =0XA1;
  pData[1] =0x00;
	pData[2] =0x00; 
	pData[3] =0x00;
	pData[4] =iqControl;
	pData[5] =iqControl>>8; 
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);	
}

/*
Order_Id： 0xA2,速度闭环控制
					  0xA3,位置闭环控制命令1
						0xA4,位置闭环控制命令2
						0xA7,位置闭环控制命令5
						0xA8,位置闭环控制命令6
*/
HAL_StatusTypeDef USER_CAN::RMD_Speedcontrol_Motor(uint16_t Id,uint16_t Order_Id,uint16_t maxSpeed,int32_t angleControl)
{
	uint8_t pData[8];	
	pData[0] =Order_Id;
  pData[1] =0x00;
	pData[2] =maxSpeed; 
	pData[3] =maxSpeed>>8;
	pData[4] =angleControl;
	pData[5] =angleControl>>8; 
	pData[6] =angleControl>>16;
	pData[7] =angleControl>>24;
	return this->Send(Id,pData);	
}

/*
Order_Id：0xA5,位置闭环控制命令3，spinDirection 0x00代表顺时针，0x01代表逆时针
					0xA6,位置闭环控制命令4
*/
HAL_StatusTypeDef USER_CAN::RMD_Anglecontrol_Motor(uint16_t Id,uint16_t Order_Id,uint8_t spinDirection,uint16_t maxSpeed,uint16_t angleControl)
{
	uint8_t pData[8];	
	pData[0] =Order_Id;
  pData[1] =spinDirection;
	pData[2] =maxSpeed; 
	pData[3] =maxSpeed>>8;
	pData[4] =angleControl;
	pData[5] =angleControl>>8; 
	pData[6] =0x00;
	pData[7] =0x00;
	return this->Send(Id,pData);	
}

HAL_StatusTypeDef USER_CAN::RMD_Receive(CAN_HandleTypeDef *hcan)
{
		if(this->FIFO == 0) return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO0,&this->RxHeader,this->rx_buf);
		else return HAL_CAN_GetRxMessage(this->hcan,CAN_RX_FIFO1,&this->RxHeader,this->rx_buf);
}

HAL_StatusTypeDef MOTOR_RMD::RMD_update(void)
{
	if(this->can_rev->RxHeader.StdId == this->ID)
	{
		switch(this->can_rev->rx_buf[0])
			{	
				case 0x30:
				{
					this->RMD_X.anglePidKp=this->can_rev->rx_buf[2];
					this->RMD_X.anglePidKi=this->can_rev->rx_buf[3];
					this->RMD_X.speedPidKp=this->can_rev->rx_buf[4];
					this->RMD_X.speedPidKi=this->can_rev->rx_buf[5];
					this->RMD_X.iqPidKp=this->can_rev->rx_buf[6];
					this->RMD_X.iqPidKi=this->can_rev->rx_buf[7];
					 break;
				}			
				case 0x33:
				{					
					this->RMD_X.Accel=(this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8)|(this->can_rev->rx_buf[6]<<16)|(this->can_rev->rx_buf[7]<<24));					
					break;
				}
				case 0x90:
				{
					this->RMD_X.encoder = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));
			  	this->RMD_X.encoderRaw = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));
				  this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						
					break;
				}
				case 0x19:
				{
					this->RMD_X.encoderOffset = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						
					break;
				}
				case 0x92:
				{
					this->RMD_X.motorAngle=(this->can_rev->rx_buf[1]|this->can_rev->rx_buf[2]<<8|(this->can_rev->rx_buf[3]<<16)|(this->can_rev->rx_buf[4]<<24));	//没完整				
					break;
				}
				case 0x94:
				{
					this->RMD_X.circleAngle  = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));
					break;
				}
			  case 0x9A:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
          this->RMD_X.voltage = (this->can_rev->rx_buf[3]|(this->can_rev->rx_buf[4]<<8));
					this->RMD_X.eerorState  = this->can_rev->rx_buf[7]&0x09; //1 低压保护，8 过温保护 ，9低压保护，过温保护
					break;
				}
			  case 0x9C:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));				
					break;
				}				
				case 0x9D:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iA = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.iB = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.iC = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));				
					break;
				}				
			  case 0xA1:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						
					break;
				}								
				case 0xA2:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));					         
          break;	
				}		      
				case 0xA3:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));					         
          break;	
				}				
				case 0xA4:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));				         
          break;	
				}	
				case 0xA5:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));					         
          break;	
				}		
				case 0xA6:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						         
          break;	
				}	
				case 0xA7:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						         
          break;	
				}	
				case 0xA8:
				{
					this->RMD_X.temperature = this->can_rev->rx_buf[1];
	        this->RMD_X.iq = (this->can_rev->rx_buf[2]|(this->can_rev->rx_buf[3]<<8));;
	        this->RMD_X.speed = (this->can_rev->rx_buf[4]|(this->can_rev->rx_buf[5]<<8));;
	        this->RMD_X.now_encoder = (this->can_rev->rx_buf[6]|(this->can_rev->rx_buf[7]<<8));						         
          break;	
				}					
			}		
		return HAL_OK;
	}
	else return HAL_ERROR;
}