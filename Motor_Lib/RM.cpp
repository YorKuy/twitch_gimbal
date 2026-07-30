#include "RM.h"

HAL_StatusTypeDef MOTOR_RM::update(void)
{
	if(this->can_rev->RxHeader.StdId == this->ID)
	{
		this->mang=(int16_t) ((this->can_rev->rx_buf[0] << 8) | this->can_rev->rx_buf[1]);
		this->sp=(int16_t) ((this->can_rev->rx_buf[2] << 8) | this->can_rev->rx_buf[3]);
		this->AT_current=(int16_t) ((this->can_rev->rx_buf[4] << 8) | this->can_rev->rx_buf[5]);
		this->temp=this->can_rev->rx_buf[6];

		this->update_mang_inf();
		
		return HAL_OK;
	}
	else return HAL_ERROR;
}

HAL_StatusTypeDef MOTOR_RM::DAMIAO_Update(void)
{
	if(this->can_rev->RxHeader.StdId == this->ID)
	{
		this->mang=(int16_t) ((this->can_rev->rx_buf[0] << 8) | this->can_rev->rx_buf[1]);
		this->sp=(int16_t) ((this->can_rev->rx_buf[2] << 8) | this->can_rev->rx_buf[3]);
		this->AT_current=(int16_t) ((this->can_rev->rx_buf[4] << 8) | this->can_rev->rx_buf[5]);
		this->temp=this->can_rev->rx_buf[6];
		this->Error_State=this->can_rev->rx_buf[7];
		this->update_mang_inf();
		
		return HAL_OK;
	}
	else return HAL_ERROR;
}

void MOTOR_RM::update_mang_inf(void)//µç»ú¹ýÈ¦¼ì²â
{
    if(this->first == 0)this->first=1,this->Last_mang=this->mang;

    if ((this->mang - this->Last_mang) < -5000)
    {
        this->mang_inf += 8191;
			  this->motor_number++;
    }
    else if ((this->mang - this->Last_mang) > 5000)
    {
        this->mang_inf -= 8191;
			  this->motor_number--;
    }
    this->mang_inf -= this->Last_mang;
    this->mang_inf += this->mang;
    this->Last_mang = this->mang;
}