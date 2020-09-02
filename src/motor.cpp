
#include "motor.h"
#include <iostream>


/* constructor */
elmo::Motor::Motor(InputData_t * in, OutputData_t * out):
	in_(in), out_(out), enabled_(false), drive_state_(DriveState::NA)
{
	if (in_ == nullptr || out_ == nullptr)
	{
		//TODO error throw
	}
}

elmo::DriveState elmo::Motor::getDriveState()
{
    //xxxx xxxx x1xx 0000
    if((this->in_->status_word & 0b0000000001001111U) == 0b0000000001000000U)
    {
        this->drive_state_ = DriveState::SwitchOnDisabled;
    }

    //xxxx xxxx x01x 0001
    else if((this->in_->status_word & 0b0000000001101111U) == 0b0000000000100001U)
    {
        this->drive_state_ = DriveState::ReadyToSwitchOn;
    }

    //xxxx xxxx x01x 0011
    else if((this->in_->status_word & 0b0000000001101111U) == 0b0000000000100011U)
    {
        this->drive_state_ = DriveState::SwitchedOn;
    }

    //xxxx xxxx x01x 0111
    else if((this->in_->status_word & 0b0000000001101111U) == 0b0000000000100111U)
    {
        this->drive_state_ = DriveState::OperationEnabled;
    }

    //xxxx xxxx x0xx 1000
    else if((this->in_->status_word & 0b0000000001001111U) == 0b0000000000001000U)
    {
        this->drive_state_ = DriveState::Fault;
    }

    else if((this->in_->status_word & 0b0000000001001111U) == 0b0000000000001111U)
    {
        this->drive_state_ = DriveState::FaultReactionActive;
    }
    return  this->drive_state_;
}

void elmo::Motor::status_control()
{
    getDriveState();
    if (this->enabled_)
    {
        switch(this->drive_state_)
        {
            case DriveState::SwitchOnDisabled:{
                this->out_->control_word = 0x0006U;
                break;
            }
            case DriveState::ReadyToSwitchOn:{
                this->out_->control_word = 0x007U;
                break;
            }
            case DriveState::SwitchedOn:
            case DriveState::OperationEnabled:{
                this->out_->control_word = 0x00FU;
                break;
            }
            default: break;
        }
    }
    else
    {
        this->out_->operation_mode = 0x0000U;
    }
    if(this->drive_state_ == DriveState::Fault || this->drive_state_ == DriveState::FaultReactionActive)
    {
        this->out_->control_word |= 0x0080U;
    }
}


void elmo::Motor::setOpMode(int8 mode)
{
	this->out_->operation_mode = mode;
}

char elmo::Motor::getOpMode()
{
    return this->in_->operation_mode;
}

int32 elmo::Motor::position_cmd()
{
    return this->out_->target_position;
}


void elmo::Motor::position_cmd(int target)
{
    this->out_->target_position = target;
}

void elmo::Motor::velocity_cmd(int target)
{
    if(this->in_->operation_mode == elmo::op_mode::CS::velocity)
    {
        this->out_->target_velocity = target;
    }
    else if(this->in_->operation_mode == elmo::op_mode::CS::position)
    {
        this->out_->offset_velocity = target;
    } else{
        this->out_->target_velocity = target;
    }

}

int32 elmo::Motor::velocity_cmd()
{
    if(this->in_->operation_mode == elmo::op_mode::CS::velocity)
    {
        return this->out_->target_velocity;
    }
    else if(this->in_->operation_mode == elmo::op_mode::CS::position)
    {
        return this->out_->offset_velocity;
    }
    else return 0;
}
static const int max_torque = 400;

static short torque_limit(short & target)
{
    if (target > max_torque)
    {
        target = max_torque;
    }
    else if(target < -max_torque)
    {
        target = -max_torque;
    }
    return target;
}



void elmo::Motor::torque_cmd(short target)
{
    if(this->in_->operation_mode == elmo::op_mode::CS::torque)
    {
        this->out_->target_torque = torque_limit(target);
    }
    else{
          this->out_->offset_torque = torque_limit(target);
    }
}

int16 elmo::Motor::torque_cmd()
{
    if(this->in_->operation_mode == elmo::op_mode::CS::torque)
    {
        return this->out_->target_torque;
    }
    else {
        return this->out_->offset_torque;
    }
}



void  elmo::Motor::enable(bool e)
{
	this->enabled_ = e;
}


int32 elmo::Motor::getActualPosition() const
{
	return this->in_->actual_position;
}
int32 elmo::Motor::getActualVelocity() const
{
	return this->in_->actual_velocity;
}
short elmo::Motor::getActualTorque() const
{
	return this->in_->actual_torque;
}


