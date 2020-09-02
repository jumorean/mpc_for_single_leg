#include "joint.h"
#include <iostream>
using namespace elmo;

Joint::Joint(MotorPtr motor)
    : motor_(motor),
      ratio_(100.0),
      initial_offset_(0),
      polarity_(1.0)
{
    motor_->setOpMode(elmo::op_mode::CS::torque);


    revolution_ = static_cast<double>(1 << 14);
	// actual value
	actual_angular = 0;
	actual_angular_vel = 0;
	actual_torque = 0;
}

using namespace boost::property_tree;

void Joint::config(const boost::property_tree::ptree & pt, const std::string & joint_name)
{
    std::string config_dir_name = configDir;
    this->ratio_ = pt.get<double>("ratio");
    // std::cout << "get " << std::endl;
    ptree pt_polarity;
    read_json(config_dir_name + "/" + pt.get<std::string>("polarity file name"), pt_polarity);

    this->polarity_ = pt_polarity.get<double>(joint_name);

    ptree pt_initial_off;
    read_json(config_dir_name + "/" + pt.get<std::string>(+"zero configuration file name"), pt_initial_off);
    this->initial_offset_ = pt_initial_off.get<int32>(joint_name);
    int encoder_bits = pt.get<int>("encoder bits");
    this->revolution_ = 1.;
    for (int i = 0; i < encoder_bits; i++)
    {
        revolution_ *= 2;
    }

    std::cout << polarity_ << "\t" << initial_offset_ << "\t" << revolution_ << "\t"
    << ratio_ << std::endl;

}

const double pi = 3.14159265358979;

void Joint::setPolarity(double pol)
{
    this->polarity_ = pol;
}

void Joint::setInitPos(int32 pos)
{
    this->initial_offset_ = pos;
}


double Joint::getActualAngular()
{
    actual_angular = this->polarity_ * static_cast<double>(motor_->getActualPosition() - initial_offset_) / revolution_ / ratio_ * 2. * pi;
    return this->actual_angular;
}

double Joint::getActualVelocity()
{
    actual_angular_vel = this->polarity_ * static_cast<double>(motor_->getActualVelocity()) / revolution_ / ratio_ * 2. * pi;
	return this->actual_angular_vel;
}

double Joint::getActualTorque()
{
    actual_torque = this->polarity_ * static_cast<double>(motor_->getActualTorque()) * ratio_ * 3.310999999999999e-4;
	return this->actual_torque;
}



void Joint::angular_cmd(double target)
{
    auto result = initial_offset_ + static_cast<int32>(polarity_ * target / 2 / pi * revolution_ * ratio_);
	this->motor_->position_cmd(result);
}

double Joint::angular_cmd()
{
    return this->polarity_ * static_cast<double>(motor_->position_cmd() - initial_offset_) / revolution_ / ratio_ * 2. * pi;
}

void Joint::velocity_cmd(double target)
{
    auto result = static_cast<int32>(target / 2 / pi * revolution_ * ratio_ * polarity_);
	this->motor_->velocity_cmd(result);
}

void Joint::torque_cmd(double target)
{
	auto result = static_cast<short>(target / ratio_ *3020.235578375114 * polarity_);
	this->motor_->torque_cmd(result);
}

double Joint::torque_cmd()
{
    return (polarity_*static_cast<double>(motor_->torque_cmd()) * ratio_ * 3.310999999999999e-4);
    // return (this->polarity_ * static_cast<double>(motor_->getTargetPosition()-initial_offset_)/revolution_ /ratio_*2.*pi);
}

double Joint::velocity_cmd()
{
    return this->polarity_ * static_cast<double>(motor_->velocity_cmd()) / revolution_ / ratio_ * 2. * pi;
}


void Joint::setOpMode(int8 mode)
{
	motor_->setOpMode(mode);
}

void Joint::enable(bool e)
{
	motor_->enable(e);
}