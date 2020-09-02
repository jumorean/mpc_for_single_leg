#pragma once


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "motor.h"
#include "transform.h"
#include "configDir.h"


namespace elmo {

class Joint {
public:
	Joint() = delete;
	// typedef std::shared_ptr<elmo::Motor> MotorPtr;
	typedef Motor * MotorPtr;
    explicit Joint(MotorPtr motor);
	void angular_cmd(double target);
	void velocity_cmd(double target);
	void torque_cmd(double target);
    double angular_cmd();
    double velocity_cmd();
	double torque_cmd();


	void setOpMode(int8 mode);
    void enable(bool enable_);

    void config(const boost::property_tree::ptree & pt, const std::string & joint_name);

    void setPolarity(double pol);
    void setInitPos(int32 pos);


	// void update_state();

	double getActualAngular();
	double getActualVelocity();
	double getActualTorque();
    double revolution_;
    double ratio_;
    // actual value
    double actual_angular;
    double actual_angular_vel;
    double actual_torque;


    int32 initial_offset_;
    double polarity_;

protected:
	MotorPtr motor_;

	
};


} //namespace ethercat


