// #define PLOTTING_ENABLED
#include <ct/core/core.h>
#include <ct/optcon/optcon.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

double sensor[4];
void joint_states_callback(const sensor_msgs::JointStateConstPtr & joint_states_msg)
{
    sensor[0] = joint_states_msg->position[8];
    sensor[1] = joint_states_msg->position[7];
    sensor[2] = joint_states_msg->velocity[8];
    sensor[3] = joint_states_msg->velocity[7];
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "controller");
    std_msgs::Float64 joint_torque[2];
    joint_torque[0].data = 0;
    joint_torque[1].data = 0;
    ros::NodeHandle n;

    ros::Publisher rf_hfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller02/command", 1);
    ros::Publisher rf_kfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller03/command", 1);
    // ct::core::ControlVector<2> u;
    int cnt = 0;
    ros::Rate rate(1000);
    while(ros::ok())
    {
        rf_hfe_pub.publish(joint_torque[0]);
        rf_kfe_pub.publish(joint_torque[1]);
        std::cout << "\rcnt = " << cnt;
        rate.sleep();
        cnt++;
        if(cnt > 1000)
        {
            std::cout << std::endl;
            break;
        }
    }
    return 0;
}