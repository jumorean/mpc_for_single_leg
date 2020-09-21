#include <ros/ros.h>
#include <Eigen/Eigen>
// #include <ros/ros.h>
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
    ros::init(argc, argv, "initializer");
    std_msgs::Float64 joint_torque[2];
    ros::NodeHandle n;
    ros::Subscriber joint_states_sub = n.subscribe("/pegasus2_model/joint_states", 1, joint_states_callback);

    ros::Publisher rf_hfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller02/command", 1);
    ros::Publisher rf_kfe_pub = n.advertise<std_msgs::Float64>("/pegasus2_model/controller03/command", 1);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    typedef Eigen::Matrix<double, 2, 1> Vector2d;
    Vector2d actual_pos;
    Vector2d target_pos;
    Vector2d actual_vel;
    Vector2d target_vel;
    Vector2d error_pos;
    Vector2d error_vel;
    Vector2d target_torque;
    double kp = 30;
    double kd = 0.5;

    target_pos << -0.608284, 1.25547;
    ros::Rate rate(1000);
    for(int cnt=0;cnt<5000;cnt++)
    {
        actual_pos(0, 0) = sensor[0];
        actual_pos(1, 0) = sensor[1];
        actual_vel(0, 0) = sensor[2];
        actual_vel(1, 0) = sensor[3];
        error_pos = target_pos - actual_pos;
        error_vel = target_vel - actual_vel;

        target_torque = error_pos * kp + error_vel * kd;
        joint_torque[0].data = target_torque(0, 0);
        joint_torque[1].data = target_torque(1, 0);
        rf_hfe_pub.publish(joint_torque[0]);
        rf_kfe_pub.publish(joint_torque[1]);
        rate.sleep();

    }
}
