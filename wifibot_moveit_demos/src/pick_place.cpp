#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_teleop_keyboard");
    ros::NodeHandle nh;

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    ros::Rate loop_rate(10);

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(6);  // Adjust the size based on your robot's joint names
    joint_state.position.resize(6, 0.0);

    joint_state.name[0] = "joint1";  // Replace with your actual joint names
    joint_state.name[1] = "joint2";
    joint_state.name[2] = "joint3";
    joint_state.name[3] = "joint4";
    joint_state.name[4] = "joint5";
    joint_state.name[5] = "joint6";

    char key;

    while (ros::ok())
    {
        std::cout << "Use '1'-'6' to control joints. Press 'q' to exit." << std::endl;
        std::cin >> key;

        if (key == 'q')
        {
            ros::shutdown();
            break;
        }

        int joint_index = key - '1';

        if (joint_index >= 0 && joint_index < joint_state.position.size())
        {
            std::cout << "Enter new position for joint " << key << ": ";
            double new_position;
            std::cin >> new_position;

            joint_state.position[joint_index] = new_position;

            joint_pub.publish(joint_state);
        }
        else
        {
            std::cout << "Invalid joint index. Please use '1'-'6'." << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
