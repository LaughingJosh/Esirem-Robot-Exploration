#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <string>

template<class T>
T readSingleParam(std::string name, T defaultVal)
{
    T param;
    if (!ros::param::get("~" + name, param))
    {
        ROS_WARN("Cannot retrieve the parameter %s", name.c_str());
        param = defaultVal;
    }
    else
    {
        std::cout << name << " : " << param << std::endl;
    }
    return param;
}

class TeleopJoy
{
public:
    TeleopJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    ros::NodeHandle nh_;

    int linear_x_{3}, linear_y_{2}, angular_z_{0};
    double lin_x_slow_{0.1}, lin_y_slow_{0.1}, ang_z_slow_{0.1};
    double lin_x_normal_{0.5}, lin_y_normal_{0.5}, ang_z_normal_{0.5};
    double lin_x_fast_{2.0}, lin_y_fast_{2.0}, ang_z_fast_{2.0};

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

TeleopJoy::TeleopJoy()
{
    linear_x_ = readSingleParam<double>("axis_linear_x", linear_x_);
    linear_y_ = readSingleParam<double>("axis_linear_y", linear_y_);
    angular_z_ = readSingleParam<double>("axis_angular_z", angular_z_);
    lin_x_slow_ = readSingleParam<double>("slow_linear_x", lin_x_slow_);
    lin_y_slow_ = readSingleParam<double>("slow_linear_y", lin_y_slow_);
    ang_z_slow_ = readSingleParam<double>("slow_angular_z", ang_z_slow_);
    lin_x_normal_ = readSingleParam<double>("normal_linear_x", lin_x_normal_);
    lin_y_normal_ = readSingleParam<double>("normal_linear_y", lin_y_normal_);
    ang_z_normal_ = readSingleParam<double>("normal_angular_z", ang_z_normal_);

    lin_x_fast_ = readSingleParam<double>("fast_linear_x", lin_x_fast_);
    lin_y_fast_ = readSingleParam<double>("fast_linear_y", lin_y_fast_);
    ang_z_fast_ = readSingleParam<double>("fast_angular_z", ang_z_fast_);

    std::string robotName = readSingleParam<std::string>("basename", "");

    std::string topicName = "/cmd_vel";
    if (!robotName.empty())
    {
        topicName = "/"+ robotName +"/cmd_vel";
    }

    ROS_WARN("Using topic name: %s", topicName.c_str());
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(topicName, 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;

    if (joy->buttons[4] == 1)
    {
        double lin_x_gain = 0;
        double lin_y_gain = 0;
        double ang_z_gain = 0;
        if (joy->buttons[5] == 1)
        {
            //slow
            lin_x_gain = lin_x_slow_;
            lin_y_gain = lin_y_slow_;
            ang_z_gain = ang_z_slow_;
        }
        else if (joy->buttons[7] == 1)
        {
            //Normal
            lin_x_gain = lin_x_normal_;
            lin_y_gain = lin_y_normal_;
            ang_z_gain = ang_z_normal_;
        }
        else
        {
            //fast
            lin_x_gain = lin_x_fast_;
            lin_y_gain = lin_y_fast_;
            ang_z_gain = ang_z_fast_;
        }
        //slow
        twist.linear.x =  lin_x_gain * joy->axes[linear_x_];
        twist.linear.y =  lin_y_gain * joy->axes[linear_y_];
        twist.angular.z = ang_z_gain * joy->axes[angular_z_];

    }
    else
    {
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = 0.0;
    }

    vel_pub_.publish(twist);
}

int main(int argc, char **argv)
{
    ROS_INFO("Init Node");
    ros::init(argc, argv, "f710_teleop_joy_node");
    TeleopJoy teleop_joy;
    ros::spin();
}
