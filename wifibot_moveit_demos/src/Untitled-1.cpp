#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ncurses.h>

void moveJoint(moveit::planning_interface::MoveGroupInterface& move_group, int joint_index, double delta)
{
  // Get the current joint values
  std::vector<double> joint_values = move_group.getCurrentJointValues();

  // Update the joint value based on the desired movement
  joint_values[joint_index] += delta;

  // Set the updated joint values
  move_group.setJointValueTarget(joint_values);

  // Plan and execute the motion
  move_group.move();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pincher_arm_control");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface group("arm");

  initscr();
  raw();
  keypad(stdscr, TRUE);
  noecho();

  printw("Press 'w' to move the arm forward, 's' to move backward, 'a' to move left, 'd' to move right.\n");
  printw("Press 'q' to exit.\n");

  int key;
  while ((key = getch()) != 'q') {
    double joint_delta = 0.1;  // Adjust as needed

    switch (key) {
      case 'w':
        moveJoint(group, 0, joint_delta);  // Move the first joint
        break;
      case 's':
        moveJoint(group, 0, -joint_delta);  // Move the first joint
        break;
      case 'a':
        moveJoint(group, 1, joint_delta);  // Move the second joint
        break;
      case 'd':
        moveJoint(group, 1, -joint_delta);  // Move the second joint
        break;
      // Add more cases for other desired movements

      default:
        break;
    }
  }

  endwin();

  ros::waitForShutdown();
  return 0;
}
