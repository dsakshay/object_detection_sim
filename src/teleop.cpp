#include <memory.h>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <iostream>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;

geometry_msgs::msg::Twist vel;

bool exit_loop;

rclcpp::WallRate loop_rate(30);

std::mutex vel_mtx;

std::map<char, std::vector<float>> moveBindings{
  {'w', {0.01, 0}}, {'x', {-0.01, 0}}, {'a', {0, 0.01}}, {'d', {0, -0.01}}, {'s', {0, 0}},

};

void printMessage()
{
  std::cout << "Reading from keyboard and publishing on cmd_vel...\n";
  std::cout << "---------------------------------------------------\n";

  std::cout << "To move around use: \n";
  std::cout << "       w     " << std::endl;
  std::cout << "  a    s    d" << std::endl;
  std::cout << "       x     " << std::endl;

  std::cout << "\nPress the key again to increase the velocity...\n";
}

int getCh()
{
  // Note: Termios Setting copied from https://github.com/methylDragon/teleop_twist_keyboard_cpp/blob/master/src/teleop_twist_keyboard.cpp
  int ch;
  struct termios oldT;
  struct termios newT;

	// Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldT);
  newT = oldT;

	// Make required changes and apply the settings
  newT.c_lflag &= ~(ICANON | ECHO);
  newT.c_iflag |= IGNBRK;
  newT.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newT.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ISIG | IEXTEN);
  newT.c_cc[VMIN] = 1;
  newT.c_cc[VTIME] = 0;

  tcsetattr(fileno(stdin), TCSANOW, &newT);

	// Get the current character
  ch = getchar();

	// Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldT);

  return ch;
}

void printVel(const geometry_msgs::msg::Twist & vel_msg)
{
  std::cout << "Currently: lin_x is " << vel_msg.linear.x << " ang_z is " << vel_msg.angular.z << std::endl;
}

void velTimer()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> vel_lock(vel_mtx);
    pub->publish(vel);
    vel_lock.unlock();

    loop_rate.sleep();
    if (exit_loop) {
      break;
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop");
  pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(node->get_logger(), "Starting Teleop Node\n");

  printMessage();

  std::thread worker(velTimer);
  char key(' ');
  float x(0), ang_z(0);

  exit_loop = false;

  while (rclcpp::ok()) {
    try {
      key = getCh();
      if (moveBindings.count(key) > 0) {
        if (key != 's') {
          x = x + moveBindings[key][0];
          ang_z = ang_z + moveBindings[key][1];

        } else {
          x = moveBindings[key][0];
          ang_z = moveBindings[key][1];

          std::cout << "\n\n" << std::endl;
          printMessage();
        }
      }
      if (key == '\x03')  // CTRL + C
      {
        RCLCPP_INFO(node->get_logger(), "Closing Teleop Node\n");

        exit_loop = true;

        break;
      }

      std::unique_lock<std::mutex> vel_lock(vel_mtx);

      vel.linear.x = x;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = ang_z;

      printVel(vel);
      vel_lock.unlock();

    } catch (const rclcpp::exceptions::RCLError & err) {
      RCLCPP_ERROR(node->get_logger(), "Unexpectedly failed with %s", err.what());
    }
    loop_rate.sleep();
  }
  worker.join();
  rclcpp::shutdown();
  return 0;
}

