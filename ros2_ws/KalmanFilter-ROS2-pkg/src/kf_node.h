#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <queue>

#include "bram_uio.h"
#include "xkalmanfilterkernel.h"
#include "../includes/kalmanfilter.h"

// custom vitis code

using namespace std;

struct pos_t
{
  float x;
  float y;
  float z;
};

struct acc_t
{
  float ax;
  float ay;
  float az;
};

BRAM bram0(0, 8000);
BRAM bram1(1, 8000);

class KFNode : public rclcpp::Node
{
public:
  KFNode(const std::string &node_name = "kf_node", const std::string &node_namespace = "kf");
  ~KFNode();

private:
  // ROS2 subscribers
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pos_meas_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr control_input_sub_;
  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pos_est_pub_;

  // Callback functions
  void pos_meas_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void control_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // Publish estimated position
  void publish_pos_est(pos_t pos_est);

  queue<pos_t> pos_meas_queue;
  queue<acc_t> control_input_queue;

  // Add here BRAM and xkalmanfilterkernel objects
  pos_t kalmanestimation(pos_t pos_meas, acc_t acc_meas);
};

int main(int argc, char **argv);