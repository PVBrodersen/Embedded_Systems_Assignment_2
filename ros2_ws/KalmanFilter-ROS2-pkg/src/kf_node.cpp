#include "kf_node.h"

KFNode::KFNode(const std::string &node_name, const std::string &node_namespace) : rclcpp::Node(node_name, node_namespace)
{

  // Custom code here to initialize BRAM and xkalmanfilterkernel
  // ...

  KalmanFilter::kalman_initialize();

  // Initialize subscribers
  pos_meas_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/sensor/pos_measurement",
      10,
      std::bind(&KFNode::pos_meas_callback, this, std::placeholders::_1));
  control_input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/controller/control_input",
      10,
      std::bind(&KFNode::control_input_callback, this, std::placeholders::_1));

  // Initialize publishers
  pos_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/kf/pos_est", 10);
}

KFNode::~KFNode()
{
  // Custom code here to close BRAM and xkalmanfilterkernel
  // ...
  KalmanFilter::kalman_cleanup();
}

// Pos measurement callback run everytime new pos posted to topic
void KFNode::pos_meas_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  pos_t pos_meas;
  pos_meas.x = msg->data[0];
  pos_meas.y = msg->data[1];
  pos_meas.z = msg->data[2];
  pos_meas_queue.push(pos_meas);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
  if (!pos_meas_queue.empty() && !control_input_queue.empty())
  {

    // Publish pos est using kalmanestimate
    KFNode::publish_pos_est(kalmanestimator(pos_meas_queue.front(), control_input_queue()));

    // Pop queues to allow for new data
    pos_meas_queue.pop_front();
    control_input_queue.pop_front();
  }
}

// Control input callback function run everytime a new control input is posted
// on relevant topic
void KFNode::control_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  acc_t control_input;
  control_input.ax = msg->data[0];
  control_input.ay = msg->data[1];
  control_input.az = msg->data[2];
  control_input_queue.push(control_input);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  if (!pos_meas_queue.empty() && !control_input_queue.empty())
  {

    // Publish pos est
    KFNode::publish_pos_est(kalmanestimator(pos_meas_queue.front(), control_input_queue()));
S
    // Pop queues to allow for new data
    pos_meas_queue.pop_front();
    control_input_queue.pop_front();
  }
}

//* Function used for loading pos and control to bram, running kalman estimate,
// and returning estimated pos
pos_t kalmanestimator(pos_t pos_meas, acc_t acc_meas)
{

  // Data to bram

  bram0[0] = *((uint32_t*)(&pos_meas.x));
  bram0[1] = *((uint32_t*)(&pos_meas.y));
  bram0[2] = *((uint32_t*)(&pos_meas.z));
  bram0[3] = *((uint32_t*)(&acc_meas.ax));
  bram0[4] = *((uint32_t*)(&acc_meas.ay));
  bram0[5] = *((uint32_t*)(&acc_meas.az));

  // kalman estimation
  KalmanFilter::kalman_estimator();

  // Parse relevant pos data to out_pos_est
  pos_t out_pos_est;
  out_pos_est.x = *((float*)(&bram1[0]));
  out_pos_est.y = *(float*)(&bram1[1]);
  out_pos_est.z = *(float*)(&bram1[2]);

  return out_pos_est;
}

// Function used for publishing designated pos in world frame
void KFNode::publish_pos_est(pos_t pos_est)
{
  geometry_msgs::msg::PoseStamped pos_est_msg;
  pos_est_msg.header.stamp = this->get_clock()->now();
  pos_est_msg.header.frame_id = "world";
  pos_est_msg.pose.orientation.w = 1.0;
  pos_est_msg.pose.orientation.x = 0.;
  pos_est_msg.pose.orientation.y = 0.;
  pos_est_msg.pose.orientation.z = 0.;
  pos_est_msg.pose.position.x = pos_est.x;
  pos_est_msg.pose.position.y = pos_est.y;
  pos_est_msg.pose.position.z = pos_est.z;
  pos_est_pub_->publish(pos_est_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KFNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
