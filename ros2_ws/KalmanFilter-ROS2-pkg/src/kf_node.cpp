#include "kf_node.h"

KFNode::KFNode(const std::string & node_name, const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace) {

  // Custom code here to initialize BRAM and xkalmanfilterkernel
  // ...
  bram_uio::init();
  xkalmanfilterkernel::init();

  // Initialize subscribers
  pos_meas_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/sensor/pos_measurement", 
    10, 
    std::bind(&KFNode::pos_meas_callback, this, std::placeholders::_1)
  );
  control_input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/controller/control_input", 
    10, 
    std::bind(&KFNode::control_input_callback, this, std::placeholders::_1)
  );

  // Initialize publishers
  pos_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/kf/pos_est", 10);

}

KFNode::~KFNode() {
  // Custom code here to close BRAM and xkalmanfilterkernel
  // ...
}

void KFNode::pos_meas_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  pos_t pos_meas;
  pos_meas.x = msg->data[0];
  pos_meas.y = msg->data[1];
  pos_meas.z = msg->data[2];
  pos_meas_queue.push(pos_meas);

  

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
  if(!pos_meas_queue.empty() && !control_input_queue.empty()){
    
    kalmanestimator(pos_meas_queue.front(), control_input_queue.front())

    pos_meas_queue.pop_front();
    control_input_queue.pop_front();    

    //Publish pos est 
    KFNode::publish_pos_est(out_pos_est);
  }

}

void KFNode::control_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  acc_t control_input;
  control_input.ax = msg->data[0];
  control_input.ay = msg->data[1];
  control_input.az = msg->data[2];
  control_input_queue.push(control_input);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  if(!pos_meas_queue.empty() && !control_input_queue.empty()){
    
    kalmanestimator(pos_meas_queue.front(), control_input_queue.front())

    pos_meas_queue.pop_front();
    control_input_queue.pop_front();    

    //Publish pos est 
    KFNode::publish_pos_est(out_pos_est);
  }

}

pos_t kalmanestimator ( pos_t pos_meas, acc_t acc_meas){
    // Data to array for kalmankernel
    float posctrl_input [6];
    posctrl_input[0] = pos_meas.x;
    posctrl_input[1] = pos_meas.y;
    posctrl_input[2] = pos_meas.z;
    posctrl_input[3] = acc_meas.ax;
    posctrl_input[4] = acc_meas.ay;
    posctrl_input[5] = acc_meas.az;

    //Init 
    float temp_data [6];

    //Run vitis function
    temp_data = kalmanfilterkernel(posctrl_input)

    //Parse relevant pos data to out_pos_est
    out_pos_est.x = temp_data[0];
    out_pos_est.y = temp_data[1];
    out_pos_est.z = temp_data[2];
    
}
void KFNode::publish_pos_est(pos_t pos_est) {
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

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
    auto node = std::make_shared<KFNode>();

    rclcpp::spin(node);

	rclcpp::shutdown();
  return 0;
}
