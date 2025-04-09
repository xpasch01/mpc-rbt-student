#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()), x_(-0.5), y_(0.0), theta_(0.0) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    // add code here

    // Subscriber for joint_states
    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));
   

    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    // add code here
    //std::cout << "Received JointState message " << std::endl;
    //for(size_t i = 0; i < msg.name.size(); ++i){
    //	std::cout << "Velocity: " << msg.velocity[i] << std::endl;
    //}
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    if(msg.velocity.size() < 2)
    {
    	RCLCPP_WARN(get_logger(), "Not enough velocity data in joint state message");
    }

    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();
    
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    // add code here

    double linear = robot_config::WHEEL_RADIUS * (left_wheel_vel + right_wheel_vel) / 2.0;
    double angular = robot_config::WHEEL_RADIUS * (left_wheel_vel - right_wheel_vel) / (2*robot_config::HALF_DISTANCE_BETWEEN_WHEELS);  

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta_);

    theta_ += angular * dt;	
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));
    
    x_ += linear * std::cos(theta_) * dt;
    y_ += linear * std::sin(theta_) * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odometry_.pose.pose.orientation = tf2::toMsg(q);
    
    odometry_.pose.pose.position.x = x_;
    odometry_.pose.pose.position.y = y_;
    
    odometry_.twist.twist.linear.x = linear;
    odometry_.twist.twist.angular.z = angular;
    
    //std::cout << "x: " << x_ << std::endl;
    //std::cout << "y: " << y_ << std::endl;
    //std::cout << "linear: " << linear << std::endl;
    //std::cout << "angular: " << angular << std::endl;
    
}

void LocalizationNode::publishOdometry() {
    // add code here
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    // add code here
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;    
    
    tf2::Quaternion q;
    q.setRPY(0,0,theta_);
    t.transform.rotation = tf2::toMsg(q);    
    
    tf_broadcaster_->sendTransform(t);
}

