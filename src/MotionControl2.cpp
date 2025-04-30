#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

        // Subscribers for odometry and laser scans
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));
        
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

        // Publisher for robot control
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Client for path planning
        plan_client_ = this->create_client<nav_msgs::srv::GetPlan>(
            "/plan_path");

        // Action server
        nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this,
            "go_to_goal",
            std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
            std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Motion control node started.");

        // Connect to path planning service server
        while (!plan_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Waiting for path planning service...");
        }
    }

void MotionControlNode::checkCollision() {
    // add code here
    float threshold = 5; // distance threshold for obstacle detection

    for(size_t i = 0; i < laser_scan_.ranges.size(); i++) {
        if (laser_scan_.ranges[i] < threshold) {
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);
            RCLCPP_WARN(get_logger(), "Obstacle detected at angle: %f", laser_scan_.angle_min + i * laser_scan_.angle_increment);
            return;
        }
    }
}

void MotionControlNode::updateTwist() {
    geometry_msgs::msg::Twist twist;

    float dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    float dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;

    float distance = sqrt(dx * dx + dy * dy);

    float v_max = 1.0; // max linear velocity
    float v = std::min(v_max, distance);

    // Convert orientation to tf2::Quaternion
    tf2::Quaternion quat;
    tf2::fromMsg(current_pose_.pose.orientation, quat);

    // Extract yaw using tf2::getYaw
    float current_yaw = tf2::getYaw(quat);

    float angle_to_goal = atan2(dy, dx);
    float angle_diff = angle_to_goal - current_yaw;

    twist.angular.z = 0.5 * angle_diff; // proportional control for angular velocity
    twist.linear.x = v;

    twist_publisher_->publish(twist);
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
    // add code here

    goal_pose_ = goal->pose;

    RCLCPP_INFO(get_logger(), "Received goal: x=%f, y=%f", goal_pose_.pose.position.x, goal_pose_.pose.position.y);

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;
    
    auto future = plan_client_->async_send_request(request,
        std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here

    RCLCPP_INFO(get_logger(), "Canceling goal");
    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    goal_handle_->canceled(result);
    return rclcpp_action::CancelResponse::ACCEPT;
    
}

void MotionControlNode::navHandleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
    // add code here

    goal_handle_ = goal_handle;

    std::thread([this]{
        execute();
    }).detach();
}

void MotionControlNode::execute() {
    // add code here

    rclcpp::Rate loop_rate(100.0); // 1 Hz

    while (rclcpp::ok()) {

        if (goal_handle_->is_canceling()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->canceled(result);
            RCLCPP_INFO(get_logger(), "Goal canceled");
            break;
        }

        updateTwist();

        checkCollision();

        loop_rate.sleep();
    }

    auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    goal_handle_->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future) {
    // add code here
    auto response = future.get();

    if (response && response->plan.poses.size() > 0) {
        path_ = response->plan;
        std::thread(&MotionControlNode::execute, this).detach();
        RCLCPP_INFO(get_logger(), "Path planned successfully. Starting navigation. Path size: %zu", path_.poses.size());
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to plan path.");
    }
    
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    // add code here

    current_pose_.pose = msg.pose.pose;
    current_pose_.header = msg.header;

    checkCollision();
    updateTwist();
    
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    // add code here
    laser_scan_ = msg;
}
