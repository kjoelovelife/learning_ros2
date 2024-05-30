#include "lesson7_cmake/teleop_in_terminal.hpp"

  TeleopInTerminal::TeleopInTerminal(std::string node_name)
        : Node(node_name) {
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);

    RCLCPP_INFO(this->get_logger(),this->msg_.c_str());

    while(rclcpp::ok()){
        // get the pressed key
        this->key_ = this->get_key_();
        if (this->key_ == 'w')
            this->forward_();
        else if (this->key_ == 'x')
            this->backward_();
        else if (this->key_ == 'a')
            this->turn_left_();
        else if (this->key_ == 'd')
            this->turn_right_();
        else if (this->key_ == 's')
            this->stop_();
        else {
            this->stop_();
            if (this->key_ == '\x03')
            break;

        }
    }

  }