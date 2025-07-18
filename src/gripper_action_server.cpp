#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <dh_ag95_gripper/action/gripper_command.hpp>

class GripperActionServer : public rclcpp::Node
{
public:
    using GripperCommand = dh_ag95_gripper::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ServerGoalHandle<GripperCommand>;

    explicit GripperActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_action_server", options)
    {
        // Action服务器
        this->action_server_ = rclcpp_action::create_server<GripperCommand>(
            this,
            "gripper_command",  // Action名称
            std::bind(&GripperActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GripperActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&GripperActionServer::handle_accepted, this, std::placeholders::_1));

        // 发布者：发送夹爪控制命令
        gripper_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "gripper_position_controller/commands", 10);

        // 订阅者：接收夹爪状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&GripperActionServer::joint_state_callback, this, std::placeholders::_1));

        // 定时器：定期检查任务状态
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&GripperActionServer::timer_callback, this));

        // 初始化状态
        current_position_ = 0.0;
        current_effort_ = 0.0;
        goal_handle_ = nullptr;

        RCLCPP_INFO(this->get_logger(), "夹爪Action服务器已启动");
    }

private:
    rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<GoalHandleGripperCommand> goal_handle_;
    double current_position_;
    double current_effort_;
    double target_position_;
    double max_effort_;
    std::chrono::steady_clock::time_point start_time_;

    // 处理新的目标请求
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperCommand::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "收到夹爪控制请求: 目标位置=%.3f, 最大力=%.3f", 
                    goal->position, goal->max_effort);

        // 检查目标位置是否合法
        if (goal->position < 0.0 || goal->position > 1.0) {
            RCLCPP_WARN(this->get_logger(), "目标位置超出范围 [0.0, 1.0]: %.3f", goal->position);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // 如果已有任务在执行，拒绝新请求（或者可以选择取消旧任务）
        if (goal_handle_ && goal_handle_->is_active()) {
            RCLCPP_WARN(this->get_logger(), "已有任务在执行，拒绝新请求");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 处理取消请求
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "收到取消请求");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 处理被接受的目标
    void handle_accepted(const std::shared_ptr<GoalHandleGripperCommand> goal_handle)
    {
        goal_handle_ = goal_handle;
        const auto goal = goal_handle->get_goal();
        target_position_ = goal->position;
        max_effort_ = goal->max_effort > 0 ? goal->max_effort : 50.0;  // 默认最大力
        start_time_ = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "开始执行夹爪控制任务");

        // 发送控制命令
        send_gripper_command(target_position_);
    }

    // 发送夹爪控制命令
    void send_gripper_command(double position)
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {position, -position};  // 两个关节相反运动
        gripper_cmd_pub_->publish(msg);
        
        RCLCPP_DEBUG(this->get_logger(), "发送夹爪命令: 位置=%.3f", position);
    }

    // 接收关节状态回调
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 查找夹爪关节
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "gripper_finger1_joint") {
                if (i < msg->position.size()) {
                    current_position_ = msg->position[i];
                }
                if (i < msg->effort.size()) {
                    current_effort_ = std::abs(msg->effort[i]);
                }
                break;
            }
        }
    }

    // 定时器回调：检查任务状态并发送反馈
    void timer_callback()
    {
        if (!goal_handle_ || !goal_handle_->is_active()) {
            return;
        }

        // 创建反馈消息
        auto feedback = std::make_shared<GripperCommand::Feedback>();
        feedback->position = current_position_;
        feedback->effort = current_effort_;
        feedback->stalled = current_effort_ > max_effort_;

        // 发送反馈
        goal_handle_->publish_feedback(feedback);

        // 检查任务完成条件
        bool reached_goal = std::abs(current_position_ - target_position_) < 0.01;  // 1cm精度
        bool stalled = current_effort_ > max_effort_;
        bool timeout = std::chrono::steady_clock::now() - start_time_ > std::chrono::seconds(10);

        if (reached_goal || stalled || timeout) {
            // 任务完成，发送结果
            auto result = std::make_shared<GripperCommand::Result>();
            result->position = current_position_;
            result->effort = current_effort_;
            result->stalled = stalled;
            result->reached_goal = reached_goal;

            if (reached_goal) {
                goal_handle_->succeed(result);
                RCLCPP_INFO(this->get_logger(), "夹爪到达目标位置: %.3f", current_position_);
            } else if (stalled) {
                goal_handle_->abort(result);
                RCLCPP_WARN(this->get_logger(), "夹爪因力过大而停止: 力=%.3f", current_effort_);
            } else if (timeout) {
                goal_handle_->abort(result);
                RCLCPP_WARN(this->get_logger(), "夹爪控制超时");
            }

            goal_handle_ = nullptr;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 