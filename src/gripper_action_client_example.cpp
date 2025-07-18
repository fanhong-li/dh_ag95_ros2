#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dh_ag95_gripper/action/gripper_command.hpp>

class GripperActionClient : public rclcpp::Node
{
public:
    using GripperCommand = dh_ag95_gripper::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    explicit GripperActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("gripper_action_client", options)
    {
        // 创建Action客户端
        this->client_ptr_ = rclcpp_action::create_client<GripperCommand>(
            this, "gripper_command");

        RCLCPP_INFO(this->get_logger(), "夹爪Action客户端已启动");
    }

    // 发送夹爪控制目标
    void send_goal(double position, double max_effort = 50.0)
    {
        // 等待Action服务器
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action服务器不可用");
            return;
        }

        // 创建目标
        auto goal_msg = GripperCommand::Goal();
        goal_msg.position = position;
        goal_msg.max_effort = max_effort;

        RCLCPP_INFO(this->get_logger(), "发送夹爪控制目标: 位置=%.3f, 最大力=%.3f", 
                    position, max_effort);

        // 设置选项
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        
        // 目标响应回调
        send_goal_options.goal_response_callback =
            std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
        
        // 反馈回调
        send_goal_options.feedback_callback =
            std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        // 结果回调
        send_goal_options.result_callback =
            std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);

        // 发送目标
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    // 取消当前目标
    void cancel_goal()
    {
        if (goal_handle_) {
            RCLCPP_INFO(this->get_logger(), "取消当前目标");
            this->client_ptr_->async_cancel_goal(goal_handle_);
        }
    }

private:
    rclcpp_action::Client<GripperCommand>::SharedPtr client_ptr_;
    GoalHandleGripperCommand::SharedPtr goal_handle_;

    // 目标响应回调
    void goal_response_callback(const GoalHandleGripperCommand::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标被接受，开始执行");
            goal_handle_ = goal_handle;
        }
    }

    // 反馈回调
    void feedback_callback(
        GoalHandleGripperCommand::SharedPtr,
        const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), 
                    "反馈: 位置=%.3f, 力=%.3f, 卡住=%s", 
                    feedback->position, 
                    feedback->effort,
                    feedback->stalled ? "是" : "否");
    }

    // 结果回调
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
    {
        goal_handle_ = nullptr;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "目标成功完成！");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "目标被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "目标被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "未知结果代码");
                break;
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "最终结果: 位置=%.3f, 力=%.3f, 卡住=%s, 到达目标=%s",
                    result.result->position,
                    result.result->effort,
                    result.result->stalled ? "是" : "否",
                    result.result->reached_goal ? "是" : "否");
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto client = std::make_shared<GripperActionClient>();
    
    // 示例：控制夹爪
    RCLCPP_INFO(client->get_logger(), "=== 夹爪控制示例 ===");
    
    // 1. 打开夹爪
    RCLCPP_INFO(client->get_logger(), "1. 打开夹爪");
    client->send_goal(0.0, 30.0);  // 位置0.0 = 完全打开
    
    // 等待3秒
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    // 2. 关闭夹爪
    RCLCPP_INFO(client->get_logger(), "2. 关闭夹爪");
    client->send_goal(1.0, 50.0);  // 位置1.0 = 完全关闭
    
    // 等待3秒
    rclcpp::sleep_for(std::chrono::seconds(3));
    
    // 3. 半开状态
    RCLCPP_INFO(client->get_logger(), "3. 半开状态");
    client->send_goal(0.5, 40.0);  // 位置0.5 = 半开
    
    // 保持运行以接收回调
    rclcpp::spin(client);
    
    rclcpp::shutdown();
    return 0;
} 