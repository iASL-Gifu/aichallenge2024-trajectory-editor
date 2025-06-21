#include "rviz_editor_plugins/editor_tool.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>
#include "editor_tool_srvs/srv/select_range.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rviz_editor_plugins
{
  EditorTool::EditorTool(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    node_ = rclcpp::Node::make_shared("editor_tool");

    mode_label_ = new QLabel("Mode: Normal");
    velocity_edit_ = new QLineEdit("0.0");
    select_range_button_ = new QPushButton("Select Range");
    start_parallel_button_ = new QPushButton("Start Parallel Move");
    confirm_parallel_button_ = new QPushButton("End Parallel Move");
    undo_button_ = new QPushButton("Undo");
    redo_button_ = new QPushButton("Redo");
    post_button_ = new QPushButton("Post Trajectory");

    select_range_client_ = node_->create_client<editor_tool_srvs::srv::SelectRange>("select_range");
    start_parallel_client_ = node_->create_client<std_srvs::srv::Trigger>("start_parallel_move");
    confirm_parallel_client_ = node_->create_client<std_srvs::srv::Trigger>("confirm_parallel_move");
    undo_client_ = node_->create_client<std_srvs::srv::Trigger>("undo");
    redo_client_ = node_->create_client<std_srvs::srv::Trigger>("redo");
    publish_trajectory_client_ = node_->create_client<std_srvs::srv::Trigger>("publish_trajectory");

    QVBoxLayout * layout = new QVBoxLayout;
    QGridLayout * grid_layout_parallel = new QGridLayout;
    QGridLayout * grid_layout_unredo = new QGridLayout;
    layout->addWidget(mode_label_);
    layout->addWidget(velocity_edit_);
    layout->addWidget(select_range_button_);
    grid_layout_parallel->addWidget(start_parallel_button_, 0, 0);
    grid_layout_parallel->addWidget(confirm_parallel_button_, 0, 1);
    grid_layout_unredo->addWidget(undo_button_, 0, 0);
    grid_layout_unredo->addWidget(redo_button_, 0, 1);
    layout->addLayout(grid_layout_parallel);
    layout->addLayout(grid_layout_unredo);
    layout->addWidget(post_button_);
    
    setLayout(layout);

    connect(select_range_button_, &QPushButton::clicked, this, &EditorTool::startSelection);
    connect(start_parallel_button_, &QPushButton::clicked, this, &EditorTool::stratParallelMove);
    connect(confirm_parallel_button_, &QPushButton::clicked, this, &EditorTool::confirmParallelMove);
    connect(undo_button_, &QPushButton::clicked, this, &EditorTool::undo);
    connect(redo_button_, &QPushButton::clicked, this, &EditorTool::redo);
    connect(post_button_, &QPushButton::clicked, this, &EditorTool::postTrajectory);

  }

  void EditorTool::onInitialize()
  {
    // Initialization logic if needed
    RCLCPP_INFO(node_->get_logger(), "EditorTool initialized");
  }

  void EditorTool::load(const rviz_common::Config & config)
  {
    Panel::load(config);
  }

  void EditorTool::save(rviz_common::Config config) const
  {
    Panel::save(config);
  }

  void EditorTool::tick()
  {
    // Periodic update logic if needed
  }
  void EditorTool::startSelection()
  {
    // Start selection logic
    auto request = std::make_shared<editor_tool_srvs::srv::SelectRange::Request>();
    std::string velocity_str = velocity_edit_->text().toStdString();
    if (velocity_str.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Velocity input is empty");
      return;
    }
    try {
      std::stod(velocity_str); // Validate if it's a number
    } catch (const std::invalid_argument & e) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid velocity input: %s", e.what());
      return;
    } catch (const std::out_of_range & e) {
      RCLCPP_ERROR(node_->get_logger(), "Velocity out of range: %s", e.what());
      return;
    }
    request->velocity = std::stod(velocity_str);
    mode_label_->setText(QString("Mode: Selection (Velocity: %1)").arg(velocity_str.c_str()));

    if (select_range_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = select_range_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Selection started with velocity: %.2f", request->velocity);
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start selection");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }

  void EditorTool::stratParallelMove()
  {
    // Start parallel move logic
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (start_parallel_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = start_parallel_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Parallel move started");
        mode_label_->setText(QString("Mode: Parellel Move"));
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start parallel move");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }

  void EditorTool::confirmParallelMove()
  {
    // Confirm parallel move logic
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (confirm_parallel_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = confirm_parallel_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Parallel move confirmed");
        mode_label_ -> setText(QString("Mode: Normal"));
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to confirm parallel move");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }

  void EditorTool::undo()
  {
    // Undo logic
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (undo_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = undo_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Undo executed");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute undo");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }

  void EditorTool::redo()
  {
    // Redo logic
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (redo_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = redo_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Redo executed");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute redo");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }

  void EditorTool::postTrajectory()
  {
    // Post trajectory logic
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (publish_trajectory_client_->wait_for_service(std::chrono::seconds(1))) {
      auto future = publish_trajectory_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "Trajectory posted");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to post trajectory");
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Service not available");
    }
  }
} // namespace rviz_editor_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_editor_plugins::EditorTool, rviz_common::Panel)
