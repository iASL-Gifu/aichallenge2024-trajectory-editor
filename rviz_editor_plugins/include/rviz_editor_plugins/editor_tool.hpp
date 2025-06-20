#ifndef RVIZ_EDITOR_PLUGINS__EDITOR_TOOL_HPP_
#define RVIZ_EDITOR_PLUGINS__EDITOR_TOOL_HPP_

#ifndef Q_MOC_RUN
#include "editor_tool_srvs/srv/select_range.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

namespace rviz_editor_plugins {
class EditorTool : public rviz_common::Panel {
  Q_OBJECT
public:
  EditorTool(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void tick();

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<editor_tool_srvs::srv::SelectRange>::SharedPtr
      select_range_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_parallel_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr confirm_parallel_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr undo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr redo_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr publish_trajectory_client_;

  QLabel *mode_label_;
  QLineEdit *velocity_edit_;
  QPushButton *select_range_button_;
  QPushButton *start_parallel_button_;
  QPushButton *confirm_parallel_button_;
  QPushButton *undo_button_;
  QPushButton *redo_button_;
  QPushButton *post_button_;

  void startSelection();
  void stratParallelMove();
  void confirmParallelMove();
  void undo();
  void redo();
  void postTrajectory();
};

} // namespace rviz_editor_plugins

#endif // RVIZ_EDITOR_PLUGINS__EDITOR_TOOL_HPP_
