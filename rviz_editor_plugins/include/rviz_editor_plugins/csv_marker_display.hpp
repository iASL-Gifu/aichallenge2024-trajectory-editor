#ifndef RVIZ_EDITOR_PLUGINS__CSV_MARKER_DISPLAY_HPP_
#define RVIZ_EDITOR_PLUGINS__CSV_MARKER_DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include "editor_tool_srvs/srv/load_csv.hpp"
#include "editor_tool_srvs/srv/select_range.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#endif

namespace rviz_editor_plugins {
class CsvMarkerDisplay : public rviz_common::Panel {
  Q_OBJECT
public:
  CsvMarkerDisplay(QWidget *parent = nullptr);

  virtual void onInitialize();
  virtual void load(const rviz_common::Config &config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void tick();

protected:
  rclcpp::Node::SharedPtr node_;
  QLabel *label_;
  QPushButton *load_button_;
  QPushButton *save_button_;
  rclcpp::Client<editor_tool_srvs::srv::LoadCsv>::SharedPtr load_client_;
  void loadCsv();
  void saveCsv();
};

} // namespace rviz_editor_plugins

#endif // RVIZ_EDITOR_PLUGINS__CSV_MARKER_DISPLAY_HPP_
