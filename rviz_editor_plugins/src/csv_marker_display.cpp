#include "rviz_editor_plugins/csv_marker_display.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "editor_tool_srvs/srv/load_csv.hpp"
#include "editor_tool_srvs/srv/select_range.hpp"

// #include <QsizePolicy>

namespace rviz_editor_plugins
{
  CsvMarkerDisplay::CsvMarkerDisplay(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    node_ = rclcpp::Node::make_shared("csv_marker_display");

    label_ = new QLabel("[No CSV loaded]");
    load_button_ = new QPushButton("Load CSV");
    save_button_ = new QPushButton("Save CSV");
    load_client_ = node_->create_client<editor_tool_srvs::srv::LoadCsv>("load_csv");
    

    QVBoxLayout * layout = new QVBoxLayout;
    QGridLayout *grid_layout = new QGridLayout;
    layout->addWidget(label_);
    grid_layout->addWidget(load_button_, 0, 0);
    grid_layout->addWidget(save_button_, 0, 1);
    layout->addLayout(grid_layout);
    setLayout(layout);

    connect(load_button_, &QPushButton::clicked, this, &CsvMarkerDisplay::loadCsv);
    connect(save_button_, &QPushButton::clicked, this, &CsvMarkerDisplay::saveCsv);
  }
  void CsvMarkerDisplay::onInitialize()
  {
    // Initialization logic if needed
    RCLCPP_INFO(node_->get_logger(), "CsvMarkerDisplay initialized");
  }
  void CsvMarkerDisplay::load(const rviz_common::Config & config)
  {
    Panel::load(config);
  }
  void CsvMarkerDisplay::save(rviz_common::Config config) const
  {
    Panel::save(config);
  }
  void CsvMarkerDisplay::tick()
  {
    // Periodic update logic if needed
  }
  void CsvMarkerDisplay::loadCsv()
  {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open CSV File"), "", tr("CSV Files (*.csv)"));
    if (!fileName.isEmpty()) {
      // Logic to load CSV file
      RCLCPP_INFO(node_->get_logger(), "Loading CSV file: %s", fileName.toStdString().c_str());

      auto request = std::make_shared<editor_tool_srvs::srv::LoadCsv::Request>();
      request->filename = fileName.toStdString();
      auto result = load_client_->async_send_request(request);
      label_->setText(QString("file loaded"));
      if (!result.valid()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send request to load CSV");
        label_->setText(QString("Failed to load CSV"));
        return;
      }
    }
  }

  void CsvMarkerDisplay::saveCsv()
  {
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save CSV File"), "", tr("CSV Files (*.csv)"));
    if (!fileName.isEmpty()) {
      // Logic to save CSV file
      RCLCPP_INFO(node_->get_logger(), "Saving CSV file: %s", fileName.toStdString().c_str());
      // Add your CSV saving logic here
    }
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_editor_plugins::CsvMarkerDisplay, rviz_common::Panel)
