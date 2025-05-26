#include "rviz_editor_plugins/csv_marker_display.hpp"
// #include "csv_marker_display.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>

// #include <QsizePolicy>

namespace rviz_editor_plugins
{
  CsvMarkerDisplay::CsvMarkerDisplay(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    node_ = rclcpp::Node::make_shared("csv_marker_display");

    load_button_ = new QPushButton("Load CSV");
    save_button_ = new QPushButton("Save CSV");

    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(load_button_);
    layout->addWidget(save_button_);
    setLayout(layout);

    connect(load_button_, &QPushButton::clicked, this, &CsvMarkerDisplay::loadCsv);
    connect(save_button_, &QPushButton::clicked, this, &CsvMarkerDisplay::saveCsv);
  }
  void CsvMarkerDisplay::onInitialize()
  {
    // Initialization logic if needed
  }
  void CsvMarkerDisplay::load(const rviz_common::Config & config)
  {
    // Load configuration logic if needed
  }
  void CsvMarkerDisplay::save(rviz_common::Config config) const
  {
    // Save configuration logic if needed
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