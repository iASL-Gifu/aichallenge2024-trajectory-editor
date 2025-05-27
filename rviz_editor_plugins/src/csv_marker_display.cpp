#include "rviz_editor_plugins/csv_marker_display.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/config.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// #include <QsizePolicy>

namespace rviz_editor_plugins
{
  CsvMarkerDisplay::CsvMarkerDisplay(QWidget * parent)
  : rviz_common::Panel(parent)
  {
    node_ = rclcpp::Node::make_shared("csv_marker_display");

    load_button_ = new QPushButton("Load CSV");
    save_button_ = new QPushButton("Save CSV");
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("race_trajectory", 10);
    // status_property_->setText("CSV を選択してください");
    

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
      std::ifstream ifs(fileName.toStdString());
      if (!ifs.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open file: %s", fileName.toStdString().c_str());
        return;
      }
      std::string line;
      if (!std::getline(ifs, line)) {
        // status_property_->setText("CSV が空です");
        RCLCPP_ERROR(node_->get_logger(), "CSV が空です");
        return;
      }
      visualization_msgs::msg::MarkerArray marker_array;
      int id = 0;
      bool any_error = false;

      while (std::getline(ifs, line)) {
        visualization_msgs::msg::Marker marker;
        if (parseLineToMarker(line, id, marker)) {
          marker_array.markers.push_back(marker);
          id++;
        } else {
          any_error = true;
          RCLCPP_WARN(node_->get_logger(), "CSV の %d 行目をパースできませんでした: [%s]", id + 1, line.c_str());
        }
      }
      ifs.close();
      if (marker_array.markers.empty()) {
        // status_property_->setText("有効なデータが見つかりませんでした");
        RCLCPP_ERROR(node_->get_logger(), "有効なデータが見つかりませんでした");
        return;
      }
      if (markerPointsToVelocityLine(marker_array)) {
        // status_property_->setText("CSV を正常に読み込み、マーカーを表示しました");
        RCLCPP_INFO(node_->get_logger(), "CSV を正常に読み込み、マーカーを表示しました");
      } else {
        // status_property_->setText("マーカーのライン生成に失敗しました");
        RCLCPP_ERROR(node_->get_logger(), "マーカーのライン生成に失敗しました");
      }
      marker_pub_->publish(marker_array);
      if (any_error) {
        // status_property_->setText("一部の行でパースエラーが発生しました (ログを確認)");
        RCLCPP_WARN(node_->get_logger(), "一部の行でパースエラーが発生しました (ログを確認)");
      } else {
        // status_property_->setText("CSV を正常に読み込み、マーカーを表示しました");
        RCLCPP_INFO(node_->get_logger(), "CSV を正常に読み込み、マーカーを表示しました");
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

  bool CsvMarkerDisplay::markerPointsToVelocityLine(visualization_msgs::msg::MarkerArray & marker_array)
  {
    int maker_count = marker_array.markers.size();
    std::vector<visualization_msgs::msg::Marker> line_markers;
    std::vector<visualization_msgs::msg::Marker> velocity_markers;
    for (int i = 0; i < maker_count-1; ++i) {
      visualization_msgs::msg::Marker velocity_marker;
      visualization_msgs::msg::Marker line_marker;
      const auto & marker = marker_array.markers[i];
      const auto & next_marker = marker_array.markers[i + 1];
      geometry_msgs::msg::Point start_point = marker.pose.position;
      geometry_msgs::msg::Point end_point = next_marker.pose.position;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = node_->now();
      line_marker.ns = "line";
      line_marker.id = i;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;

      line_marker.pose.orientation.w = 1.0;  // 単位四元数
      line_marker.scale.x = 0.1;  // 線の太さ
      line_marker.color = marker.color;
      line_marker.color.a = 1.0;  // 不透明度
      line_marker.points.push_back(start_point);
      line_marker.points.push_back(end_point);
      line_markers.push_back(line_marker);

      velocity_marker.header = marker.header;
      velocity_marker.ns = "speed_label";
      velocity_marker.id = i;
      velocity_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      velocity_marker.action = visualization_msgs::msg::Marker::ADD;
      velocity_marker.pose.position.x = start_point.x + 0.02;
      velocity_marker.pose.position.y = start_point.y + 0.02;
      velocity_marker.pose.position.z = start_point.z + 0.1;  // 高さを調整

      velocity_marker.pose.orientation.w = 1.0;  // 単位四元数
      velocity_marker.scale.z = 1.0;  // テキストのサイズ
      velocity_marker.color = marker.color;
      velocity_marker.color.a = 1.0;  // 不透明度
      velocity_marker.text = marker.text;
      velocity_markers.push_back(velocity_marker);
    }
    // last marker is connected to first marker
    visualization_msgs::msg::Marker last_line_marker;
    visualization_msgs::msg::Marker last_velocity_marker;
    const auto & last_marker = marker_array.markers[maker_count - 1];
    const auto & first_marker = marker_array.markers[0];
    geometry_msgs::msg::Point last_start_point = last_marker.pose.position;
    geometry_msgs::msg::Point last_end_point = first_marker.pose.position;
    last_line_marker.header.frame_id = "map";
    last_line_marker.header.stamp = node_->now();
    last_line_marker.ns = "line";
    last_line_marker.id = maker_count - 1;
    last_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    last_line_marker.action = visualization_msgs::msg::Marker::ADD;
    last_line_marker.pose.orientation.w = 1.0;  // 単位四元数
    last_line_marker.scale.x = 0.1;  // 線の太さ
    last_line_marker.color = last_marker.color;
    last_line_marker.color.a = 1.0;  // 不透明度
    last_line_marker.points.push_back(last_start_point);
    last_line_marker.points.push_back(last_end_point);
    line_markers.push_back(last_line_marker);

    marker_array.markers.insert(
      marker_array.markers.end(),
      line_markers.begin(),
      line_markers.end()
    );
    marker_array.markers.insert(
      marker_array.markers.end(),
      velocity_markers.begin(),
      velocity_markers.end()
    );

    return true;
  }

  bool CsvMarkerDisplay::parseLineToMarker(const std::string & line, int id, visualization_msgs::msg::Marker & marker)
  {
    // 1 行をカンマ区切りでトークン化
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> elems;
    while (std::getline(ss, token, ',')) {
      elems.push_back(token);
    }
    // ヘッダ行を除去する処理を呼び出し側で行っているので，
    // ヘッダ以外の行は 8 要素 (x, y, z, x_quat, y_quat, z_quat, w_quat, speed) のはず
    if (elems.size() != 8) {
      return false;
    }
    // 座標
    double x = std::stod(elems[0]);
    double y = std::stod(elems[1]);
    double z = std::stod(elems[2]);
    // 四元数
    double qx = std::stod(elems[3]);
    double qy = std::stod(elems[4]);
    double qz = std::stod(elems[5]);
    double qw = std::stod(elems[6]);
    // speed（今回はマーカーのサイズに反映する例）
    double speed = std::stod(elems[7]);
    speed *= 3.6;  // m/s to km/h

    // Marker の設定
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "arrow";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.text = std::to_string(speed).substr(0, 4);

    // ポーズの設定
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    // 矢印サイズは speed をもとにスケール (調整は自由)
    marker.scale.x = 0.5;        // 矢印の長さ
    marker.scale.y = 0.2;  // 矢印の幅
    marker.scale.z = 0.2;  // 矢印の太さ


    // speedに応じて色を設定
    if (speed < 10.0) {
      marker.color.r = 0.0f;  // 赤
      marker.color.g = 1.0f;  // 緑
      marker.color.b = 0.0f;  // 青
    } else if (speed < 20.0) {
      marker.color.r = 1.0f;  // 赤
      marker.color.g = 1.0f;  // 緑
      marker.color.b = 0.0f;  // 青
    } else {
      marker.color.r = 1.0f;  // 赤
      marker.color.g = 0.0f;  // 緑
      marker.color.b = 0.0f;  // 青
    }
    marker.color.a = 1.0f;  // 不透明度
    // marker.color.r = 0.0f;
    // marker.color.g = 0.0f;
    // marker.color.b = 1.0f;
    // marker.color.a = 1.0f;

    return true;
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_editor_plugins::CsvMarkerDisplay, rviz_common::Panel)