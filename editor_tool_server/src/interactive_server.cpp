#include <chrono>
#include <memory>
#include <string>
#include <fstream>

#include "editor_tool_server/interactive_server.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
// #include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
// #include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "editor_tool_srvs/srv/load_csv.hpp"

// using std::placeholders::_1;

namespace editor_tool_server
{
  EditorToolServer::EditorToolServer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("editor_tool_server", options)
  {
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "editor_tool_server",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface());
    load_csv_service_ = this->create_service<editor_tool_srvs::srv::LoadCsv>("load_csv", std::bind(&EditorToolServer::LoadCsv,this ,std::placeholders::_1, std::placeholders::_2));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("race_trajectory", 10);
  }

  void EditorToolServer::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    std::ostringstream oss;
    oss << "Feedback from marker '" << feedback->marker_name << "' " <<
      " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid) {
      mouse_point_ss << " at " << feedback->mouse_point.x <<
        ", " << feedback->mouse_point.y <<
        ", " << feedback->mouse_point.z <<
        " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type) {
      case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
        oss << ": button click" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        oss << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        oss << ": pose changed" <<
          "\nposition = " <<
          feedback->pose.position.x <<
          ", " << feedback->pose.position.y <<
          ", " << feedback->pose.position.z <<
          "\norientation = " <<
          feedback->pose.orientation.w <<
          ", " << feedback->pose.orientation.x <<
          ", " << feedback->pose.orientation.y <<
          ", " << feedback->pose.orientation.z <<
          "\nframe: " << feedback->header.frame_id <<
          " time: " << feedback->header.stamp.sec << "sec, " <<
          feedback->header.stamp.nanosec << " nsec";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_DOWN:
        oss << ": mouse down" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP:
        oss << ": mouse up" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;
    }

    server_->applyChanges();
  }

  void EditorToolServer::alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    geometry_msgs::msg::Pose pose = feedback->pose;

    pose.position.x = round(pose.position.x - 0.01) + 0.01;
    pose.position.y = round(pose.position.y - 0.01) + 0.01;

    std::ostringstream oss;
    oss << feedback->marker_name << ":" <<
      " aligning position = " <<
      feedback->pose.position.x <<
      ", " << feedback->pose.position.y <<
      ", " << feedback->pose.position.z <<
      " to " <<
      pose.position.x <<
      ", " << pose.position.y <<
      ", " << pose.position.z;
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

    server_->setPose(feedback->marker_name, pose);
    server_->applyChanges();
  }

  void EditorToolServer::LoadCsv(const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request> request, std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response> response)
  {
    std::string fileName = request->filename;
    if (fileName.empty()) {
      RCLCPP_ERROR(get_logger(), "Filename is empty");
      return;
    }
    RCLCPP_INFO(get_logger(), "Loading CSV file: %s", fileName.c_str());
      std::ifstream ifs(fileName);
      if (!ifs.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
        return;
      }
      std::string line;
      if (!std::getline(ifs, line)) {
        RCLCPP_ERROR(get_logger(), "CSV is empty");
        return;
      }
      visualization_msgs::msg::MarkerArray marker_array;
      int id = 0;
      bool any_error = false;

      while (std::getline(ifs, line)) {
        visualization_msgs::msg::Marker marker;
        if (parseLineToMarker(line, id, marker)) {
          makeMoveTrajectoryMarker(marker);
          marker_array.markers.push_back(marker);
          id++;
        } else {
          any_error = true;
        }
      }
      ifs.close();
      server_->applyChanges();
      if (marker_array.markers.empty()) {
        // status_property_->setText("有効なデータが見つかりませんでした");
        RCLCPP_ERROR(get_logger(), "有効なデータが見つかりませんでした");
        return;
      }
      if (markerPointsToVelocityLine(marker_array)) {
        // status_property_->setText("CSV を正常に読み込み、マーカーを表示しました");
        RCLCPP_INFO(get_logger(), "Loaded CSV successfully and displayed markers");
      } else {
        // status_property_->setText("マーカーのライン生成に失敗しました");
        RCLCPP_ERROR(get_logger(), "マーカーのライン生成に失敗しました");
      }
      // marker_pub_->publish(marker_array);
      if (any_error) {
        // status_property_->setText("一部の行でパースエラーが発生しました (ログを確認)");
        RCLCPP_WARN(get_logger(), "一部の行でパースエラーが発生しました (ログを確認)");
      } else {
        // status_property_->setText("CSV を正常に読み込み、マーカーを表示しました");
        RCLCPP_INFO(get_logger(), "Loaded CSV successfully and displayed markers");
      }
      return;
  }

  void EditorToolServer::makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->now();
    int_marker.name = marker.ns + "_" + std::to_string(marker.id);
    int_marker.description = "Move Trajectory Marker";
    int_marker.pose = marker.pose;
    int_marker.scale = 1.0;
    visualization_msgs::msg::InteractiveMarkerControl control;
    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    control.markers.push_back(marker);
    control.always_visible = true;
    int_marker.controls.push_back(control);
    server_->insert(int_marker);
    server_->setCallback(int_marker.name, std::bind(&EditorToolServer::processFeedback, this, std::placeholders::_1));

    server_->setCallback(int_marker.name, std::bind(&EditorToolServer::alignMarker, this, std::placeholders::_1), visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
  }

  bool EditorToolServer::markerPointsToVelocityLine(visualization_msgs::msg::MarkerArray & marker_array)
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
      line_marker.header.stamp = this->now();
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
    last_line_marker.header.stamp = this->now();
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

  bool EditorToolServer::parseLineToMarker(const std::string & line, int id, visualization_msgs::msg::Marker & marker)
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
    marker.header.stamp = this->now();
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

  bool EditorToolServer::saveCsv(const std::string & file_name, const visualization_msgs::msg::MarkerArray & marker_array)
  {
    std::ofstream ofs(file_name);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_name.c_str());
      return false;
    }
    ofs << "x,y,z,qx,qy,qz,qw,speed\n";  // ヘッダ行
    for (const auto & marker : marker_array.markers) {
      ofs << marker.pose.position.x << ","
          << marker.pose.position.y << ","
          << marker.pose.position.z << ","
          << marker.pose.orientation.x << ","
          << marker.pose.orientation.y << ","
          << marker.pose.orientation.z << ","
          << marker.pose.orientation.w << ","
          << std::stod(marker.text) / 3.6 << "\n";  // speed を m/s に変換して保存
    }
    ofs.close();
    RCLCPP_INFO(get_logger(), "CSV saved to %s", file_name.c_str());
    return true;
  }
} // namespace editor_tool_server

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<editor_tool_server::EditorToolServer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}