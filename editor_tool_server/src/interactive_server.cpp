// File: interactive_server.cpp
#include "editor_tool_server/interactive_server.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace editor_tool_server
{
  EditorToolServer::EditorToolServer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("editor_tool_server", options)
  {
    // Initialize the InteractiveMarkerServer
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "editor_tool_server",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface());

    // Service to load CSV (CSV -> markers + interactive markers)
    load_csv_service_ = this->create_service<editor_tool_srvs::srv::LoadCsv>(
      "load_csv",
      std::bind(&EditorToolServer::LoadCsv, this,
                std::placeholders::_1, std::placeholders::_2));

    // Service to save CSV (markers -> CSV)
    // save_csv_service_ = this->create_service<editor_tool_srvs::srv::SaveCsv>(
    //   "save_csv",
    //   std::bind(&EditorToolServer::SaveCsvService, this,
    //             std::placeholders::_1, std::placeholders::_2));

    // Publisher for MarkerArray (trajectory points + lines + speed labels)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "race_trajectory", 10);
  }

  void EditorToolServer::LoadCsv(
    const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response> response)
  {
    const std::string & fileName = request->filename;
    if (fileName.empty()) {
      RCLCPP_ERROR(get_logger(), "LoadCsv: filename is empty");
      // response->success = false;
      return;
    }
    RCLCPP_INFO(get_logger(), "Loading CSV file: %s", fileName.c_str());

    std::ifstream ifs(fileName);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
      // response->success = false;
      return;
    }

    std::string line;
    // Skip header line
    if (!std::getline(ifs, line)) {
      RCLCPP_ERROR(get_logger(), "CSV is empty");
      // response->success = false;
      return;
    }

    // Clear any existing markers and mappings
    trajectory_markers_.clear();
    name_to_index_.clear();
    server_->clear();  // remove existing interactive markers

    int id = 0;
    bool any_error = false;

    // Parse each CSV line into a Marker
    while (std::getline(ifs, line)) {
      visualization_msgs::msg::Marker marker;
      if (parseLineToMarker(line, id, marker)) {
        trajectory_markers_.push_back(marker);

        // Create an interactive marker for this waypoint
        makeMoveTrajectoryMarker(marker);
        std::string name = marker.ns + "_" + std::to_string(marker.id);
        name_to_index_[name] = id;
        id++;
      } else {
        any_error = true;
      }
    }
    ifs.close();

    if (trajectory_markers_.empty()) {
      RCLCPP_ERROR(get_logger(), "No valid data found in CSV");
      // response->success = false;
      return;
    }
    server_->applyChanges();
    // Publish initial MarkerArray (points + lines + speed labels)
    publishMarkers();

    if (any_error) {
      RCLCPP_WARN(get_logger(), "Some lines failed to parse (check log)");
    } else {
      RCLCPP_INFO(get_logger(), "CSV loaded and markers displayed successfully");
    }
    // response->success = true;
  }

  // void EditorToolServer::SaveCsvService(
  //   const std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Request> request,
  //   std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Response> response)
  // {
  //   const std::string & fileName = request->filename;
  //   if (fileName.empty()) {
  //     RCLCPP_ERROR(get_logger(), "SaveCsvService: filename is empty");
  //     response->success = false;
  //     return;
  //   }

  //   // Prepare a temporary MarkerArray containing only the trajectory points
  //   visualization_msgs::msg::MarkerArray tmp_array;
  //   tmp_array.markers = trajectory_markers_;

  //   // Save to CSV
  //   bool result = saveCsv(fileName, tmp_array);
  //   response->success = result;
  //   if (result) {
  //     RCLCPP_INFO(get_logger(), "CSV saved to %s", fileName.c_str());
  //   } else {
  //     RCLCPP_ERROR(get_logger(), "Failed to save CSV to %s", fileName.c_str());
  //   }
  // }

  void EditorToolServer::processFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    std::ostringstream oss;
    oss << "Feedback from marker '" << feedback->marker_name << "' "
        << "/ control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid) {
      mouse_point_ss << " at " << feedback->mouse_point.x
                     << ", " << feedback->mouse_point.y
                     << ", " << feedback->mouse_point.z
                     << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type) {
      case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
        oss << ": button click" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        oss << ": menu item " << feedback->menu_entry_id
            << " clicked" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        break;

      case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        oss << ": pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\norientation = "
            << feedback->pose.orientation.w
            << ", " << feedback->pose.orientation.x
            << ", " << feedback->pose.orientation.y
            << ", " << feedback->pose.orientation.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nanosec << " nsec";
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

void EditorToolServer::alignMarker(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    // 1) もとの位置を取得
    auto it = name_to_index_.find(feedback->marker_name);
    if (it == name_to_index_.end()) {
      // マーカー名が見つからなければ何もしない
      return;
    }
    int idx = it->second;

    // 2) マウスで得られた新しいPoseをグリッド(0.01間隔)にスナップ（必要なら）
    geometry_msgs::msg::Pose raw = feedback->pose;
    raw.position.x = std::round(raw.position.x * 100.0) / 100.0;  // 0.01単位で切り上げ
    raw.position.y = std::round(raw.position.y * 100.0) / 100.0;
    // raw.position.z = std::round(raw.position.z * 100.0) / 100.0; // 必要ならZも

    // 3) インタラクティブマーカー側にも更新を反映（見た目を即座に追従）
    server_->setPose(feedback->marker_name, raw);
    server_->applyChanges();

    // 4) 「もとに保持しているMarker」の古いPoseを取得
    geometry_msgs::msg::Pose old_pose = trajectory_markers_[idx].pose;
    geometry_msgs::msg::Pose target_pose = raw;

    // ── ここで“感度”を調整して、一度に動く量を f 倍に縮小 ──
    const double f = 0.1;  // 10%だけ動かす。0.05 や 0.2 などで調整できる。

    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = old_pose.position.x + f * (target_pose.position.x - old_pose.position.x);
    new_pose.position.y = old_pose.position.y + f * (target_pose.position.y - old_pose.position.y);
    new_pose.position.z = old_pose.position.z + f * (target_pose.position.z - old_pose.position.z);

    // もし回転もゆっくり追従させたいなら slerp（球面線形補間）などを使うか、
    // 以下のように単純にコピーしても良い：
    new_pose.orientation = target_pose.orientation;

    // 5) trajectory_markers_ に新しいPoseをセット
    trajectory_markers_[idx].pose = new_pose;
    trajectory_markers_[idx].header.stamp = this->now();

    // 6) マーカー配列全体（points + lines + speed labels）を再生成して発行
    publishMarkers();
  }

  void EditorToolServer::makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->now();
    int_marker.name = marker.ns + "_" + std::to_string(marker.id);
    int_marker.description = "Move Trajectory Marker";

    // ─── インタラクティブマーカー全体のスケールを大きめに設定 ───
    //   1.0 → 2.0 などに変更すると、ドラッグ可能エリアが拡大される
    int_marker.scale = 2.0;

    // マーカーの初期ポーズをそのままコピー
    int_marker.pose = marker.pose;

    // ─── MOVE_PLANE モードのコントロールを１つ追加 ───
    visualization_msgs::msg::InteractiveMarkerControl control;
    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

    // --- ① 透明Sphereを作って「ヒットボックス」を大きくする ---
    visualization_msgs::msg::Marker hit_box;
    hit_box.header.frame_id = "map";
    hit_box.header.stamp = this->now();
    hit_box.ns = "hit_box";
    hit_box.id = marker.id;          // IDはmarkerと合わせても別でもよい
    hit_box.type = visualization_msgs::msg::Marker::SPHERE;
    // Sphereの直径を1.0mに設定 (arrowよりも大きくする)
    hit_box.scale.x = 1.0;
    hit_box.scale.y = 1.0;
    hit_box.scale.z = 1.0;
    // 完全透明にする（見た目には表示されないが、RViz上で拾われる）
    hit_box.color.r = 0.0f;
    hit_box.color.g = 0.0f;
    hit_box.color.b = 0.0f;
    hit_box.color.a = 0.0f;
    hit_box.pose = marker.pose;      // arrowと同じ位置に置く

    // 透明Sphereを control に追加
    control.markers.push_back(hit_box);

    // --- ② 実際に見える矢印マーカーを追加 ---
    visualization_msgs::msg::Marker visible_arrow = marker;
    control.markers.push_back(visible_arrow);

    // 常に表示する（アルウェイズ ヴィジブル）
    control.always_visible = true;
    int_marker.controls.push_back(control);

    // コールバック登録
    server_->insert(
      int_marker,
      std::bind(&EditorToolServer::processFeedback, this, std::placeholders::_1));
    server_->setCallback(
      int_marker.name,
      std::bind(&EditorToolServer::alignMarker, this, std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
  }

  bool EditorToolServer::markerPointsToVelocityLine(
    visualization_msgs::msg::MarkerArray & marker_array)
  {
    const int marker_count = static_cast<int>(marker_array.markers.size());
    if (marker_count < 1) {
      return false;
    }

    std::vector<visualization_msgs::msg::Marker> line_markers;
    std::vector<visualization_msgs::msg::Marker> velocity_markers;

    // Connect each pair of consecutive points with a line and add a speed label
    for (int i = 0; i < marker_count - 1; ++i) {
      const auto & marker = marker_array.markers[i];
      const auto & next_marker = marker_array.markers[i + 1];

      geometry_msgs::msg::Point start_point = marker.pose.position;
      geometry_msgs::msg::Point end_point = next_marker.pose.position;

      // Line marker
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = this->now();
      line_marker.ns = "line";
      line_marker.id = i;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.1;
      line_marker.color = marker.color;
      line_marker.color.a = 1.0;
      line_marker.points.push_back(start_point);
      line_marker.points.push_back(end_point);
      line_markers.push_back(line_marker);

      // Velocity text marker
      visualization_msgs::msg::Marker velocity_marker;
      velocity_marker.header = marker.header;
      velocity_marker.ns = "speed_label";
      velocity_marker.id = i;
      velocity_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      velocity_marker.action = visualization_msgs::msg::Marker::ADD;
      velocity_marker.pose.position.x = start_point.x + 0.02;
      velocity_marker.pose.position.y = start_point.y + 0.02;
      velocity_marker.pose.position.z = start_point.z + 0.1;
      velocity_marker.pose.orientation.w = 1.0;
      velocity_marker.scale.z = 1.0;
      velocity_marker.color = marker.color;
      velocity_marker.color.a = 1.0;
      velocity_marker.text = marker.text;
      velocity_markers.push_back(velocity_marker);
    }

    // Connect last point back to first point
    const auto & last_marker = marker_array.markers[marker_count - 1];
    const auto & first_marker = marker_array.markers[0];
    geometry_msgs::msg::Point last_start_point = last_marker.pose.position;
    geometry_msgs::msg::Point last_end_point = first_marker.pose.position;

    visualization_msgs::msg::Marker last_line_marker;
    last_line_marker.header.frame_id = "map";
    last_line_marker.header.stamp = this->now();
    last_line_marker.ns = "line";
    last_line_marker.id = marker_count - 1;
    last_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    last_line_marker.action = visualization_msgs::msg::Marker::ADD;
    last_line_marker.pose.orientation.w = 1.0;
    last_line_marker.scale.x = 0.1;
    last_line_marker.color = last_marker.color;
    last_line_marker.color.a = 1.0;
    last_line_marker.points.push_back(last_start_point);
    last_line_marker.points.push_back(last_end_point);
    line_markers.push_back(last_line_marker);

    // Append line and velocity markers to the original array
    marker_array.markers.insert(
      marker_array.markers.end(),
      line_markers.begin(),
      line_markers.end());
    marker_array.markers.insert(
      marker_array.markers.end(),
      velocity_markers.begin(),
      velocity_markers.end());

    return true;
  }

  void EditorToolServer::publishMarkers()
  {
    // Combine the base trajectory markers into a MarkerArray
    visualization_msgs::msg::MarkerArray publish_array;
    publish_array.markers = trajectory_markers_;

    // Append line and speed-label markers
    markerPointsToVelocityLine(publish_array);

    // Publish to the /race_trajectory topic
    marker_pub_->publish(publish_array);
  }

  bool EditorToolServer::parseLineToMarker(
    const std::string & line,
    int id,
    visualization_msgs::msg::Marker & marker)
  {
    // Tokenize by comma
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> elems;
    while (std::getline(ss, token, ',')) {
      elems.push_back(token);
    }
    // Expect exactly 8 elements: x, y, z, x_quat, y_quat, z_quat, w_quat, speed
    if (elems.size() != 8) {
      return false;
    }
    // Parse position
    double x = std::stod(elems[0]);
    double y = std::stod(elems[1]);
    double z = std::stod(elems[2]);
    // Parse quaternion
    double qx = std::stod(elems[3]);
    double qy = std::stod(elems[4]);
    double qz = std::stod(elems[5]);
    double qw = std::stod(elems[6]);
    // Parse speed (m/s → km/h)
    double speed = std::stod(elems[7]) * 3.6;

    // Setup base Marker
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "arrow";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Display speed (one decimal place)
    {
      std::ostringstream text_ss;
      text_ss << std::fixed << std::setprecision(1) << speed;
      marker.text = text_ss.str();
    }

    // Set pose
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    // Arrow scale: length, width, height (constant)
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Color by speed: <10 green, <20 yellow, else red
    if (speed < 10.0) {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else if (speed < 20.0) {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;

    return true;
  }

  bool EditorToolServer::saveCsv(
    const std::string & file_name,
    const visualization_msgs::msg::MarkerArray & marker_array)
  {
    std::ofstream ofs(file_name);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_name.c_str());
      return false;
    }
    // Header: x,y,z,qx,qy,qz,qw,speed
    ofs << "x,y,z,qx,qy,qz,qw,speed\n";
    for (const auto & marker : marker_array.markers) {
      // marker.text contains speed in km/h; convert back to m/s
      double speed_kmh = std::stod(marker.text);
      double speed_ms = speed_kmh / 3.6;
      ofs << marker.pose.position.x << ","
          << marker.pose.position.y << ","
          << marker.pose.position.z << ","
          << marker.pose.orientation.x << ","
          << marker.pose.orientation.y << ","
          << marker.pose.orientation.z << ","
          << marker.pose.orientation.w << ","
          << speed_ms << "\n";
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
