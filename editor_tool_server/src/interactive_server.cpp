// File: interactive_server.cpp
#include "editor_tool_server/interactive_server.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>  // std::atan2, std::round

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace editor_tool_server
{
  EditorToolServer::EditorToolServer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("editor_tool_server", options)
  {
    // --- InteractiveMarkerServer の初期化 ---
    server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      "editor_tool_server",
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_topics_interface(),
      this->get_node_services_interface());

    // --- CSV 読み込みサービスの登録 ---
    load_csv_service_ = this->create_service<editor_tool_srvs::srv::LoadCsv>(
      "load_csv",
      std::bind(&EditorToolServer::LoadCsv, this,
                std::placeholders::_1, std::placeholders::_2));

    // --- 選択モード開始サービスの登録 (SelectRange.srv) ---
    select_range_service_ = this->create_service<editor_tool_srvs::srv::SelectRange>(
      "select_range",
      std::bind(&EditorToolServer::StartSelection, this,
                std::placeholders::_1, std::placeholders::_2));

    // --- MarkerArray パブリッシャの初期化 ---
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "race_trajectory", 10);

    // 初期状態は選択モードオフ、インデックス -1
    selection_mode_ = false;
    sel_idx1_ = sel_idx2_ = -1;
    selection_velocity_ = 0.0;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // CSV を読み込んで trajectory_markers_ を初期化し、
  // インタラクティブマーカーを生成 ⇒ 最後に publishMarkers() でトピックに流す
  //
  //////////////////////////////////////////////////////////////////////////////
  void EditorToolServer::LoadCsv(
    const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response> /*response*/)
  {
    const std::string & fileName = request->filename;
    if (fileName.empty()) {
      RCLCPP_ERROR(get_logger(), "LoadCsv: filename is empty");
      return;
    }
    RCLCPP_INFO(get_logger(), "Loading CSV file: %s", fileName.c_str());

    std::ifstream ifs(fileName);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", fileName.c_str());
      return;
    }

    std::string line;
    // ヘッダ行をスキップ
    if (!std::getline(ifs, line)) {
      RCLCPP_ERROR(get_logger(), "CSV is empty");
      return;
    }

    // 既存マーカーとマッピングをクリア
    trajectory_markers_.clear();
    name_to_index_.clear();
    server_->clear();  // 既存のインタラクティブマーカーをすべて削除

    int id = 0;
    bool any_error = false;

    // CSV の各行を parseLineToMarker で Marker に変換し、trajectory_markers_ に追加
    while (std::getline(ifs, line)) {
      visualization_msgs::msg::Marker marker;
      if (parseLineToMarker(line, id, marker)) {
        trajectory_markers_.push_back(marker);

        // インタラクティブマーカーを生成
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
      return;
    }

    // 生成したインタラクティブマーカーを RViz に反映
    server_->applyChanges();

    // 初期状態の MarkerArray（ポイント＋ライン＋速度ラベル）を publish
    publishMarkers();

    if (any_error) {
      RCLCPP_WARN(get_logger(), "Some lines failed to parse (check log)");
    } else {
      RCLCPP_INFO(get_logger(), "CSV loaded and markers displayed successfully");
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // 選択モードを開始するサービスコールバック
  //  - request->velocity: ２点選択後に間のマーカーに反映する速度値
  //  - レスポンスの success は省略せず true/false を返す
  //
  //////////////////////////////////////////////////////////////////////////////
  void EditorToolServer::StartSelection(
    const std::shared_ptr<editor_tool_srvs::srv::SelectRange::Request> request,
    std::shared_ptr<editor_tool_srvs::srv::SelectRange::Response> response)
  {
    // 既に選択モード中なら弾く
    if (selection_mode_) {
      RCLCPP_WARN(get_logger(), "SelectRange: already in selection mode");
      response->success = false;
      return;
    }

    // 速度を格納し、選択モードを ON にする
    selection_velocity_ = request->velocity;
    sel_idx1_ = sel_idx2_ = -1;
    selection_mode_ = true;

    RCLCPP_INFO(
      get_logger(),
      "SelectRange: selection mode started. click two markers to select range. velocity=%.2f",
      selection_velocity_);
    response->success = true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // インタラクティブマーカーへのフィードバックを処理
  //  - 通常はサーバに反映するロギングのみ
  //  - ただし selection_mode_ が true のとき、BUTTON_CLICK で２点を選択
  //    ⇒ ２つ目が選ばれた段階で青くハイライトして速度を反映
  //
  //////////////////////////////////////////////////////////////////////////////
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
      // ── BUTTON_CLICK のケースは残しつつ、選択処理は MOUSE_UP へ移行 ──
      case visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK:
        oss << ": button click" << mouse_point_ss.str() << ".";
        RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
        // ※ ここではもう選択処理しない
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

        // ── ここで選択モード中なら“クリック（解放）”として扱う ──
        if (selection_mode_) {
          auto it = name_to_index_.find(feedback->marker_name);
          if (it != name_to_index_.end()) {
            int clicked_idx = it->second;

            if (sel_idx1_ < 0) {
              sel_idx1_ = clicked_idx;
              RCLCPP_INFO(get_logger(), "Selected first index: %d", sel_idx1_);
            }
            else if (sel_idx2_ < 0 && clicked_idx != sel_idx1_) {
              sel_idx2_ = clicked_idx;
              RCLCPP_INFO(get_logger(), "Selected second index: %d", sel_idx2_);

              // ── ２点選択完了：範囲ハイライト＆速度反映ロジック ──
              const int N = static_cast<int>(trajectory_markers_.size());
              int i1 = sel_idx1_, i2 = sel_idx2_;

              // ループ状に短い方を優先して範囲を塗り替える
              int forward_len = (i2 >= i1) ? (i2 - i1) : (N - (i1 - i2));
              int backward_len = (i1 >= i2) ? (i1 - i2) : (N - (i2 - i1));
              bool use_forward = (forward_len <= backward_len);

              if (use_forward) {
                int idx = i1;
                while (true) {
                    // 速度ごとに色を変える
                    if (selection_velocity_ < 10.0) {
                      trajectory_markers_[idx].color.r = 1.0f;
                      trajectory_markers_[idx].color.g = 0.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    } else if (selection_velocity_ < 20.0) {
                      trajectory_markers_[idx].color.r = 1.0f;
                      trajectory_markers_[idx].color.g = 1.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    } else {
                      trajectory_markers_[idx].color.r = 0.0f;
                      trajectory_markers_[idx].color.g = 1.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    }
                  {
                    std::ostringstream ss;
                    ss << std::fixed << std::setprecision(1) << selection_velocity_;
                    trajectory_markers_[idx].text = ss.str();
                  }
                  if (idx == i2) break;
                  idx = (idx + 1) % N;
                }
              }
              else {
                int idx = i1;
                while (true) {
                    // 速度ごとに色を変える
                    if (selection_velocity_ < 10.0) {
                      trajectory_markers_[idx].color.r = 1.0f;
                      trajectory_markers_[idx].color.g = 0.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    } else if (selection_velocity_ < 20.0) {
                      trajectory_markers_[idx].color.r = 1.0f;
                      trajectory_markers_[idx].color.g = 1.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    } else {
                      trajectory_markers_[idx].color.r = 0.0f;
                      trajectory_markers_[idx].color.g = 1.0f;
                      trajectory_markers_[idx].color.b = 0.0f;
                    }
                  {
                    std::ostringstream ss;
                    ss << std::fixed << std::setprecision(1) << selection_velocity_;
                    trajectory_markers_[idx].text = ss.str();
                  }
                  if (idx == i2) break;
                  idx = (idx - 1 + N) % N;
                }
              }

              // 結果を publish して、選択モードを終了
              publishMarkers();
              selection_mode_ = false;
              sel_idx1_ = sel_idx2_ = -1;
              RCLCPP_INFO(get_logger(), "Selection completed; exiting selection mode.");
            }
          }
        }
        break;
    }

    // インタラクティブマーカーの変更（POSE_UPDATE 等）はすべて applyChanges() で反映
    server_->applyChanges();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // alignMarker の修正：
  //  - グリッドスナップ後、古い位置から f=0.1 倍だけ移動
  //  - さらに「次のマーカーの方向を向く」ようにオリエンテーションを再計算
  //
  //////////////////////////////////////////////////////////////////////////////
  void EditorToolServer::alignMarker(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
  {
    // 1) もとの位置を取得
    auto it = name_to_index_.find(feedback->marker_name);
    if (it == name_to_index_.end()) {
      return;  // マーカーが見つからなかったら何もしない
    }
    int idx = it->second;

    // 2) マウスで得られた新しい Pose をグリッド(0.01間隔)にスナップ
    geometry_msgs::msg::Pose raw = feedback->pose;
    raw.position.x = std::round(raw.position.x * 100.0) / 100.0;
    raw.position.y = std::round(raw.position.y * 100.0) / 100.0;
    // raw.position.z = std::round(raw.position.z * 100.0) / 100.0; // 必要なら Z も

    // 3) インタラクティブマーカー側にも更新を反映（見た目だけ先に動かす）
    server_->setPose(feedback->marker_name, raw);
    server_->applyChanges();

    // 4) trajectory_markers_ の古い Pose を取得し、感度 f で新しい Pose を計算
    geometry_msgs::msg::Pose old_pose = trajectory_markers_[idx].pose;
    geometry_msgs::msg::Pose target_pose = raw;
    const double f = 0.1;  // 10% ずつジャンプする

    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = old_pose.position.x + f * (target_pose.position.x - old_pose.position.x);
    new_pose.position.y = old_pose.position.y + f * (target_pose.position.y - old_pose.position.y);
    new_pose.position.z = old_pose.position.z + f * (target_pose.position.z - old_pose.position.z);

    // 5) 「次のマーカーの方向を向く」オリエンテーションを計算
    {
      const int N = static_cast<int>(trajectory_markers_.size());
      int next_idx = (idx + 1) % N;
      const auto & next_marker = trajectory_markers_[next_idx];

      double dx = next_marker.pose.position.x - new_pose.position.x;
      double dy = next_marker.pose.position.y - new_pose.position.y;
      // Z 軸は無視して XY 平面上の角度を計算
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      new_pose.orientation = tf2::toMsg(q);
    }

    // 6) trajectory_markers_ に新しい Pose をセット
    trajectory_markers_[idx].pose = new_pose;
    trajectory_markers_[idx].header.stamp = this->now();

    // 7) MarkerArray 全体を再生成して publish
    publishMarkers();
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // インタラクティブマーカーを作成する際、クリック可能領域（ヒットボックス）を拡大する実装
  //
  //////////////////////////////////////////////////////////////////////////////
  void EditorToolServer::makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = this->now();
    int_marker.name = marker.ns + "_" + std::to_string(marker.id);
    int_marker.description = "Move Trajectory Marker";

    // ── インタラクティブマーカー全体のスケールを大きめに設定（クリック可能領域を拡張） ──
    int_marker.scale = 2.0;  // 1.0 → 2.0 にすると、かなり広めにクリックできる

    // マーカーの初期ポーズをそのままコピー
    int_marker.pose = marker.pose;

    // ── MOVE_PLANE モードのコントロールを追加 ──
    visualization_msgs::msg::InteractiveMarkerControl control;
    tf2::Quaternion orien(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    control.orientation = tf2::toMsg(orien);
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

    // --- ① 透明 Sphere を作って、クリック領域をさらに大きくする ---
    visualization_msgs::msg::Marker hit_box;
    hit_box.header.frame_id = "map";
    hit_box.header.stamp = this->now();
    hit_box.ns = "hit_box";
    hit_box.id = marker.id;  // ID は同じでも別でもよい
    hit_box.type = visualization_msgs::msg::Marker::SPHERE;
    // Sphere の直径を 1.0m に設定（arrow より大きくする）
    hit_box.scale.x = 1.0;
    hit_box.scale.y = 1.0;
    hit_box.scale.z = 1.0;
    // 完全に透明にする（見た目は見えないが RViz 上で拾われる）
    hit_box.color.r = 0.0f;
    hit_box.color.g = 0.0f;
    hit_box.color.b = 0.0f;
    hit_box.color.a = 0.0f;
    hit_box.pose = marker.pose;  // arrow と同じ位置に置く

    // 透明 Sphere を control に追加
    control.markers.push_back(hit_box);

    // --- ② 実際に見える矢印マーカーを追加 ---
    visualization_msgs::msg::Marker visible_arrow = marker;
    control.markers.push_back(visible_arrow);

    control.always_visible = true;
    int_marker.controls.push_back(control);

    // コールバックを登録
    server_->insert(
      int_marker,
      std::bind(&EditorToolServer::processFeedback, this, std::placeholders::_1));
    server_->setCallback(
      int_marker.name,
      std::bind(&EditorToolServer::alignMarker, this, std::placeholders::_1),
      visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // MarkerArray の中のポイント列を「ライン」と「速度ラベル」に変換して末尾に追加
  //
  //////////////////////////////////////////////////////////////////////////////
  bool EditorToolServer::markerPointsToVelocityLine(
    visualization_msgs::msg::MarkerArray & marker_array)
  {
    const int marker_count = static_cast<int>(marker_array.markers.size());
    if (marker_count < 1) {
      return false;
    }

    std::vector<visualization_msgs::msg::Marker> line_markers;
    std::vector<visualization_msgs::msg::Marker> velocity_markers;

    // 各連続ペアをつなぎ、ラインマーカーと速度テキストマーカーを作成
    for (int i = 0; i < marker_count - 1; ++i) {
      const auto & marker = marker_array.markers[i];
      const auto & next_marker = marker_array.markers[i + 1];

      geometry_msgs::msg::Point start_point = marker.pose.position;
      geometry_msgs::msg::Point end_point   = next_marker.pose.position;

      // ── ラインマーカー ──
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = this->now();
      line_marker.ns = "line";
      line_marker.id = i;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      line_marker.pose.orientation.w = 1.0;
      line_marker.scale.x = 0.1;  // 線の太さ
      line_marker.color = marker.color;
      line_marker.color.a = 1.0;
      line_marker.points.push_back(start_point);
      line_marker.points.push_back(end_point);
      line_markers.push_back(line_marker);

      // ── 速度テキストマーカー ──
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
      velocity_marker.scale.z = 1.0;  // テキストの大きさ
      velocity_marker.color = marker.color;
      velocity_marker.color.a = 1.0;
      velocity_marker.text = marker.text;
      velocity_markers.push_back(velocity_marker);
    }

    // 最後のマーカー → 最初のマーカーをつなぐ
    const auto & last_marker  = marker_array.markers[marker_count - 1];
    const auto & first_marker = marker_array.markers[0];
    geometry_msgs::msg::Point last_start_point = last_marker.pose.position;
    geometry_msgs::msg::Point last_end_point   = first_marker.pose.position;

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

    // velocity_markers には最終マーカーのラベルは不要なので追加しない

    // オリジナル配列に「ライン」と「速度テキスト」を結合
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

  //////////////////////////////////////////////////////////////////////////////
  //
  // MarkerArray 全体を publish する。基礎の trajectory_markers_ → ライン＋速度ラベルを付加 → publish
  //
  //////////////////////////////////////////////////////////////////////////////
  void EditorToolServer::publishMarkers()
  {
    visualization_msgs::msg::MarkerArray publish_array;
    publish_array.markers = trajectory_markers_;  // 基礎マーカー

    // ライン＋速度ラベルを追加
    markerPointsToVelocityLine(publish_array);

    // /race_trajectory トピックにパブリッシュ
    marker_pub_->publish(publish_array);
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // 1 行の CSV をパースして Marker を構築
  //   フォーマット: x,y,z,qx,qy,qz,qw,speed (m/s)
  //   内部では speed → km/h に変換して text に入れる
  //
  //////////////////////////////////////////////////////////////////////////////
  bool EditorToolServer::parseLineToMarker(
    const std::string & line,
    int id,
    visualization_msgs::msg::Marker & marker)
  {
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> elems;
    while (std::getline(ss, token, ',')) {
      elems.push_back(token);
    }
    if (elems.size() != 8) {
      return false;
    }
    // 座標
    double x  = std::stod(elems[0]);
    double y  = std::stod(elems[1]);
    double z  = std::stod(elems[2]);
    // 四元数
    double qx = std::stod(elems[3]);
    double qy = std::stod(elems[4]);
    double qz = std::stod(elems[5]);
    double qw = std::stod(elems[6]);
    // speed (m/s → km/h)
    double speed = std::stod(elems[7]) * 3.6;

    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "arrow";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // テキストに速度 (小数1桁) を入れる
    {
      std::ostringstream text_ss;
      text_ss << std::fixed << std::setprecision(1) << speed;
      marker.text = text_ss.str();
    }

    // ポーズ設定
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    // 矢印のスケール (長さ, 幅, 太さ)
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // 速度に応じて色分け: <10→赤, <20→黄, それ以上→緑
    if (speed < 10.0) {
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
    } else if (speed < 20.0) {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    } else {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    marker.color.a = 1.0f;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //
  // MarkerArray を CSV に保存する (基礎マーカーのみ)
  //   text に入っている km/h を m/s に戻して書き込む
  //
  //////////////////////////////////////////////////////////////////////////////
  bool EditorToolServer::saveCsv(
    const std::string & file_name,
    const visualization_msgs::msg::MarkerArray & marker_array)
  {
    std::ofstream ofs(file_name);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", file_name.c_str());
      return false;
    }
    // ヘッダ
    ofs << "x,y,z,qx,qy,qz,qw,speed\n";

    for (const auto & marker : marker_array.markers) {
      double speed_kmh = std::stod(marker.text);
      double speed_ms  = speed_kmh / 3.6;
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
