// File: interactive_server.hpp
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "editor_tool_srvs/srv/load_csv.hpp"
#include "editor_tool_srvs/srv/select_range.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

namespace editor_tool_server
{
  class EditorToolServer : public rclcpp::Node
  {
  public:
    /// Constructor: initializes the node, service servers, and publisher
    explicit EditorToolServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~EditorToolServer() = default;

    /// Applies pending changes on the InteractiveMarkerServer
    inline void applyChanges()
    {
      server_->applyChanges();
    }

  private:
    // --- サービスコールバック ---

    /// CSVファイルから軌跡データを読み込み、インタラクティブマーカーと内部データを初期化
    void LoadCsv(
      const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request>  request,
      std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response>       response);
    
    void LoadCsvFile(const std::string & file_name);

    /// 選択モードを開始し、２点を選択した後、その間にあるマーカーを青くして速度を反映する
    void StartSelection(
      const std::shared_ptr<editor_tool_srvs::srv::SelectRange::Request>  request,
      std::shared_ptr<editor_tool_srvs::srv::SelectRange::Response>        response);

    void StartParallelMove(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void ConfirmParallelMove(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void undo(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void redo(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void publishTrajectorySrv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void publishTrajectory();
    
    void redrawMarkers();
    void createMoveHelperMarker();
    void saveStateForUndo();

    // --- インタラクティブマーカーフィードバックハンドラ ---

    /// インタラクティブマーカーへの一般的なフィードバックを処理
    void processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

    /// マーカーをグリッドにスナップし、“感度”を考慮して緩やかに動かし、かつ「次のマーカーの方向を向く」ようオリエンテーションを再計算
    void alignMarker(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

    // --- マーカー生成・更新・公開 ---

    /// 与えられた基礎マーカーからインタラクティブマーカーを作成（クリック可能領域を拡大）
    void makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker);

    /// MarkerArray の中の連続した点列をラインと速度ラベルに変換して追加
    bool markerPointsToVelocityLine(visualization_msgs::msg::MarkerArray & marker_array);

    /// trajectory_markers_（基礎マーカー群）を元に改めて MarkerArray を生成して publish
    void publishMarkers();

    // --- CSV ↔ MarkerArray 変換ヘルパー ---

    /// 1行の CSV テキストを Marker に変換
    bool parseLineToMarker(
      const std::string & line,
      int id,
      visualization_msgs::msg::Marker & marker);

    /// MarkerArray のデータを CSV ファイルに出力
    bool saveCsv(
      const std::string & file_name,
      const visualization_msgs::msg::MarkerArray & marker_array);

    std::vector<int> getRangeIndices(int idx1, int idx2);

    // --- Member variables ---

    /// インタラクティブマーカーを管理するサーバ
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;

    /// Service
    rclcpp::Service<editor_tool_srvs::srv::LoadCsv>::SharedPtr load_csv_service_;
    rclcpp::Service<editor_tool_srvs::srv::SelectRange>::SharedPtr select_range_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_parallel_move_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr confirm_parallel_move_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr undo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr redo_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_trajectory_service_;

    /// Node param
    std::string csv_file_path_;
    bool publish_on_initialize_;
    float wait_seconds_;
    bool publish_on_change_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> & parameters);


    /// MarkerArray をまとめて publish するパブリッシャ
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

    /// 基礎となる軌跡マーカーを保持。CSV から読み込んだ順番どおりに格納
    std::vector<visualization_msgs::msg::Marker> trajectory_markers_;

    /// 「インタラクティブマーカー名(ns_id)」から trajectory_markers_ のインデックスを引けるマップ
    std::unordered_map<std::string, int> name_to_index_;

    /// 選択モード（速度変更用）
    bool selection_mode_{false};
    int sel_idx1_{-1};
    int sel_idx2_{-1};
    double selection_velocity_{0.0};

    // // 並列モード
    bool parallel_mode_{false};
    int p_sel_idx1_{-1};
    int p_sel_idx2_{-1};
    std::string move_marker_name_{"parallel_picker"};
    geometry_msgs::msg::Point move_marker_prev_pos_;

    // Undo/Redo バッファ（必要な場合）
    std::vector<std::vector<visualization_msgs::msg::Marker>> undo_stack_;
    std::vector<std::vector<visualization_msgs::msg::Marker>> redo_stack_;
  };
} // namespace editor_tool_server
