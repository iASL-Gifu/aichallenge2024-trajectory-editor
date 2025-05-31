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
// #include "editor_tool_srvs/srv/save_csv.hpp"

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
    // --- Service callbacks ---

    /// Loads trajectory data from a CSV file into markers and interactive markers
    void LoadCsv(
      const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request>  request,
      std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response>       response);

    /// Saves the current trajectory markers back to a CSV file
    // void SaveCsvService(
    //   const std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Request>  request,
    //   std::shared_ptr<editor_tool_srvs::srv::SaveCsv::Response>        response);

    // --- Interactive Marker feedback handlers ---

    /// Handles generic feedback events from interactive markers
    void processFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

    /// Snaps the marker to a grid, updates the stored marker pose, and republishes trajectory
    void alignMarker(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

    // --- Marker creation, updating, and publishing ---

    /// Creates an interactive marker from a base visualization marker
    void makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker);

    /// Given a MarkerArray of trajectory points, appends line and speed-label markers
    bool markerPointsToVelocityLine(visualization_msgs::msg::MarkerArray & marker_array);

    /// Publishes the current trajectory markers (including lines and speed labels) to the topic
    void publishMarkers();

    // --- CSV ↔ MarkerArray conversion helpers ---

    /// Parses a single CSV line into a visualization marker with pose and speed info
    bool parseLineToMarker(
      const std::string & line,
      int id,
      visualization_msgs::msg::Marker & marker);

    /// Writes a MarkerArray of trajectory points back into a CSV file
    bool saveCsv(
      const std::string & file_name,
      const visualization_msgs::msg::MarkerArray & marker_array);

    // --- Member variables ---

    /// Manages interactive markers and their callbacks
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;

    /// Service server for loading CSV files (CSV → markers)
    rclcpp::Service<editor_tool_srvs::srv::LoadCsv>::SharedPtr load_csv_service_;

    /// Service server for saving CSV files (markers → CSV)
    // rclcpp::Service<editor_tool_srvs::srv::SaveCsv>::SharedPtr save_csv_service_;

    /// Publisher for MarkerArray containing trajectory points + lines + speed labels
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    /// Holds the base trajectory markers in the order they were loaded from CSV
    std::vector<visualization_msgs::msg::Marker> trajectory_markers_;

    /// Maps an interactive-marker name (ns_id) to its index in trajectory_markers_
    std::unordered_map<std::string, int> name_to_index_;
  };
} // namespace editor_tool_server
