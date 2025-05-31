#include <chrono>
#include <memory>
#include <string>

#include "interactive_markers/interactive_marker_server.hpp"
#include "interactive_markers/menu_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "editor_tool_srvs/srv/load_csv.hpp"

namespace editor_tool_server
{
  class EditorToolServer : public rclcpp::Node
  {
    public:
      explicit EditorToolServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

      ~EditorToolServer() = default;

      inline void applyChanges()
      {
        server_->applyChanges();
      }
      
    
    private:
      void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
      void alignMarker(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);
      void LoadCsv(const std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Request> request, std::shared_ptr<editor_tool_srvs::srv::LoadCsv::Response> response);
      void makeMoveTrajectoryMarker(visualization_msgs::msg::Marker & marker);
      bool markerPointsToVelocityLine(visualization_msgs::msg::MarkerArray & marker_array);
      bool parseLineToMarker(const std::string & line, int id, visualization_msgs::msg::Marker & marker);
      bool saveCsv(const std::string & file_name, const visualization_msgs::msg::MarkerArray & marker_array);
      
      std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
      rclcpp::Service<editor_tool_srvs::srv::LoadCsv>::SharedPtr load_csv_service_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  };
}