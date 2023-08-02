
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// lanelet libraries
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace std::chrono_literals;

class OsmVisualizer : public rclcpp::Node
{
  public:
    OsmVisualizer();

  private:
    void timer_callback();
    bool readParameters();
    void writeToFile(const std_msgs::msg::Float64MultiArray& multi_array);
    void fill_marker(lanelet::LaneletMapPtr &t_map);
    void fill_array(lanelet::LaneletMapPtr &t_map);
    void fill_array_with_left_right(lanelet::LaneletMapPtr &t_map);
    void fill_min_max_values(const lanelet::Lanelet &li);
    double getDistance(const lanelet::ConstLanelet &ll , size_t i);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_publisher_;

    std_msgs::msg::Float64MultiArray m_array;
    visualization_msgs::msg::MarkerArray m_marker_array;

    //params
    std::string map_path_; 
    bool enable_inc_path_points_;
    double interval_;
};