#include "../include/osmVisualizer/osmVisualizer.hpp"

OsmVisualizer::OsmVisualizer() : Node("OsmVisualizer")
{
  this->declare_parameter("map_path", "/home/atakan/Downloads/Town10.osm");
  this->declare_parameter("enable_inc_path_points", true);
  this->declare_parameter("interval", 2.0);
  if (!readParameters())
      rclcpp::shutdown();
  
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hd_map", 10);
  array_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/array", 10);
  timer_ = this->create_wall_timer( 500ms, std::bind(&OsmVisualizer::timer_callback, this));

  lanelet::LaneletMapPtr map = lanelet::load(map_path_, lanelet::projection::UtmProjector(lanelet::Origin({15, 30})));
  for (auto point : map->pointLayer)
  {
      point.x() = point.attribute("local_x").asDouble().value();
      point.y() = point.attribute("local_y").asDouble().value();
  }

  fill_marker(map);
  fill_array_with_left_right(map);
  // writeToFile(m_array);
}

bool OsmVisualizer::readParameters()
{
    if (!this->get_parameter("map_path", map_path_))
    {
        std::cout << "Failed to read parameter 'map_path' " << std::endl;
        return false;
    }
    if (!this->get_parameter("enable_inc_path_points", enable_inc_path_points_))
    {
        std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
        return false;
    }
    if (!this->get_parameter("interval", interval_))
    {
        std::cout << "Failed to read parameter 'interval' to increase the path points" << std::endl;
        return false;
    }
    return true;
}

void OsmVisualizer::timer_callback()
{
  // publisher_->get_subscription_count() > 0 && m_marker_array.markers.size() > 0
  // if( publisher_->get_subscription_count() > 0)
    {
      publisher_->publish(m_marker_array);
      array_publisher_->publish(m_array);
    }
}

void OsmVisualizer::fill_array(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = 100000;
  m_array.layout.dim[0].stride = 100000*2;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 2;
  m_array.layout.dim[1].stride = 2;


  for (const auto &ll : t_map->laneletLayer)
  {
    for(size_t i = 0; i<ll.centerline2d().size()-1; i++)
    {
      if(getDistance(ll,i) > 2 && enable_inc_path_points_)
      {
        double dist = getDistance(ll,i);
        double interval = 1;
        int num_points = dist / interval;

        for(int k = 0 ; k<num_points;k++)
        {
          m_array.data.push_back(((ll.centerline2d()[i+1].x()-ll.centerline2d()[i].x()) / num_points) * k + ll.centerline2d()[i].x());
          m_array.data.push_back(((ll.centerline2d()[i+1].y()-ll.centerline2d()[i].y()) / num_points) * k + ll.centerline2d()[i].y());
        }
      }
      else
      {
      m_array.data.push_back(ll.centerline2d()[i].x());
      m_array.data.push_back(ll.centerline2d()[i].y());
      }
    }
  }
}


void OsmVisualizer::writeToFile(const std_msgs::msg::Float64MultiArray& multi_array)
{
  std::ofstream file("data.txt");
  if (file.is_open())
  {
    for (size_t i = 0; i < multi_array.data.size(); ++i)
    {
      file << multi_array.data[i] << ",";
      if ((i + 1) % (multi_array.layout.dim[0].size) == 0)
        file << "\n";
      if ((i + 1) % (multi_array.layout.dim[1].size) == 0)
        file << "\n";
    }
    file.close();
  }
}

void OsmVisualizer::fill_array_with_left_right(lanelet::LaneletMapPtr &t_map)
{
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[0].label = "rows";
  m_array.layout.dim[0].size = t_map->laneletLayer.size();
  m_array.layout.dim[0].stride = t_map->laneletLayer.size()*4;
  m_array.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  m_array.layout.dim[1].label = "cols";
  m_array.layout.dim[1].size = 4;
  m_array.layout.dim[1].stride = 4;

  for (const auto &ll : t_map->laneletLayer)
  {
    std::vector<lanelet::ConstLineString3d> bounds;
    bounds.push_back(ll.leftBound());
    bounds.push_back(ll.rightBound());

    size_t size = (bounds[0].size() < bounds[1].size()) ? bounds[0].size() : bounds[1].size();
    for(size_t i = 0; i<size; i++)
    {
      m_array.data.push_back(bounds[0][i].x());
      m_array.data.push_back(bounds[0][i].y());
      m_array.data.push_back(bounds[1][i].x());
      m_array.data.push_back(bounds[1][i].y());
    }
  }
}


double OsmVisualizer::getDistance(const lanelet::ConstLanelet &ll , size_t i) 
{
    return std::sqrt(std::pow(ll.centerline2d()[i].x() - ll.centerline2d()[i+1].x(),2)+std::pow(ll.centerline2d()[i].y()-ll.centerline2d()[i+1].y(),2));
}

void OsmVisualizer::fill_marker(lanelet::LaneletMapPtr &t_map)
{
  size_t i =  0;

  // for lanelets
  std::vector<lanelet::ConstLineString3d> bounds;
  bounds.push_back(ll.leftBound());
  bounds.push_back(ll.rightBound());

  for (const auto &bound : bounds)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock{}.now();
    marker.ns = "lanelet";
    marker.id = i;
    i++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 232;
    marker.color.g = 44;
    marker.color.b = 44;
    for (const auto &point : bound)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0;
        marker.points.push_back(p);
    }
    m_marker_array.markers.push_back(marker);
  }
}
