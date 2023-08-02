<p align="center">
  <a href="#dart-about">About</a> &#xa0; | &#xa0; 
  <a href="#sparkles-features">Features</a> &#xa0; | &#xa0;
  <a href="#white_check_mark-requirements">Requirements</a> &#xa0; | &#xa0;
  <a href="#checkered_flag-starting">Starting</a> &#xa0; | &#xa0;
  <a href="#memo-license">License</a> &#xa0; | &#xa0;
  <a href="https://github.com/{{github}}" target="_blank">Author</a>
</p>

# map_visualizer package

## :dart: About ##
This package provides the visualize the map in ROS2 using MarkerArray and Occupancygrid map types.

## :rocket: Technologies ##

The following tools were used in this project:

- [ROS](https://www.ros.org/) Tested on ROS2 Foxy distro.
- [C++](https://cppreference.com)

## :white_check_mark: Requirements ##
Before starting :checkered_flag:, you need to have ROS2(Foxy) and install some ros2 packages.

`sudo apt-get install ros-<distro>-lanelet2`
E.g: `sudo apt-get install ros-foxy-lanelet2`

## osm_visualizer node

### :sparkles: Features

osm_visualizer node visualizes the .osm(OpenStreetMap) file into RViz MarkerArray. 

### :checkered_flag: How to Run

`ros2 run map_visualizer osm_visualizer --ros-args -p map_path:=path/to/map.osm`
or
`ros2 launch map_visualizer osm_visualizer.launch.xml`
or
`ros2 launch map_visualizer osm_visualizer.launch.py`

### Subscribed Topics

- None

### Published Topics

- /hd_map (visualization_msgs/msg/marker_array) : visualization messages for RViz
- /array (std_msgs/msg/float64_multi_array) : the left and right bounds points of the map lanelet layers for occupancy_pub node

### Parameters

| Name                   | Type        | Description                                                                                         | Default value           |
| :--------------------- | :---------- | :-------------------------------------------------------------------------------------------------  | :---------------------- |
| map_path               | std::string | lanelet2 map path                                                                                   | change on launch file   |
| enable_inc_path_points | bool        | flag to increment points of left and rigth boundry's linestrings                                    | true                    |
| interval               | std::string | interval to increment left and right boundry points with linear interpolation for occupancygrid map | 0.5                     |


---

## occupancy_pub node

### :sparkles: Features

occupancy_pub node turn the osm data provided by osm_visualizer node into nav_msgs/msg/occupancy_grid. 

### :checkered_flag: How to Run

`ros2 run map_visualizer occupancy_pub`

### Subscribed Topics

- /array (std_msgs/msg/float64_multi_array) : the left and right bounds points of the map lanelet layers
- /initialpose (geometry_msgs/msg/pose_with_covariance_stamped) : RViz initial pose message to calculate initial location on grid
- /goal_pose (geometry_msgs/msg/pose_stamped) : RViz goal pose message to calculate goal location on grid

### Published Topics

- /occupancy_grid (nav_msgs/msg/occupancy_grid) : occupancy grid messages for RViz
- /obsx (std_msgs/msg/float64_multi_array) : x index of obstacles on grid for A* package
- /obsy (std_msgs/msg/float64_multi_array) : y index of obstacles on grid for A* package
- /gridsize (std_msgs/msg/float64_multi_array) : gridsize information message for A* package

### Parameters

- None


## :memo: License

This project is under license from MIT. For more details, see the [LICENSE](LICENSE) file.


Made with by <a href="https://github.com/mucahitayhan" target="_blank">Mücahit Ayhan</a>

&#xa0;

<a href="#top">Back to top</a>