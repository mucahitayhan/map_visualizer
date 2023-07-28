#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class OccupancyGridPublisher : public rclcpp::Node {
public:
    OccupancyGridPublisher() : Node("occupancy_grid_publisher") {
        osm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/array", 10, std::bind(&OccupancyGridPublisher::float64MultiArrayCallback, this, std::placeholders::_1));

        std::vector<int8_t> matrix_data = {
            0, 100, 0, 0,
            100, 0, 100, 100,
            0, 0, 0, 100,
            100, 100, 100, 0,
            100,0,0,0
        };

        int width = 4;  // x
        int height = 5; // y

        occupancy_grid_msg.header.frame_id = "map";  
        occupancy_grid_msg.info.width = width;
        occupancy_grid_msg.info.height = height;
        occupancy_grid_msg.info.resolution = 1.0; 
        occupancy_grid_msg.info.origin.position.x = 0.0;
        occupancy_grid_msg.info.origin.position.y = 0.0;
        occupancy_grid_msg.info.origin.position.z = 0.0;
        occupancy_grid_msg.data = matrix_data;

        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
    }

private:
    void publishOccupancyGrid() 
    {

        if(first_ && !array_data_x.empty())
        {

            std::vector<int32_t>int_vector_x = std::vector<int32_t>(array_data_x.begin(),array_data_x.end());
            std::vector<int32_t>int_vector_y = std::vector<int32_t>(array_data_y.begin(),array_data_y.end());
            int min_x=*min_element(int_vector_x.begin(), int_vector_x.end());
            int min_y=*min_element(int_vector_y.begin(), int_vector_y.end());
            int max_x=*max_element(int_vector_x.begin(), int_vector_x.end());
            int max_y=*max_element(int_vector_y.begin(), int_vector_y.end());
            occupancy_grid_msg.header.frame_id = "map";  // Doldurmak istediğiniz frame_id'yi belirtin.
            int width = max_x - min_x; // max_x - min_x
            int height = max_y - min_y; // max_y - min_y
            occupancy_grid_msg.info.width = width;
            occupancy_grid_msg.info.height = height;
            occupancy_grid_msg.info.origin.position.x = min_x;
            occupancy_grid_msg.info.origin.position.y = min_y;
            occupancy_grid_msg.info.origin.position.z = 0.0;
            occupancy_grid_msg.info.resolution = 1.0; 
            std::cout << width << std::endl;
            std::cout << height << std::endl;
            occupancy_grid_msg.data = createMatrixWithMod( int_vector_x , int_vector_y , width , height , min_x , min_y);
            first_ = false;
        }
        occupancy_grid_publisher_->publish(occupancy_grid_msg);
    }
    
    void float64MultiArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr t_msg)
    {
        if(first)
        {
            for(int i = 0 ; i < t_msg->data.size()/2 ; i++)
            {
                array_data_x.push_back(round(t_msg->data[2*i]));
                array_data_y.push_back(round(t_msg->data[2*i+1]));
            }
            first = false;
        }
    }

    std::vector<int8_t> createMatrixWithMod(std::vector<int32_t>& int_vector_x, std::vector<int32_t>& int_vector_y, int& rows, int& columns, int& min_x, int& min_y) 
    {
        std::cout << int_vector_y.size()<< std::endl;
        std::vector<int8_t> matrix_data(rows * columns, 0); // Initialize the matrix with zeros
        for (size_t i = 0; i < int_vector_x.size(); ++i) 
        {
            int32_t x = int_vector_x[i] - min_x;
            int32_t y = int_vector_y[i] - min_y;

            if (x >= 0 && x < rows && y >= 0 && y < columns) {
                matrix_data[(y * rows) + x] = 100;
            }
        }
        return matrix_data;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr osm_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    std::vector<double> array_data_x;
    std::vector<double> array_data_y;
    bool first_{true};
    bool first{true};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}
    