#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class OccupancyGridPublisher : public rclcpp::Node {
public:
    OccupancyGridPublisher() : Node("occupancy_grid_publisher") {
        osm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/array", 10, std::bind(&OccupancyGridPublisher::float64MultiArrayCallback, this, std::placeholders::_1));
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid",10);
        obsx_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsx",10);
        obsy_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("obsy",10);
        grid_size_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("gridsize",10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&OccupancyGridPublisher::publishOccupancyGrid, this));
    }

private:
    void publishOccupancyGrid() 
    {
        if(first_ && !array_data_right_y.empty())
        {

            std::vector<int32_t>int_vector_left_x = std::vector<int32_t>(array_data_left_x.begin(),array_data_left_x.end());
            std::vector<int32_t>int_vector_left_y = std::vector<int32_t>(array_data_left_y.begin(),array_data_left_y.end());
            std::vector<int32_t>int_vector_right_x = std::vector<int32_t>(array_data_right_x.begin(),array_data_right_x.end());
            std::vector<int32_t>int_vector_right_y = std::vector<int32_t>(array_data_right_y.begin(),array_data_right_y.end());

            int min_left_x=*min_element(int_vector_left_x.begin(), int_vector_left_x.end());
            int min_left_y=*min_element(int_vector_left_y.begin(), int_vector_left_y.end());
            int max_left_x=*max_element(int_vector_left_x.begin(), int_vector_left_x.end());
            int max_left_y=*max_element(int_vector_left_y.begin(), int_vector_left_y.end());
            int min_right_x=*min_element(int_vector_right_x.begin(), int_vector_right_x.end());
            int min_right_y=*min_element(int_vector_right_y.begin(), int_vector_right_y.end());
            int max_right_x=*max_element(int_vector_right_x.begin(), int_vector_right_x.end());
            int max_right_y=*max_element(int_vector_right_y.begin(), int_vector_right_y.end());

            int min_x = min_left_x < min_right_x ? min_left_x : min_right_x;
            int max_x = max_left_x > max_right_x ? max_left_x : max_right_x;
            int min_y = min_left_y < min_right_y ? min_left_y : min_right_y;
            int max_y = max_left_y > max_right_y ? max_left_y : max_right_y;

            int width = max_x - min_x; // max_x - min_x
            int height = max_y - min_y; // max_y - min_y

            std::vector<int32_t> int_vector_x;
            std::vector<int32_t> int_vector_y;
            for(size_t i = 0 ; i<int_vector_left_x.size() ; i++)
            {
                int min_x_for_fill = int_vector_left_x[i]<int_vector_right_x[i]?int_vector_left_x[i]:int_vector_right_x[i];
                int min_y_for_fill = int_vector_left_y[i]<int_vector_right_y[i]?int_vector_left_y[i]:int_vector_right_y[i];
                int x = abs(int_vector_left_x[i]-int_vector_right_x[i]);
                int y = abs(int_vector_left_y[i]-int_vector_right_y[i]);
                bool inc_x_points = x > y ? true : false;
                if(inc_x_points)
                {
                    for(int j = 0 ; j<x ; j++)
                    {
                        int_vector_x.push_back(min_x_for_fill + j);
                        int_vector_y.push_back(min_y_for_fill);
                    }
                }
                else
                {
                    for(int j = 0 ; j<y ; j++)
                    {
                        int_vector_x.push_back(min_x_for_fill);
                        int_vector_y.push_back(min_y_for_fill + j);
                    }
                }
            }

            occupancy_grid_msg.header.frame_id = "map";  // Doldurmak istediğiniz frame_id'yi belirtin.
            occupancy_grid_msg.info.width = width;
            occupancy_grid_msg.info.height = height;
            occupancy_grid_msg.info.origin.position.x = min_x;
            occupancy_grid_msg.info.origin.position.y = min_y;
            occupancy_grid_msg.info.origin.position.z = 0.0;
            occupancy_grid_msg.info.resolution = 1.0; 
            occupancy_grid_msg.data = createMatrixWithMod( int_vector_x , int_vector_y , width , height , min_x , min_y);
            first_ = false;

            grid_msg_.data = {static_cast<double>(width) , static_cast<double>(height),static_cast<double>(min_x),static_cast<double>(min_y)};
        }
        occupancy_grid_publisher_->publish(occupancy_grid_msg);
        obsx_publisher_->publish(x_msg_);
        obsy_publisher_->publish(y_msg_);
        grid_size_publisher->publish(grid_msg_);
    }

    void float64MultiArrayCallback(const std_msgs::msg::Float64MultiArray::SharedPtr t_msg)
    {
        if(first)
        {
            for(size_t i = 0 ; i < t_msg->data.size()/4 ; i++)
            {
                array_data_left_x.push_back(round(t_msg->data[4*i]));
                array_data_left_y.push_back(round(t_msg->data[4*i+1]));
                array_data_right_x.push_back(round(t_msg->data[4*i+2]));
                array_data_right_y.push_back(round(t_msg->data[4*i+3]));
            }
            first = false;
        }
    }


    std::vector<int8_t> createMatrixWithMod(std::vector<int32_t>& int_vector_x, std::vector<int32_t>& int_vector_y, int& rows, int& columns, int& min_x, int& min_y) 
    {
        std::cout << int_vector_y.size()<< std::endl;
        std::vector<int8_t> matrix_data(rows * columns, 100);
        for (size_t i = 0; i < int_vector_x.size(); ++i) 
        {
            int32_t x = int_vector_x[i] - min_x;
            int32_t y = int_vector_y[i] - min_y;

            if (x >= 0 && x < rows && y >= 0 && y < columns) {
                matrix_data[(y * rows) + x] = 0;
            }
        }

        for( int i = 0 ; i < columns ; i++ )
        {
            for( int j = 0 ; j < rows ; j++ )
            {
                if(matrix_data[(i * rows) + j]==100)
                {
                    x_msg_.data.push_back(j);
                    y_msg_.data.push_back(i);
                }
            }
        }

        return matrix_data;
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr osm_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsx_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr obsy_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grid_size_publisher;
    std_msgs::msg::Float64MultiArray x_msg_;
    std_msgs::msg::Float64MultiArray y_msg_;
    std_msgs::msg::Float64MultiArray grid_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_msg;
    std::vector<double> array_data_left_x;
    std::vector<double> array_data_left_y;
    std::vector<double> array_data_right_x;
    std::vector<double> array_data_right_y;
    bool first_{true};
    bool first{true};
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}
    