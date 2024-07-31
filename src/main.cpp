#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <Eigen/Geometry>

//Nvblox test libs 
#include "nvblox/core/types.h"
#include "nvblox/interpolation/interpolation_2d.h"
#include "nvblox/sensors/image.h"
#include "nvblox/nvblox.h"
#include "nvblox/sensors/lidar.h"
#include "nvblox/map/blox.h"
#include "nvblox/map/layer.h"
#include "nvblox/map/layer_cake.h"
#include "nvblox/map/voxels.h"
#include "nvblox/mapper/mapper.h"
#include "nvblox/mapper/multi_mapper.h"

#include <iostream>

using namespace std;
using namespace nvblox;

class TopicReader : public rclcpp::Node
{
public:
    TopicReader() : Node("topic_reader")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/imu", 10, std::bind(&TopicReader::odom_callback, this, std::placeholders::_1));
        
        pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", 10, std::bind(&TopicReader::pc2_callback, this, std::placeholders::_1));
    }

private:

    DepthImage depthImageFromPointcloud(const Eigen::MatrixX3f& pointcloud_,
                                    const Lidar& lidar_) {
    
    DepthImage depth_image_(lidar_.num_elevation_divisions(),
                            lidar_.num_azimuth_divisions(), MemoryType::kUnified);
        depth_image_.setZero();
        
        bool check;
        for (int idx = 0; idx < pointcloud_.rows(); idx++) {
            const Vector3f p_C = pointcloud_.row(idx);
            
            Index2D u_C ;
            
            check = lidar_.project(p_C, &u_C);
            
             if (u_C.y() >= 0 && u_C.y() < depth_image_.rows() && u_C.x() >= 0 && u_C.x() < depth_image_.cols()) {
                depth_image_(u_C.y(), u_C.x()) = p_C.norm();
            } else {
                //RCLCPP_WARN(this->get_logger(), "Index out of bounds: (%d, %d)", u_C.y(), u_C.x());
            }
            
           
        }
            
        return depth_image_;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Save odometry data to a member variable
        last_odometry_ = *msg;

    }

    void pc2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        Eigen::MatrixX3f pointcloud(num_azimuth_divisions * num_elevation_divisions, 3);

        // Call create_transformation_matrix with the last odometry data
        create_transformation_matrix(std::make_shared<nav_msgs::msg::Odometry>(last_odometry_));
        
        float last_x=1;
        float last_y=1;
        float last_z=1;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        size_t num_points = msg->width * msg->height;
        
        for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
            
            if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)){
                
                pointcloud(i,0) = last_x;
                pointcloud(i,1) = last_y;
                pointcloud(i,2) = last_z; 

            }
            else{
                last_x = *iter_x;
                last_y = *iter_y;
                last_z = *iter_z;
                
                // RCLCPP_INFO(this->get_logger(), "(%f, %f, %f)", *iter_x, *iter_y, *iter_z);
                pointcloud(i,0) = *iter_x;
                pointcloud(i,1) = *iter_y;
                pointcloud(i,2) = *iter_z;  

            }
            
        }

        DepthImage depth_image = depthImageFromPointcloud(pointcloud,lidar);

        mapper.integrateLidarDepth(depth_image,transform, lidar);

        // // Produce the ESDF
        mapper.updateEsdf();
        
        checkCollision();
        
    }

    void create_transformation_matrix(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        transform.translation() << msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z;

        // Extract orientation (quaternion) and convert to rotation matrix
        Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                             msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y,
                             msg->pose.pose.orientation.z);
        transform.rotate(q);
        
        position << msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z;

        rotation_matrix = q.toRotationMatrix();

    }

    void checkCollision(){   
        
        // Find collision point to 5 meters to the right
        // Extract the position from the odometry message
        
        int no_obstacle_flag = 0;

        // Define a vector pointing to the right in the local frame
        Eigen::Vector3f right_vector(1.0f, 0.0f, 0.0f); 

        // Transform the right vector into the world frame
        Eigen::Vector3f world_right_vector = rotation_matrix * right_vector;

        EsdfLayer& esdf_layer = mapper.esdf_layer(); 
        //cout<<"Robot Position: "<< position.transpose() <<endl;

        for(float pos = 0.1f; pos<=5.0f; pos = pos + 0.1f){
            
            // Calculate the collision point untill 5 meters to the right
            collision_point = position + pos * world_right_vector;
            
            auto result = esdf_layer.getVoxel(collision_point);
            
            if(result.first.observed && result.first.is_inside ){
                RCLCPP_INFO(this->get_logger(), "Collision Point: x: %f, y: %f, z: %f",
                collision_point.x(), collision_point.y(), collision_point.z());


                RCLCPP_INFO(this->get_logger(), "Position to the right: %f", pos);
                
                no_obstacle_flag = 1;
                
                break;

            }

        
        }
        if(no_obstacle_flag == 0){
            RCLCPP_INFO(this->get_logger(), "No obstacle to the right!!!!!!!");
            
        }

    }


    // const int num_azimuth_divisions = 1024;
    // const int num_elevation_divisions = 32;
    
    const int num_azimuth_divisions = 440;
    const int num_elevation_divisions = 900;

    const float vertical_fov_rad = 32.0 * M_PI / 180.0;
    const float min_valid_range_m = 0.0f;
    const float max_valid_range_m = 20.0f;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
    nav_msgs::msg::Odometry last_odometry_;
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    Mapper mapper{0.2f, MemoryType::kDevice};
    Lidar lidar{num_azimuth_divisions, num_elevation_divisions, min_valid_range_m,
                          max_valid_range_m, vertical_fov_rad};
    Eigen::Vector3f collision_point;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();  // Initialize to zero
    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();  // Initialize to identity matrix

    
};

int main(int argc, char **argv)
{
   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
