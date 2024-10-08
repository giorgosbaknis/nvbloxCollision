#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp" 
#include "visualization_msgs/msg/marker.hpp"
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
#include "nvblox/mapper/mapper_params.h"
#include "nvblox/mapper/multi_mapper.h"
#include "nvblox/sensors/image.h"

#include <iostream>

using namespace std;
using namespace nvblox;

class TopicReader : public rclcpp::Node
{
public:
    TopicReader() : Node("topic_reader")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/husky1/odometry/imu", 10, std::bind(&TopicReader::odom_callback, this, std::placeholders::_1));

        pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/husky1/ouster/points", 10, std::bind(&TopicReader::pc2_callback, this, std::placeholders::_1));

        image_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/husky1/camera/color/image_raw", 10, std::bind(&TopicReader::image_color_callback, this, std::placeholders::_1));

        image_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/husky1/camera/aligned_depth_to_color/image_raw", 10, std::bind(&TopicReader::image_depth_callback, this, std::placeholders::_1));

        // Publisher for collision point
        collision_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/husky1/ouster/collision", 5);

        

        params.esdf_slice_min_height = 0.0f;
        params.esdf_slice_max_height = 1.0f;
        params.esdf_slice_height = 1.0f;
        params.projective_integrator_max_integration_distance_m = 7.0;
        params.lidar_projective_integrator_max_integration_distance_m = 20.0;
        params.projective_integrator_truncation_distance_vox = 4.0;
        params.projective_integrator_weighting_mode = nvblox::WeightingFunctionType::kInverseSquareWeight;
        params.projective_integrator_max_weight = 100.0;
        params.free_region_occupancy_probability = 0.3;
        params.occupied_region_occupancy_probability = 0.7;
        params.unobserved_region_occupancy_probability = 0.5;
        params.occupied_region_half_width_m = 0.1;
        params.free_region_decay_probability = 0.55;
        params.occupied_region_decay_probability = 0.4;
        params.mesh_integrator_min_weight = 0.0001;
        params.mesh_integrator_weld_vertices = true;
        params.esdf_integrator_min_weight = 0.0001;
        params.esdf_integrator_max_site_distance_vox = 1.0; //1.0
        params.esdf_integrator_max_distance_m = 2.0; //2.0


        mapper.setMapperParams(params);

    }

private:

    DepthImage depthImageFromPointcloud(const Eigen::MatrixX3f& pointcloud_,
                                    const Lidar& lidar_) {
    
    DepthImage depth_image_(lidar_.num_elevation_divisions(),
                            lidar_.num_azimuth_divisions(), MemoryType::kUnified);
        depth_image_.setZero();
        
        for (int idx = 0; idx < pointcloud_.rows(); idx++) {
            const Vector3f p_C = pointcloud_.row(idx);
            
            Index2D u_C ;
            
            lidar_.project(p_C, &u_C);
            
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
        
        
        
        //DepthImage depth_image = depthImageFromPointcloud(pointcloud,lidar);

        // mapper.integrateLidarDepth(depth_image,tf_lidar_to_world, lidar);

        //Camera integrate depth 
       mapper.integrateDepth(depth_image_camera, tf_camera_to_world, camera);
       mapper.integrateColor(color_image, tf_camera_to_world, camera);

        // Produce the esdf
        mapper.updateEsdf();
        mapper.updateMesh(UpdateFullLayer::kNo,tf_camera_to_world);
        
        //OccupancyLayer& occupancy_layer = mapper.occupancy_layer();

        mapper.saveMeshAsPly("./mesh.ply");
        // mapper.saveEsdfAsPly("./test.ply");
        
        //mapper.saveLayerCake("layerCake");

        checkCollision();
        
    }

    
    void image_color_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image data to ColorImage data
        for (int row_idx = 0; row_idx < msg->height; ++row_idx)
        {
            for (int col_idx = 0; col_idx < msg->width; ++col_idx)
            {
                int index = row_idx * msg->step + col_idx * 3; // Calculate index for rgb8
                uint8_t r = msg->data[index];
                uint8_t g = msg->data[index + 1];
                uint8_t b = msg->data[index + 2];

                Color color(r, g, b);
                color_image(row_idx, col_idx) = color;
            }
        }
    }
    
    void image_depth_callback(const sensor_msgs::msg::Image::SharedPtr msg){
        // Convert the ROS image data to DepthImage data
        for (int row_idx = 0; row_idx < msg->height; ++row_idx)
        {
            for (int col_idx = 0; col_idx < msg->width; ++col_idx)
            {
                int index = row_idx * msg->step + col_idx * 2; // 2 bytes per pixel (16 bits)
                uint16_t depth_value = msg->data[index] | (msg->data[index + 1] << 8); // Combine the two bytes
                //cout<<"depth value: "<<depth_value<<endl;
                depth_image_camera(row_idx, col_idx) = static_cast<float>(depth_value) / 1000.0f; // Convert to meters if necessary
            }
        }
    }
    
    void create_transformation_matrix(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        
        position_odometry << msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z;

        
        // Extract position and orientation from odometry for base_link in world frame
        Eigen::Isometry3f base_to_world = Eigen::Isometry3f::Identity();
        base_to_world.translation() << msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z;

        Eigen::Quaternionf q_world(msg->pose.pose.orientation.w,
                                msg->pose.pose.orientation.x,
                                msg->pose.pose.orientation.y,
                                msg->pose.pose.orientation.z);

        rotation_matrix_odometry = q_world.toRotationMatrix();

        base_to_world.rotate(q_world);

        // Static transform from base_link to os_sensor (LiDAR frame)
        Eigen::Isometry3f base_to_lidar = Eigen::Isometry3f::Identity();
        // Translation is zero as per provided data
        base_to_lidar.translation() << 0.0, 0.0, 0.0;

        // Rotation quaternion from provided transform data
        Eigen::Quaternionf q_lidar(0.9961947202682495, // w
                                0.0,                 // x
                                0.08715575188398361, // y
                                0.0);                // z
        base_to_lidar.rotate(q_lidar);

        // Combine transformations to get LiDAR to world
        tf_lidar_to_world = base_to_world * base_to_lidar;


        // Static transform from base_link to camera (camera frame)
        Eigen::Isometry3f base_to_camera = Eigen::Isometry3f::Identity();
        // Set translation
        base_to_camera.translation() << 0.1, 0.0, -0.1;
        
        // Set rotation
        Eigen::Matrix3f rotation_matrix_camera;
        rotation_matrix_camera << 0.0, 0.0, -1.0,
                                1.0, 0.0,  0.0,
                                0.0, 1.0,  0.0;
        

        base_to_camera.rotate(rotation_matrix_camera);

        // Combine transformations to get camera to world
        tf_camera_to_world = base_to_world * base_to_camera;

    }

    void checkCollision(){   
        
        // Find collision point to 5 meters to the right
        // Extract the position from the odometry message
        
        int no_obstacle_flag = 0;

        // Define a vector pointing to the right in the local frame
        Eigen::Vector3f right_vector(0.0f, -1.0f, 0.0f); 

        // Transform the right vector into the world frame
        Eigen::Vector3f world_right_vector = rotation_matrix_odometry * right_vector;

        EsdfLayer& esdf_layer = mapper.esdf_layer(); 
        // OccupancyLayer& esdf_layer = mapper.occupancy_layer();

        for(float pos = 0.2f; pos<=5.0f; pos = pos + 0.2f){
            
            // Calculate the collision point untill 5 meters to the right
            collision_point = position_odometry + pos * world_right_vector;
            
            auto result = esdf_layer.getVoxel(collision_point);
            //cout<<collision_point.transpose()<<endl;
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


        // Publish the collision point
        publish_collision_point(collision_point);

    }


    void publish_collision_point(const Eigen::Vector3f &point)
    {
        auto point_msg = geometry_msgs::msg::PointStamped();
        point_msg.header.stamp = this->get_clock()->now();
        point_msg.header.frame_id = "world";  // Set the frame ID as appropriate

        point_msg.point.x = point.x();
        point_msg.point.y = point.y();
        point_msg.point.z = point.z();
        collision_point_pub_->publish(point_msg);
    }

 

    const int num_azimuth_divisions = 440;
    const int num_elevation_divisions = 900;

    const float vertical_fov_rad = 32.0 * M_PI / 180.0;
    const float min_valid_range_m = 0.0f;
    const float max_valid_range_m = 7.0f;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_depth_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr collision_point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr esdf_marker_pub_; 

    nav_msgs::msg::Odometry last_odometry_;
    Eigen::Isometry3f tf_lidar_to_world = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f tf_camera_to_world = Eigen::Isometry3f::Identity();
    Mapper mapper{0.2f, MemoryType::kDevice};

    MapperParams params;

    Lidar lidar{num_azimuth_divisions, num_elevation_divisions, min_valid_range_m,
                          max_valid_range_m, vertical_fov_rad};


    // Color camera intrinsic parameters
    const float fu = 643.591796875;      // Focal length in the u (x/width) direction
    const float fv = 642.68017578125;    // Focal length in the v (y/height) direction
    const int width = 1280;              // Width of the image plane
    const int height = 720;              // Height of the image plane
    const float cu = 654.1656494140625;  // Principal point in the u (x/width) direction
    const float cv = 362.5810241699219;  // Principal point in the v (y/height) direction
    Camera camera{fu, fv, cu, cv, width, height};


    //Color image from camera
    ColorImage color_image{height, width, MemoryType::kUnified};

    //Depth image from camera
    DepthImage depth_image_camera{height, width, MemoryType::kUnified};

    Eigen::Vector3f collision_point;
    Eigen::Vector3f position_odometry = Eigen::Vector3f::Zero();  // Initialize to zero
    Eigen::Matrix3f rotation_matrix_odometry = Eigen::Matrix3f::Identity();  // Initialize to identity matrix

    
};

int main(int argc, char **argv)
{
   
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
