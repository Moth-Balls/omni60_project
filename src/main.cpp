/* Early fusion object detection - Publishes 5 separate point clouds */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <string>
#include <memory>
#include <vector>
#include <array>
#include <mutex> 

#include "color_detect.hpp"
#include "cloud.hpp"



// Global Variables
std::array<cv::Mat, 5> original_img_for_projection;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
rclcpp::Node::SharedPtr node;

// Publishers for each camera's points of interest
std::array<rclcpp::Publisher<PointCloud2>::SharedPtr, 5> camera_interest_points_publishers_;

// PCL Cloud
std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud; // Stores the latest raw LiDAR cloud
std::mutex pcl_cloud_mutex; // Mutex to protect access to pcl_cloud

// Bounding Boxes
std::array<std::vector<BoundingBoxData>, 5> detected_bounding_boxes;

// Image Subscriptions
std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 5> image_subscriptions;

std::array<std::string, 5> camera_frame_ids = {"camera0_link",
                                               "camera1_link",
                                               "camera2_link",
                                               "camera3_link",
                                               "camera4_link"};

void lidar_callback(const PointCloud2::SharedPtr point_cloud_msg) {
    if (!node) {
        // RCLCPP_DEBUG(node->get_logger(), "lidar_callback: Node not initialized.");
        return;
    }

    if (!point_cloud_msg || point_cloud_msg->data.empty()) {
        RCLCPP_WARN(node->get_logger(), "lidar_callback: Input cloud message is null or empty.");
        std::lock_guard<std::mutex> lock(pcl_cloud_mutex);
        pcl_cloud = nullptr;
        return;
    }

    auto temp_pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*point_cloud_msg, *temp_pcl_cloud);
    
    std::lock_guard<std::mutex> lock(pcl_cloud_mutex);
    pcl_cloud = temp_pcl_cloud; // pcl_cloud is now in point_cloud_msg->header.frame_id
    RCLCPP_DEBUG(node->get_logger(), "lidar_callback: Updated global pcl_cloud with %zu points in frame %s", pcl_cloud->size(), pcl_cloud->header.frame_id.c_str());
}

void image_callback(const RosImage::SharedPtr camera_image_msg, int camera_index) {
    if (!node) return;
    
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> current_lidar_scan;
    {
        std::lock_guard<std::mutex> lock(pcl_cloud_mutex);
        if (!pcl_cloud || pcl_cloud->empty()) {
            RCLCPP_DEBUG(node->get_logger(), "image_callback for cam %d: Global pcl_cloud not available.", camera_index);
            return;
        }
        current_lidar_scan = pcl_cloud; // Get a shared_ptr copy of the current scan
    }

    cv::Mat current_frame;
    try {
        current_frame = cv_bridge::toCvCopy(camera_image_msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception for cam %d: %s", camera_index, e.what());
        return;
    }

    original_img_for_projection[camera_index] = current_frame.clone();
    detected_bounding_boxes[camera_index].clear();
    cv::Mat annotated_image = detect_green_object(original_img_for_projection[camera_index], detected_bounding_boxes[camera_index]);

    std::string current_camera_frame = camera_frame_ids[camera_index];

    if (original_img_for_projection[camera_index].empty() || detected_bounding_boxes[camera_index].empty()) {
        // RCLCPP_DEBUG(node->get_logger(), "Cam %d: No image or no bounding boxes.", camera_index);
        return;
    }

    // 1. Transform the global LiDAR scan to the current_camera_frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_camera_frame(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        if (current_lidar_scan->header.frame_id.empty()) {
             RCLCPP_WARN(node->get_logger(), "Global PCL cloud has empty frame_id for cam %d.", camera_index);
             return;
        }
        geometry_msgs::msg::TransformStamped transform_to_cam = tf_buffer_->lookupTransform(
            current_camera_frame, current_lidar_scan->header.frame_id, camera_image_msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
        pcl_ros::transformPointCloud(*current_lidar_scan, *cloud_in_camera_frame, transform_to_cam);
        cloud_in_camera_frame->header.frame_id = current_camera_frame; 
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "Cam %d: Could not transform cloud from %s to %s: %s",
                    camera_index, current_lidar_scan->header.frame_id.c_str(), current_camera_frame.c_str(), ex.what());
        return;
    }

    // 2. Project points using the cloud now in the camera's frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_in_boxes_cam_frame; 
    points_in_boxes_cam_frame = project_points(annotated_image, cloud_in_camera_frame, detected_bounding_boxes[camera_index]);

    if (points_in_boxes_cam_frame && !points_in_boxes_cam_frame->empty()) {
        // 3. Publish this camera-specific cloud of interest
        // These points are already in current_camera_frame
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*points_in_boxes_cam_frame, cloud_msg);
        cloud_msg.header.stamp = camera_image_msg->header.stamp;
        cloud_msg.header.frame_id = current_camera_frame; // Points are in this camera's frame

        if (camera_interest_points_publishers_[camera_index]) {
            camera_interest_points_publishers_[camera_index]->publish(cloud_msg);
            RCLCPP_DEBUG(node->get_logger(), "Cam %d: Published %zu points of interest to topic for frame %s.",
                        camera_index, points_in_boxes_cam_frame->size(), current_camera_frame.c_str());
        }
    }
    
    
    // cv::imshow("Annotated Cam " + std::to_string(camera_index), annotated_image);
    // cv::waitKey(1);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("omni60_node");

    RCLCPP_INFO(node->get_logger(), "Omni60 Node Started - Publishing 5 separate interest clouds.");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

    auto qos = rclcpp::SensorDataQoS();

    auto lidar_subscription = node->create_subscription<PointCloud2>(
        "/velodyne_points", qos, lidar_callback);

    std::array<std::string, 5> camera_topics = {"/camera0/image_raw",
                                               "/camera1/image_raw",
                                               "/camera2/image_raw",
                                               "/camera3/image_raw",
                                               "/camera4/image_raw"};

    for (int i = 0; i < 5; ++i) {
        // Create a unique topic name for each camera's points of interest
        std::string publisher_topic = "/camera" + std::to_string(i) + "/points_of_interest";
        camera_interest_points_publishers_[i] = node->create_publisher<PointCloud2>(publisher_topic, 10);
        
        image_subscriptions[i] = node->create_subscription<RosImage>(
            camera_topics[i], 
            qos, 
            [i](const RosImage::SharedPtr msg){ 
                image_callback(msg, i);
            }
        );
        RCLCPP_INFO(node->get_logger(), "Subscribed to %s and created publisher on %s", camera_topics[i].c_str(), publisher_topic.c_str());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




