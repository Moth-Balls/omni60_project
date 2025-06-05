/* Early fusion object detection - Publishes a single combined point cloud */

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

rclcpp::Publisher<PointCloud2>::SharedPtr combined_cloud_publisher_;

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud; // Stores the latest raw LiDAR cloud
std::mutex pcl_cloud_mutex;                                 // Mutex to protect access to pcl_cloud

// Bounding Boxes
std::array<std::vector<BoundingBoxData>, 5> detected_bounding_boxes;

// Image Subscriptions
std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 5> image_subscriptions;

std::array<std::string, 5> camera_frame_ids = {"camera0_link",
                                               "camera1_link",
                                               "camera2_link",
                                               "camera3_link",
                                               "camera4_link"};

// Store the point clouds from each camera
std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 5> camera_clouds;
std::array<bool, 5> camera_cloud_ready = {false, false, false, false, false};
std::mutex camera_clouds_mutex;

void lidar_callback(const PointCloud2::SharedPtr point_cloud_msg) {
    if (!node) {
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
    pcl_cloud = temp_pcl_cloud;
    RCLCPP_DEBUG(node->get_logger(), "lidar_callback: Updated global pcl_cloud with %zu points in frame %s", pcl_cloud->size(), pcl_cloud->header.frame_id.c_str());
}

void image_callback(const RosImage::SharedPtr camera_image_msg, int camera_index) {
    if (!node) return;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> current_lidar_scan;
    rclcpp::Time lidar_scan_timestamp(0,0,RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(pcl_cloud_mutex);
        if (!pcl_cloud || pcl_cloud->empty()) {
            RCLCPP_DEBUG(node->get_logger(), "image_callback for cam %d: Global pcl_cloud not available.", camera_index);
            return;
        }
        current_lidar_scan = pcl_cloud; // Get a shared_ptr copy
        if(current_lidar_scan->header.stamp > 0){
            lidar_scan_timestamp = rclcpp::Time(current_lidar_scan->header.stamp * 1000LL); 
        } else {
            // Fallback if lidar PCL stamp is not set, though fromROSMsg should set it.
            lidar_scan_timestamp = rclcpp::Time(camera_image_msg->header.stamp);
        }
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
        // Even if no boxes, mark as ready to not block indefinitely if other cameras have data
        std::lock_guard<std::mutex> lock(camera_clouds_mutex);
        camera_clouds[camera_index] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // Empty cloud
        camera_clouds[camera_index]->header.frame_id = current_camera_frame;
        rclcpp::Time image_stamp_for_header(camera_image_msg->header.stamp);
        camera_clouds[camera_index]->header.stamp = image_stamp_for_header.nanoseconds() / 1000ULL;
        camera_cloud_ready[camera_index] = true;
        // Proceed to check if all are ready
    } else {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_camera_frame(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            if (current_lidar_scan->header.frame_id.empty()) {
                RCLCPP_WARN(node->get_logger(), "Global PCL cloud has empty frame_id for cam %d.", camera_index);
                return;
            }
            // Use the image message stamp for TF lookup, assuming sensors are reasonably synchronized
            geometry_msgs::msg::TransformStamped transform_to_cam = tf_buffer_->lookupTransform(
                current_camera_frame, current_lidar_scan->header.frame_id, 
                rclcpp::Time(camera_image_msg->header.stamp), rclcpp::Duration::from_seconds(0.2));
            pcl_ros::transformPointCloud(*current_lidar_scan, *cloud_in_camera_frame, transform_to_cam);
            cloud_in_camera_frame->header.frame_id = current_camera_frame;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(node->get_logger(), "Cam %d: Could not transform cloud from %s to %s: %s",
                        camera_index, current_lidar_scan->header.frame_id.c_str(), current_camera_frame.c_str(), ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr points_in_boxes_cam_frame;
        points_in_boxes_cam_frame = project_points(annotated_image, cloud_in_camera_frame, detected_bounding_boxes[camera_index]);

        if (points_in_boxes_cam_frame) {
            points_in_boxes_cam_frame->header.frame_id = current_camera_frame;
            rclcpp::Time image_stamp_for_header(camera_image_msg->header.stamp);
            points_in_boxes_cam_frame->header.stamp = image_stamp_for_header.nanoseconds() / 1000ULL; // PCL stamp is microseconds
        }
        
        std::lock_guard<std::mutex> lock(camera_clouds_mutex);
        camera_clouds[camera_index] = points_in_boxes_cam_frame ? points_in_boxes_cam_frame : std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if(!points_in_boxes_cam_frame) { // If project_points returned null, ensure header for empty cloud
            camera_clouds[camera_index]->header.frame_id = current_camera_frame;
            rclcpp::Time image_stamp_for_header(camera_image_msg->header.stamp);
            camera_clouds[camera_index]->header.stamp = image_stamp_for_header.nanoseconds() / 1000ULL;
        }
        camera_cloud_ready[camera_index] = true;
    }


    {
        std::lock_guard<std::mutex> lock(camera_clouds_mutex);
        bool all_are_ready = true;
        for (bool ready : camera_cloud_ready) {
            if (!ready) {
                all_are_ready = false;
                break;
            }
        }

        if (all_are_ready) {
            RCLCPP_DEBUG(node->get_logger(), "All camera clouds are ready, combining and publishing...");
            std::string target_velodyne_frame_id_local;
            {
                std::lock_guard<std::mutex> pcl_lock(pcl_cloud_mutex); // Lock for accessing global pcl_cloud
                if (pcl_cloud && !pcl_cloud->header.frame_id.empty()) {
                    target_velodyne_frame_id_local = pcl_cloud->header.frame_id;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Velodyne cloud not available or frame_id empty, cannot determine target frame for combining.");
                    // Reset flags to prevent getting stuck if Velodyne cloud is temporarily unavailable
                    for (int i = 0; i < 5; ++i) {
                        camera_cloud_ready[i] = false;
                        // camera_clouds[i].reset(); // Keep potentially valid old data or clear? Let's clear.
                        camera_clouds[i] = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                        // No need to set header for these reset clouds as they won't be used immediately
                    }
                    return; 
                }
            }

            // camera_clouds array is already protected by the outer camera_clouds_mutex
            pcl::PointCloud<pcl::PointXYZ>::Ptr combined_pcl_result = combine_clouds(
                node->get_logger(),
                tf_buffer_,
                camera_clouds, // Pass the global array
                target_velodyne_frame_id_local);

            if (combined_pcl_result && !combined_pcl_result->empty()) {
                sensor_msgs::msg::PointCloud2 combined_ros_msg;
                pcl::toROSMsg(*combined_pcl_result, combined_ros_msg);

                // Use the timestamp from the combined PCL cloud (set by combine_clouds)
                // or fallback to current time if it's zero.
                if (combined_pcl_result->header.stamp > 0) {
                    combined_ros_msg.header.stamp = rclcpp::Time(combined_pcl_result->header.stamp * 1000LL);
                } else {
                    combined_ros_msg.header.stamp = node->get_clock()->now(); 
                }
                combined_ros_msg.header.frame_id = target_velodyne_frame_id_local;
                
                combined_cloud_publisher_->publish(combined_ros_msg);
            } else {
                RCLCPP_WARN(node->get_logger(), "Combined cloud is empty or null, not publishing.");
            }

            // Reset flags and clouds for the next cycle
            for (int i = 0; i < 5; ++i) {
                camera_cloud_ready[i] = false;
                camera_clouds[i].reset(); // Clear the shared_ptr
            }
        } else {
            RCLCPP_DEBUG(node->get_logger(), "Not all camera clouds are ready yet for camera %d.", camera_index);
        }
    } 
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("omni60_node");

    RCLCPP_INFO(node->get_logger(), "Omni60 Node Started - Publishing a single combined interest cloud.");

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

    std::string combined_publisher_topic = "/velodyne/target_points";
    combined_cloud_publisher_ = node->create_publisher<PointCloud2>(combined_publisher_topic, 10);
    RCLCPP_INFO(node->get_logger(), "Created publisher on %s", combined_publisher_topic.c_str());

    image_subscriptions.fill(nullptr);
    for (int i = 0; i < 5; ++i) {
        image_subscriptions[i] = node->create_subscription<RosImage>(
            camera_topics[i],
            qos,
            [i](const RosImage::SharedPtr msg){
                image_callback(msg, i);
            }
        );
        RCLCPP_INFO(node->get_logger(), "Subscribed to %s", camera_topics[i].c_str());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




