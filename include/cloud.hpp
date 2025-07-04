#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <tf2_ros/buffer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/transforms.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "color_detect.hpp"
#include <cmath>
#include <vector>
#include <std_msgs/msg/header.hpp> 

using RosImage = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

namespace rclcpp 
{
class Logger;
}


inline pcl::PointCloud<pcl::PointXYZ>::Ptr tf_pointcloud(const rclcpp::Logger& logger, const std::shared_ptr<tf2_ros::Buffer>& tf_buffer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, const std::string& target_frame_id, const std_msgs::msg::Header& header) {
    if (!tf_buffer) {
        RCLCPP_WARN(logger, "transform_pointcloud: TF buffer is null.");
        return nullptr;
    }
    if (!input_cloud) {
        RCLCPP_WARN(logger, "transform_pointcloud: Input cloud message is null.");
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer->lookupTransform(target_frame_id, header.frame_id, header.stamp);
        pcl_ros::transformPointCloud(*input_cloud, *transformed_cloud, transform);

    } catch (tf2::TransformException &ex) 
    {
        RCLCPP_WARN(logger, "Could not transform point cloud: %s", ex.what());
        return nullptr;
    }

    return transformed_cloud;
}


inline std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filter_cloud(const rclcpp::Logger& logger, const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_clouds(5);
    std::vector<std::pair<float, float>> angle_ranges = {
        {-46.0f, 46.0f},    // Camera 0
        {26.0f, 118.0f},   // Camera 1
        {98.0f, 190.0f},  // Camera 2
        {170.0f, 262.0f}, // Camera 3
        {242.0f, 334.0f}  // Camera 4
    };

    if (!input_cloud) 
    {
        RCLCPP_WARN(logger, "filter_lidar_data: Input cloud message is null.");
        return filtered_clouds; // Return 5 empty clouds
    }

    for (size_t i = 0; i < 5; ++i)
    {
        filtered_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
        filtered_clouds[i]->header = input_cloud->header;

        float min_angle_rad = angle_ranges[i].first * M_PI / 180.0f;
        float max_angle_rad = angle_ranges[i].second * M_PI / 180.0f;

        for (const auto& point : input_cloud->points) 
        {
            if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))) 
            {
                continue;
            }

            float angle = std::atan2(point.y, point.x);
            float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            if (angle >= min_angle_rad && angle <= max_angle_rad && distance > 1.0f) 
            {
                filtered_clouds[i]->emplace_back(point);
            }
        }
    }

    return filtered_clouds;
}


inline pcl::PointCloud<pcl::PointXYZ>::Ptr project_points(
    cv::Mat& img, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<BoundingBoxData>& boxes) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr points_in_boxes(new pcl::PointCloud<pcl::PointXYZ>);

    // if the input cloud is null or empty, return an empty cloud
    if (!cloud || cloud->empty()) {
        if (cloud) {
             points_in_boxes->header = cloud->header;
        }
        points_in_boxes->width = 0;
        points_in_boxes->height = 1;
        points_in_boxes->is_dense = true;
        return points_in_boxes;
    }

   
    points_in_boxes->header = cloud->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices; // To store indices of valid points
    pcl::removeNaNFromPointCloud(*cloud, *cloud_no_nan, indices);

    // if all points were NaN/Inf, return an empty cloud
    if (cloud_no_nan->empty()) {
        points_in_boxes->width = 0;
        points_in_boxes->height = 1;
        points_in_boxes->is_dense = true;
        return points_in_boxes;
    }
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_stat_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_no_nan);
    sor.setMeanK(10);
    sor.setStddevMulThresh(0.4);
    sor.filter(*cloud_stat_filtered);

    // if all points filtered out return empty
    if (cloud_stat_filtered->empty()) {
        points_in_boxes->width = 0;
        points_in_boxes->height = 1;
        points_in_boxes->is_dense = true;
        return points_in_boxes;
    }

    const float fx = 241.4268569946289f;
    const float fy = 241.42684936523438f;
    const float cx = 376.0f;
    const float cy = 240.0f;

    const float img_width = 752.0f;
    const float img_height = 480.0f;

    // intrinsic camera data
    Eigen::Matrix3f K;
    K << fx, 0.0f, cx,
         0.0f, fy,   cy,
         0.0f, 0.0f, 1.0f;


    Eigen::Matrix3f cam_to_optical;
    cam_to_optical <<
        0.0f, -1.0f,  0.0f,
        0.0f,  0.0f, -1.0f,
        1.0f,  0.0f,  0.0f;

    Eigen::MatrixXf points_cam_filtered(3, cloud_stat_filtered->size());
    for (size_t i = 0; i < cloud_stat_filtered->size(); ++i) {
        points_cam_filtered(0, i) = cloud_stat_filtered->points[i].x;
        points_cam_filtered(1, i) = cloud_stat_filtered->points[i].y;
        points_cam_filtered(2, i) = cloud_stat_filtered->points[i].z;
    }

    // transform points to the camera optical frame
    Eigen::MatrixXf points_optical_filtered = cam_to_optical * points_cam_filtered;


    Eigen::MatrixXf projected_coords_filtered = K * points_optical_filtered;

    for (size_t i = 0; i < cloud_stat_filtered->size(); ++i) {

        float Z = points_optical_filtered(2, i);

        // point must be in front of the camera
        if (Z <= 1e-3f) {
            continue;
        }

        // calculate 2D image coordinates (u, v)
        float u = projected_coords_filtered(0, i) / Z;
        float v = projected_coords_filtered(1, i) / Z;

        bool in_box = false;
        if (!boxes.empty()) {
            for (const auto& box_data : boxes) {
                const cv::Rect& box = box_data.rect;
                // check if the projected point (u,v) is inside the current bounding box
                if (u >= box.x && u < (box.x + box.width) &&
                    v >= box.y && v < (box.y + box.height)) {
                    in_box = true;
                    break; 
                }
            }
        }

        if (in_box) {
            points_in_boxes->points.push_back(cloud_stat_filtered->points[i]);

            if (u >= 0 && u < img_width && v >= 0 && v < img_height) {
                cv::circle(img, cv::Point(static_cast<int>(u), static_cast<int>(v)), 2, cv::Scalar(0, 255, 0), -1);
            }
        }
    }

    points_in_boxes->width = points_in_boxes->points.size();
    points_in_boxes->height = 1;
    points_in_boxes->is_dense = true;

    return points_in_boxes;
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr combine_clouds (
    const rclcpp::Logger& logger,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const std::array<pcl::PointCloud<pcl::PointXYZ>::Ptr, 5>& camera_clouds,
    const std::string& target_velodyne_frame_id)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    combined_cloud_ptr->header.frame_id = target_velodyne_frame_id;

    rclcpp::Time latest_stamp(0, 0, RCL_ROS_TIME);

    for (const auto& single_camera_cloud : camera_clouds) {
        if (!single_camera_cloud) {
            continue;
        }
        if (single_camera_cloud->empty()) {
            continue;
        }

        std_msgs::msg::Header source_ros_header;
        source_ros_header.frame_id = single_camera_cloud->header.frame_id;

        if (source_ros_header.frame_id.empty()) {
            RCLCPP_WARN(logger, "Source cloud for combination has an empty frame_id. Skipping this cloud.");
            continue;
        }
        
        uint64_t stamp_mc = single_camera_cloud->header.stamp; 
        source_ros_header.stamp = rclcpp::Time(stamp_mc * 1000LL);

        rclcpp::Time current_segment_stamp(source_ros_header.stamp);

        if (current_segment_stamp.nanoseconds() > 0 && current_segment_stamp > latest_stamp) {
            latest_stamp = current_segment_stamp;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_segment_ptr;

        if (source_ros_header.frame_id == target_velodyne_frame_id) {
            transformed_segment_ptr = single_camera_cloud;
        } else {
            
            transformed_segment_ptr = tf_pointcloud(logger, tf_buffer, single_camera_cloud, target_velodyne_frame_id, source_ros_header);
        }

        if (transformed_segment_ptr && !transformed_segment_ptr->empty()) {
            *combined_cloud_ptr += *transformed_segment_ptr;
        } else {
            RCLCPP_WARN(logger, "Failed to transform or segment was empty for cloud from frame %s to %s. Skipping this segment.",
                         source_ros_header.frame_id.c_str(), target_velodyne_frame_id.c_str());
        }
    }

    if (combined_cloud_ptr->points.empty()) {
        RCLCPP_DEBUG(logger, "Combined cloud is empty after processing all segments.");
    }

    combined_cloud_ptr->width = combined_cloud_ptr->points.size();
    combined_cloud_ptr->height = 1;
    combined_cloud_ptr->is_dense = true; 

    if (latest_stamp.nanoseconds() > 0) {
         combined_cloud_ptr->header.stamp = latest_stamp.nanoseconds() / 1000LL; 
    }
    
    return combined_cloud_ptr;
}
