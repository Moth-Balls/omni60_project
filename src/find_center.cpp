/* 3D Point Cloud center finder */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/Dense>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;

rclcpp::Node::SharedPtr node = nullptr;

rclcpp::Publisher<PointCloud2>::SharedPtr flattened_cloud_publisher = nullptr;
rclcpp::Publisher<PointCloud2>::SharedPtr cluster_centroids_publisher = nullptr;


sensor_msgs::msg::PointCloud2::SharedPtr pcl_to_ros_msg(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ptr, const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*cloud_ptr, *cloud_msg);
    cloud_msg->header.frame_id = frame_id;
    return cloud_msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr flatten_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud) {
    if (!input_cloud) {
        RCLCPP_WARN(node->get_logger(), "flatten_cloud: Input cloud is null.");
        return nullptr;
    }

    if (input_cloud->empty()) {
        RCLCPP_WARN(node->get_logger(), "flatten_cloud: Input cloud is empty.");
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    flattened_cloud->header = input_cloud->header; 
    flattened_cloud->width = input_cloud->width;
    flattened_cloud->height = input_cloud->height;
    flattened_cloud->is_dense = input_cloud->is_dense;

    for (const auto& point : input_cloud->points) {
        pcl::PointXYZ flattened_point;
        flattened_point.x = point.x;
        flattened_point.y = point.y;
        flattened_point.z = 0; 
        flattened_cloud->points.push_back(flattened_point);
    }

    return flattened_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_removal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double mean_k, double stddev_mult_thresh) {
    if (!cloud) {
        RCLCPP_WARN(node->get_logger(), "statistical_outlier_removal: Input cloud is null.");
        return nullptr;
    }

    if (cloud->empty()) {
        RCLCPP_WARN(node->get_logger(), "statistical_outlier_removal: Input cloud is empty.");
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mult_thresh);
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr extract_clusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double cluster_tolerance, int min_size, int max_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>); 

    if (!cloud || cloud->empty()) {
        RCLCPP_WARN(node->get_logger(), "extract_clusters: Input cloud is null or empty.");
        return cluster_centroids_cloud; 
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_size);       
    ec.setMaxClusterSize(max_size);       
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto& cluster : cluster_indices) {
        float x_sum = 0, y_sum = 0, z_sum = 0;
        for (int index : cluster.indices) {
            x_sum += cloud->points[index].x;
            y_sum += cloud->points[index].y;
            z_sum += cloud->points[index].z;
        }

        pcl::PointXYZ centroid;
        centroid.x = x_sum / cluster.indices.size();
        centroid.y = y_sum / cluster.indices.size();
        centroid.z = z_sum / cluster.indices.size();

        cluster_centroids_cloud->push_back(centroid);
    }

    cluster_centroids_cloud->header = cloud->header;
    return cluster_centroids_cloud;
}

void cloud_callback(const PointCloud2::SharedPtr point_cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*point_cloud_msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr flattened_cloud = flatten_cloud(pcl_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = statistical_outlier_removal(flattened_cloud, 50, 1.0);

    // parameters for Euclidean Clustering
    double cluster_tolerance = 0.5;  // adjust
    int min_cluster_size = 10;       // adjust
    int max_cluster_size = 1000;      // adjust

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_centroids = extract_clusters(filtered_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);

    // Publish cluster centroids
    if (cluster_centroids && !cluster_centroids->empty()) {
        sensor_msgs::msg::PointCloud2::SharedPtr cluster_centroids_msg =
            pcl_to_ros_msg(cluster_centroids, point_cloud_msg->header.frame_id); 
        cluster_centroids_msg->header.stamp = point_cloud_msg->header.stamp;
        cluster_centroids_publisher->publish(*cluster_centroids_msg);
    }

    if (filtered_cloud) {
        sensor_msgs::msg::PointCloud2::SharedPtr flattened_cloud_msg =
            pcl_to_ros_msg(filtered_cloud, point_cloud_msg->header.frame_id);
        flattened_cloud_msg->header.stamp = point_cloud_msg->header.stamp;
        flattened_cloud_publisher->publish(*flattened_cloud_msg);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc,argv);

    node = rclcpp::Node::make_shared("find_center_node");

    // euclidean clustering parameters
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;

    // define parameters
    node->declare_parameter<double>("cluster_tolerance", 0.5);
    node->declare_parameter<int>("min_cluster_size", 10);
    node->declare_parameter<int>("max_cluster_size", 1000);

    // fetch parameters
    node->get_parameter("cluster_tolerance", cluster_tolerance);
    node->get_parameter("min_cluster_size", min_cluster_size);
    node->get_parameter("max_cluster_size", max_cluster_size);

    RCLCPP_INFO(node->get_logger(), "cluster_tolerance: %f", cluster_tolerance);
    RCLCPP_INFO(node->get_logger(), "min_cluster_size: %d", min_cluster_size);
    RCLCPP_INFO(node->get_logger(), "max_cluster_size: %d", max_cluster_size);

    RCLCPP_INFO(node->get_logger(), "Find Center Node Started");

    flattened_cloud_publisher = node->create_publisher<PointCloud2>("/flattened_cloud", 10);
    cluster_centroids_publisher = node->create_publisher<PointCloud2>("/center_points", 10);

    auto cloud_subscription = node->create_subscription<PointCloud2>("/velodyne/target_points", 10, cloud_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




