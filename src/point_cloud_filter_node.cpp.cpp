// MEDIAN FILTERING AND RANSAC
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/median_filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub_filtered;   // Median and distance filtered points
ros::Publisher pub_ground;     // Ground points
ros::Publisher pub_nonGround;  // Non-ground points

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*input_cloud_msg, *cloud);

    // Apply Median Filtering first
    pcl::PointCloud<pcl::PointXYZ>::Ptr median_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::MedianFilter<pcl::PointXYZ> median_filter;
    median_filter.setInputCloud(cloud);
    median_filter.setWindowSize(5);  // Set the window size for median filtering
    median_filter.filter(*median_filtered_cloud);

    // Apply distance filtering after median filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr distance_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    double min_distance = 5.0;
    double max_distance = 30.0;

    for (size_t i = 0; i < median_filtered_cloud->points.size(); ++i) {
        const auto& point = median_filtered_cloud->points[i];
        double distance_xy = std::sqrt(point.x * point.x + point.y * point.y);

        if (distance_xy >= min_distance && distance_xy <= max_distance) {
            distance_filtered_cloud->points.push_back(point);
        }
    }

    // Publish the output of median and distance filtering
    sensor_msgs::PointCloud2 output_filtered;
    pcl::toROSMsg(*distance_filtered_cloud, output_filtered);
    output_filtered.header = input_cloud_msg->header;
    pub_filtered.publish(output_filtered);

    // Ground plane segmentation using RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.5);
    seg.setInputCloud(distance_filtered_cloud);
    seg.segment(*ground_inliers, *coefficients);

    if (ground_inliers->indices.empty()) {
        std::cerr << "No ground plane found." << std::endl;
        return;
    }

    std::cout << "PROCESSING" << std::endl;

    // Extract the ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(distance_filtered_cloud);
    extract.setIndices(ground_inliers);
    extract.setNegative(false);
    extract.filter(*ground_points);

    // Publish ground points
    sensor_msgs::PointCloud2 output_ground;
    pcl::toROSMsg(*ground_points, output_ground);
    output_ground.header = input_cloud_msg->header;
    pub_ground.publish(output_ground);

    // Extract non-ground points
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setNegative(true);
    extract.filter(*non_ground_points);

    // Publish non-ground points
    sensor_msgs::PointCloud2 output_nonGround;
    pcl::toROSMsg(*non_ground_points, output_nonGround);
    output_nonGround.header = input_cloud_msg->header;
    pub_nonGround.publish(output_nonGround);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;

    // Subscribe to the input point cloud topic
    ros::Subscriber sub = nh.subscribe("/mbuggy/os3/points", 1, cloudCallback);

    // Publishers for different outputs
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point_cloud", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/ground_points", 1);
    pub_nonGround = nh.advertise<sensor_msgs::PointCloud2>("/non_ground_points", 1);

    ros::spin();

    return 0;
}

// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/visualization/pcl_visualizer.h>

// // Global publishers
// ros::Publisher pub_filtered;
// ros::Publisher pub_removed;
// ros::Publisher pub_ground;

// void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg) {
//     // Convert ROS message to PCL point cloud
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::fromROSMsg(*input_cloud_msg, *cloud);

//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundCloud(new pcl::PointCloud<pcl::PointXYZ>());

//     // Process the cloud to find points within the specified distance range
//     double min_distance = 5.0;
//     double max_distance = 30.0;

//     for (size_t i = 0; i < cloud->points.size(); ++i) {
//         const auto& point = cloud->points[i];
//         double distance_xy = std::sqrt(point.x * point.x + point.y * point.y);

//         if (distance_xy >= min_distance && distance_xy <= max_distance) {
//             inliers->indices.push_back(i);
//         }
//     }

//     std::cout << "POINT CLOUD PROCESSING" << std::endl;

//     // Extract inliers (filtered points)
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud(cloud);
//     extract.setIndices(inliers);
//     extract.setNegative(false);
//     extract.filter(*cloud_filtered);

//     // Extract outliers (removed points)
//     extract.setNegative(true);
//     extract.filter(*cloud_removed);

//     // Perform plane segmentation using RANSAC on the filtered cloud
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(0.05);  // Adjust this threshold based on your data
//     seg.setInputCloud(cloud_filtered);
//     seg.segment(*plane_inliers, *coefficients);

//     if (plane_inliers->indices.size() == 0) {
//         std::cerr << "No plane found." << std::endl;
//         return;
//     }

//     // Extract the plane points from the filtered cloud
//     extract.setInputCloud(cloud_filtered);
//     extract.setIndices(plane_inliers);
//     extract.setNegative(false);
//     extract.filter(*planeCloud);

//     // Extract the non-ground points (points not on the plane)
//     pcl::PointCloud<pcl::PointXYZ>::Ptr nonGroundCloudFiltered(new pcl::PointCloud<pcl::PointXYZ>());
//     extract.setInputCloud(cloud_filtered);
//     extract.setIndices(plane_inliers);
//     extract.setNegative(true); // Non-ground points
//     extract.filter(*nonGroundCloudFiltered);

//     // Combine the removed points (ground and outliers)
//     pcl::PointCloud<pcl::PointXYZ>::Ptr removedCloud(new pcl::PointCloud<pcl::PointXYZ>());
//     *removedCloud = *planeCloud + *cloud_removed;

//     // Convert to ROS messages and publish the combined point clouds
//     sensor_msgs::PointCloud2 output_filtered;
//     pcl::toROSMsg(*nonGroundCloudFiltered, output_filtered);
//     output_filtered.header = input_cloud_msg->header;
//     pub_filtered.publish(output_filtered);

//     sensor_msgs::PointCloud2 output_removed;
//     pcl::toROSMsg(*removedCloud, output_removed);
//     output_removed.header = input_cloud_msg->header;
//     pub_removed.publish(output_removed);

//     sensor_msgs::PointCloud2 output_ground;
//     pcl::toROSMsg(*planeCloud, output_ground);
//     output_ground.header = input_cloud_msg->header;
//     pub_ground.publish(output_ground);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "pcl_processing_node");
//     ros::NodeHandle nh;

//     // Define the publishers
//     pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
//     pub_removed = nh.advertise<sensor_msgs::PointCloud2>("removed_points", 1);
//     pub_ground = nh.advertise<sensor_msgs::PointCloud2>("ground_points", 1);

//     // Subscribe to the input point cloud topic
//     ros::Subscriber sub = nh.subscribe("/mbuggy/os2/points", 1, cloudCallback);

//     ros::spin();
//     return 0;
// }

