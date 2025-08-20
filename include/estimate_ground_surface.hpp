#ifndef OBSTACLE_DETECTION_HPP
#define OBSTACLE_DETECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <map>

#include <Eigen/src/Eigenvalues/SelfAdjointEigenSolver.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class ObstacleDetection : public rclcpp::Node
{
public:
    explicit ObstacleDetection(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // コールバック関数
    void topic_callback(const sensor_msgs::msg::PointCloud2 &msg);

    // RANSACによる平面検出関数
    void detectAndColorPlaneWithXBoundary(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud);

    bool processSectionWithCoefficients(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                        const std::vector<int> &section_indices,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
                                        const std::string &section_name,
                                        uint8_t r, uint8_t g, uint8_t b,
                                        pcl::ModelCoefficients::Ptr &coefficients);

    double calculatePlaneAngle(const pcl::ModelCoefficients::Ptr &plane1,
                               const pcl::ModelCoefficients::Ptr &plane2);

    void alignBackPlaneToFront(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                               const std::vector<int> &back_indices,
                               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
                               const pcl::ModelCoefficients::Ptr &front_coefficients);

    // TF2関連
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // サブスクライバーとパブリッシャー
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _lidar_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_publisher_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    // パラメータ変数
    // LiDARとIMUの誤差補正
    double roll_diff;
    double pitch_diff;

    // LiDARの高さオフセット
    double height;

    // 点群切り取り範囲
    double minX, minY, minZ, minA;
    double maxX, maxY, maxZ, maxA;

    // グリッド関連
    double grid_min_x, grid_max_x;
    double grid_min_y, grid_max_y;
    double grid_resolution;
    double grid_variance;
    int grid_cluster_size;
    double z_threshold_low, z_threshold_high;
    double step_ratio_threshold;

    // RANSACパラメータ
    double ransac_distance_threshold; // 平面からの距離閾値
    int ransac_max_iterations;        // 最大反復回数
    double ransac_probability;        // 成功確率

    // X境界分割RANSACのパラメータ
    double x_division_boundary;  // 分割境界のX座標
    int min_points_per_division; // 各分割で最低必要な点数
    double plane_angle_threshold;   // 平面間角度閾値（度）

    // グリッドマップ
    std::map<std::pair<int, int>, std::vector<double>> grid_map;
};

#endif // OBSTACLE_DETECTION_HPP