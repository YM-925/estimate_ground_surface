#include <boost/container/vector.hpp>
#include <estimate_ground_surface.hpp>
#include <functional>

using std::placeholders::_1;

ObstacleDetection::ObstacleDetection(const rclcpp::NodeOptions &options) // コンストラクタ
    : Node("estimate_ground_surface", options),                               // ノード名
      tf_buffer_(this->get_clock()),                                     // TF2バッファ
      tf_listener_(tf_buffer_)                                           // TF2リスナー
{
    // 使用するパラメータの宣言

    // LiDARとIMUの誤差を補正するための変数
    roll_diff = get_parameter("rotation_angle.roll_difference").as_double();
    pitch_diff = get_parameter("rotation_angle.pitch_difference").as_double();

    // LiDARの高さを示す変数
    height = get_parameter("lidar_height.z_offset").as_double();

    // 点群の範囲を決定する変数
    minX = get_parameter("crop_box.min.x").as_double();
    minY = get_parameter("crop_box.min.y").as_double();
    minZ = get_parameter("crop_box.min.z").as_double();
    minA = get_parameter("crop_box.min.a").as_double();
    maxX = get_parameter("crop_box.max.x").as_double();
    maxY = get_parameter("crop_box.max.y").as_double();
    maxZ = get_parameter("crop_box.max.z").as_double();
    maxA = get_parameter("crop_box.max.a").as_double();

    // グリッドに関する変数
    grid_min_x = get_parameter("grid.min_x").as_double();
    grid_max_x = get_parameter("grid.max_x").as_double();
    grid_min_y = get_parameter("grid.min_y").as_double();
    grid_max_y = get_parameter("grid.max_y").as_double();
    grid_resolution = get_parameter("grid.resolution").as_double();
    grid_variance = get_parameter("grid.variance").as_double();
    grid_cluster_size = get_parameter("grid.cluster_size").as_int();
    z_threshold_low = get_parameter("grid.z_threshold_low").as_double();
    z_threshold_high = get_parameter("grid.z_threshold_high").as_double();
    step_ratio_threshold = get_parameter("grid.ratio_threshold").as_double();

    // RANSACパラメータ（デフォルト値を設定、必要に応じてパラメータファイルから取得可能）
    ransac_distance_threshold = 0.02;  // 平面からの距離閾値（m）
    ransac_max_iterations = 1000;      // 最大反復回数
    ransac_probability = 0.99;         // 成功確率

    // サブスクライバーとパブリッシャー
    _lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&ObstacleDetection::topic_callback, this, _1));
    point_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_processing_result", 10);
    plane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/detected_planes", 10);
    obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker", 10);
    // _imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", 10, std::bind(&ObstacleDetection::imu_topic_callback, this, _1));
}

// RANSACによる平面検出関数
void ObstacleDetection::detectPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obstacle_cloud,
                                    Eigen::Vector4f& plane_coefficients)
{
    // RANSACセグメンテーションの設定
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations);
    seg.setDistanceThreshold(ransac_distance_threshold);
    seg.setProbability(ransac_probability);
    seg.setInputCloud(input_cloud);

    // 平面検出を実行
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
        return;
    }

    // 平面の係数を保存
    plane_coefficients << coefficients->values[0], 
                          coefficients->values[1], 
                          coefficients->values[2], 
                          coefficients->values[3];

    RCLCPP_INFO(this->get_logger(), "Plane coefficients: a=%f, b=%f, c=%f, d=%f", 
                plane_coefficients[0], plane_coefficients[1], 
                plane_coefficients[2], plane_coefficients[3]);
    
    RCLCPP_INFO(this->get_logger(), "Plane inliers: %zu points", inliers->indices.size());

    // 平面点群とそれ以外（障害物）の点群を分離
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);

    // 平面の点群を抽出
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    // 平面以外（障害物）の点群を抽出
    extract.setNegative(true);
    extract.filter(*obstacle_cloud);

    // 平面点群を緑色に着色
    for (auto& point : plane_cloud->points)
    {
        point.r = 0;
        point.g = 255;
        point.b = 0;
    }

    // 障害物点群を赤色に着色
    for (auto& point : obstacle_cloud->points)
    {
        point.r = 255;
        point.g = 0;
        point.b = 0;
    }
}

// 平面の可視化マーカーを作成
visualization_msgs::msg::Marker ObstacleDetection::createPlaneMarker(const Eigen::Vector4f& coefficients,
                                                                    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "detected_plane";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 平面の法線ベクトル
    Eigen::Vector3f normal(coefficients[0], coefficients[1], coefficients[2]);
    normal.normalize();

    // 平面上の点を計算（原点から平面への最短距離の点）
    float d = -coefficients[3];
    Eigen::Vector3f plane_point = normal * d;

    // マーカーの位置設定
    marker.pose.position.x = plane_point.x();
    marker.pose.position.y = plane_point.y();
    marker.pose.position.z = plane_point.z();

    // 平面の向きを設定（法線ベクトルからクォータニオンを計算）
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f rotation_axis = z_axis.cross(normal);
    float rotation_angle = acos(z_axis.dot(normal));

    if (rotation_axis.norm() > 1e-6)
    {
        rotation_axis.normalize();
        Eigen::AngleAxisf rotation(rotation_angle, rotation_axis);
        Eigen::Quaternionf quat(rotation);
        
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
    }
    else
    {
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    }

    // マーカーのサイズ（平面を表現する薄い立方体）
    marker.scale.x = 5.0;  // 幅
    marker.scale.y = 5.0;  // 高さ
    marker.scale.z = 0.01; // 厚み（薄くして平面らしく）

    // マーカーの色（半透明の青）
    marker.color.a = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}

// コールバック関数
void ObstacleDetection::topic_callback(const sensor_msgs::msg::PointCloud2 &msg)
{
    grid_map.clear();

    // ポイントクラウド格納
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // RANSACによる平面検出結果用の点群
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector4f plane_coefficients;

    // ROSからPCLに変換
    pcl::fromROSMsg(msg, *raw_cloud);

    // pcl::PointXYZ型からpcl::PointXYZRGB型への変換
    pcl::copyPointCloud(*raw_cloud, *raw_cloud_rgb);

    // Voxel Gridフィルタの設定（コメントアウトされているが、必要に応じて使用可能）
    // pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
    // voxelGrid.setInputCloud(raw_cloud_rgb);
    // voxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);
    // voxelGrid.filter(*raw_cloud_rgb);

    // CropBoxによる切り取り
    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.setInputCloud(raw_cloud_rgb);
    crop.filter(*cropped_cloud);

    if (cropped_cloud->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cropped cloud is empty. Skipping processing.");
        return;
    }

    // == 傾き補正 ==
    // 回転行列の作成（XY平面に整列）
    Eigen::Affine3f transform_roll_pitch = Eigen::Affine3f::Identity();
    transform_roll_pitch.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    transform_roll_pitch.rotate(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()));

    // 全点に対して回転と高さオフセットを適用
    for (auto &point : cropped_cloud->points)
    {
        // 回転と高さオフセットを適用
        Eigen::Vector3f original_point(point.x, point.y, point.z);
        Eigen::Vector3f transformed_point = transform_roll_pitch * original_point;

        point.x = transformed_point.x();
        point.y = transformed_point.y();
        point.z = transformed_point.z() + height;
    }

    // RANSACによる平面検出を実行
    detectPlanes(cropped_cloud, plane_cloud, obstacle_cloud, plane_coefficients);

    // 結果を発行
    sensor_msgs::msg::PointCloud2 processed_msg, plane_msg, obstacle_msg;
    
    // 処理済み点群（全体）を発行
    pcl::toROSMsg(*cropped_cloud, processed_msg);
    processed_msg.header.frame_id = msg.header.frame_id;
    processed_msg.header.stamp = msg.header.stamp;
    point_publisher_->publish(processed_msg);

    // 検出された平面点群を発行
    if (!plane_cloud->empty())
    {
        pcl::toROSMsg(*plane_cloud, plane_msg);
        plane_msg.header.frame_id = msg.header.frame_id;
        plane_msg.header.stamp = msg.header.stamp;
        plane_publisher_->publish(plane_msg);
    }

    // 障害物点群を発行
    if (!obstacle_cloud->empty())
    {
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header.frame_id = msg.header.frame_id;
        obstacle_msg.header.stamp = msg.header.stamp;
        obstacle_publisher_->publish(obstacle_msg);
    }

    // 平面の可視化マーカーを発行
    if (!plane_cloud->empty())
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker plane_marker = createPlaneMarker(plane_coefficients, msg.header.frame_id);
        marker_array.markers.push_back(plane_marker);
        marker_publisher_->publish(marker_array);
    }
}