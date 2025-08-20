#include <boost/container/vector.hpp>
#include <estimate_ground_surface.hpp>
#include <functional>

using std::placeholders::_1;

ObstacleDetection::ObstacleDetection(const rclcpp::NodeOptions &options) // コンストラクタ
    : Node("estimate_ground_surface", options),                          // ノード名
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
    ransac_distance_threshold = 0.05; // 平面からの距離閾値（m）
    ransac_max_iterations = 2000;    // 最大反復回数
    ransac_probability = 0.99;       // 成功確率

    // サブスクライバーとパブリッシャー
    _lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar", 10, std::bind(&ObstacleDetection::topic_callback, this, _1));
    point_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/colored_pointcloud", 10);
    // _imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu", 10, std::bind(&ObstacleDetection::imu_topic_callback, this, _1));
}

// コールバック関数
void ObstacleDetection::topic_callback(const sensor_msgs::msg::PointCloud2 &msg)
{
    grid_map.clear();

    // ポイントクラウド格納
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

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
    detectAndColorPlane(cropped_cloud, colored_cloud);

    sensor_msgs::msg::PointCloud2 colored_msg;
    pcl::toROSMsg(*colored_cloud, colored_msg);
    colored_msg.header.frame_id = msg.header.frame_id;
    colored_msg.header.stamp = msg.header.stamp;
    point_publisher_->publish(colored_msg);
}

void ObstacleDetection::detectAndColorPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud)
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

    // 元の点群をコピー
    *colored_cloud = *input_cloud;

    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No plane detected. All points remain original color.");
        // 平面が見つからない場合、全ての点を白色にする
        for (auto &point : colored_cloud->points)
        {
            point.r = 255;
            point.g = 255;
            point.b = 255;
        }
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Plane detected with %zu inliers", inliers->indices.size());

    // 全ての点をまず灰色（非平面）に設定
    for (auto &point : colored_cloud->points)
    {
        point.r = 128;
        point.g = 128;
        point.b = 128;
    }

    // 検出された平面の点を緑色に着色
    for (const auto &index : inliers->indices)
    {
        colored_cloud->points[index].r = 0;   // 赤成分
        colored_cloud->points[index].g = 255; // 緑成分
        colored_cloud->points[index].b = 0;   // 青成分
    }

    // 平面の係数をログ出力
    RCLCPP_INFO(this->get_logger(), "Plane coefficients: a=%f, b=%f, c=%f, d=%f",
                coefficients->values[0], coefficients->values[1],
                coefficients->values[2], coefficients->values[3]);
}
