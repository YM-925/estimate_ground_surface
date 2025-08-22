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
    ransac_max_iterations = 2000;     // 最大反復回数
    ransac_probability = 0.99;        // 成功確率

    // X分割RANSACのパラメータ
    x_division_boundary = get_parameter("x_division.boundary").as_double();
    min_points_per_division = get_parameter("x_division.min_points").as_int();
    plane_angle_threshold = get_parameter("x_division.plane_angle_threshold").as_double();

    plane_distance_threshold = get_parameter("x_division.plane_distance_threshold").as_double();

    // パラメータファイルから取得できる場合
    try
    {
        min_points_per_division = get_parameter("x_division.min_points").as_int();
    }
    catch (...)
    {
        // パラメータが存在しない場合はデフォルト値を使用
    }

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
    detectAndColorPlaneWithXBoundary(cropped_cloud, colored_cloud);

    sensor_msgs::msg::PointCloud2 colored_msg;
    pcl::toROSMsg(*colored_cloud, colored_msg);
    colored_msg.header.frame_id = msg.header.frame_id;
    colored_msg.header.stamp = msg.header.stamp;
    point_publisher_->publish(colored_msg);
}

void ObstacleDetection::detectAndColorPlaneWithXBoundary(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud)
{
    // 元の点群をコピー
    *colored_cloud = *input_cloud;

    // 点群を境界で前後に分割
    std::vector<int> front_indices; // X < boundary の点群
    std::vector<int> back_indices;  // X >= boundary の点群

    for (int i = 0; i < input_cloud->points.size(); i++)
    {
        const auto &point = input_cloud->points[i];
        if (point.x < x_division_boundary)
        {
            front_indices.push_back(i);
        }
        else
        {
            back_indices.push_back(i);
        }
    }

    RCLCPP_INFO(this->get_logger(), "X boundary: %.2f", x_division_boundary);
    RCLCPP_INFO(this->get_logger(), "Front section (X < %.2f): %zu points",
                x_division_boundary, front_indices.size());
    RCLCPP_INFO(this->get_logger(), "Back section (X >= %.2f): %zu points",
                x_division_boundary, back_indices.size());

    // 全ての点をまず灰色（非平面）に設定
    for (auto &point : colored_cloud->points)
    {
        point.r = 128;
        point.g = 128;
        point.b = 128;
    }

    // 平面の係数を格納する変数
    pcl::ModelCoefficients::Ptr front_coefficients(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr back_coefficients(new pcl::ModelCoefficients);
    bool front_plane_detected = false;
    bool back_plane_detected = false;

    // 前方セクション（X < boundary）でRANSAC実行
    if (front_indices.size() >= min_points_per_division)
    {
        front_plane_detected = processSectionWithCoefficients(input_cloud, front_indices, colored_cloud,
                                                              "Front", 0, 100, 0, front_coefficients, nullptr);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Front section has too few points (%zu < %d), skipping",
                    front_indices.size(), min_points_per_division);
    }

    // 後方セクション（X >= boundary）でRANSAC実行
    std::vector<int> back_plane_indices;
    if (back_indices.size() >= min_points_per_division)
    {
        back_plane_detected = processSectionWithCoefficients(input_cloud, back_indices, colored_cloud,
                                                             "Back", 0, 255, 0, back_coefficients, &back_plane_indices);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Back section has too few points (%zu < %d), skipping",
                    back_indices.size(), min_points_per_division);
    }

    // 両方の平面が検出された場合、角度と距離を確認して必要に応じて調整
    if (front_plane_detected && back_plane_detected)
    {
        double angle = calculatePlaneAngle(front_coefficients, back_coefficients);
        RCLCPP_INFO(this->get_logger(), "Angle between front and back planes: %.2f degrees", angle);
        double distance = calculatePlaneDistanceAtBoundary(front_coefficients, back_coefficients);

        if (angle > plane_angle_threshold || distance > plane_distance_threshold)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Condition met - Angle: %.2f° (threshold: %.2f°), Distance: %.3f m (threshold: %.3f m)",
                        angle, plane_angle_threshold, distance, plane_distance_threshold);
            RCLCPP_INFO(this->get_logger(), "Aligning back plane to front plane");

            for (const int &idx : back_plane_indices)
            {
                colored_cloud->points[idx].r = 128;
                colored_cloud->points[idx].g = 128;
                colored_cloud->points[idx].b = 128;
            }

            // 後方平面を前方平面に合わせて再着色
            alignBackPlaneToFront(input_cloud, back_indices, colored_cloud, front_coefficients);
        }
    }
}

bool ObstacleDetection::processSectionWithCoefficients(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                                       const std::vector<int> &section_indices,
                                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
                                                       const std::string &section_name,
                                                       uint8_t r, uint8_t g, uint8_t b,
                                                       pcl::ModelCoefficients::Ptr &coefficients,
                                                       std::vector<int> *plane_indices)
{
    // セクションの点群を作成
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr section_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const int &idx : section_indices)
    {
        section_cloud->points.push_back(input_cloud->points[idx]);
    }
    section_cloud->width = section_cloud->points.size();
    section_cloud->height = 1;
    section_cloud->is_dense = false;

    // RANSACセグメンテーションの設定
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_max_iterations);
    seg.setDistanceThreshold(ransac_distance_threshold);
    seg.setProbability(ransac_probability);
    seg.setInputCloud(section_cloud);

    // 平面検出を実行
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "No plane detected in %s section", section_name.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "%s section: Plane detected with %zu inliers",
                section_name.c_str(), inliers->indices.size());

    // 検出された平面の点を指定色で着色
    for (const auto &inlier_idx : inliers->indices)
    {
        int original_idx = section_indices[inlier_idx];
        colored_cloud->points[original_idx].r = r;
        colored_cloud->points[original_idx].g = g;
        colored_cloud->points[original_idx].b = b;
    }

    // 平面として着色された点のインデックスを記録（plane_indicesが指定されている場合のみ）
    if (plane_indices != nullptr)
    {
        for (const auto &inlier_idx : inliers->indices)
        {
            int original_idx = section_indices[inlier_idx];
            plane_indices->push_back(original_idx);
        }
    }

    // 平面の係数をログ出力
    RCLCPP_INFO(this->get_logger(), "%s section plane coefficients: a=%f, b=%f, c=%f, d=%f",
                section_name.c_str(), coefficients->values[0], coefficients->values[1],
                coefficients->values[2], coefficients->values[3]);

    return true;
}

double ObstacleDetection::calculatePlaneAngle(const pcl::ModelCoefficients::Ptr &plane1,
                                              const pcl::ModelCoefficients::Ptr &plane2)
{
    // 平面の法線ベクトルを取得
    Eigen::Vector3f normal1(plane1->values[0], plane1->values[1], plane1->values[2]);
    Eigen::Vector3f normal2(plane2->values[0], plane2->values[1], plane2->values[2]);

    // 法線ベクトルを正規化
    normal1.normalize();
    normal2.normalize();

    // 内積を計算（コサイン値）
    float cos_angle = normal1.dot(normal2);

    // コサイン値を-1から1の範囲に制限
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));

    // 角度を計算（ラジアンから度に変換）
    float angle_rad = std::acos(std::abs(cos_angle)); // 絶対値を取って鋭角を得る
    float angle_deg = angle_rad * 180.0f / M_PI;

    return static_cast<double>(angle_deg);
}

void ObstacleDetection::alignBackPlaneToFront(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
                                              const std::vector<int> &back_indices,
                                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud,
                                              const pcl::ModelCoefficients::Ptr &front_coefficients)
{
    // 前方平面の係数
    float a = front_coefficients->values[0];
    float b = front_coefficients->values[1];
    float c = front_coefficients->values[2];
    float d = front_coefficients->values[3];

    RCLCPP_INFO(this->get_logger(), "Aligning back section points to front plane (a=%f, b=%f, c=%f, d=%f)",
                a, b, c, d);

    int aligned_count = 0;

    // 後方セクションの各点について、前方平面からの距離を計算
    for (const int &idx : back_indices)
    {
        const auto &point = input_cloud->points[idx];

        // 点から平面までの距離を計算
        float distance = std::abs(a * point.x + b * point.y + c * point.z + d) /
                         std::sqrt(a * a + b * b + c * c);

        // 距離が閾値以下の場合、平面の点として着色
        if (distance <= ransac_distance_threshold)
        {
            colored_cloud->points[idx].r = 0; // 青色で着色（前方平面に合わせた後方点）
            colored_cloud->points[idx].g = 100;
            colored_cloud->points[idx].b = 0;
            aligned_count++;
        }
        // 距離が閾値を超える場合は灰色のまま（非平面点）
    }

    // RCLCPP_INFO(this->get_logger(), "Aligned %d back section points to front plane", aligned_count);
}

double ObstacleDetection::calculatePlaneDistanceAtBoundary(const pcl::ModelCoefficients::Ptr &front_plane,
                                                           const pcl::ModelCoefficients::Ptr &back_plane)
{
    // 前方平面の係数
    float a1 = front_plane->values[0];
    float b1 = front_plane->values[1];
    float c1 = front_plane->values[2];
    float d1 = front_plane->values[3];

    // 後方平面の係数
    float a2 = back_plane->values[0];
    float b2 = back_plane->values[1];
    float c2 = back_plane->values[2];
    float d2 = back_plane->values[3];

    // X境界における平面の高さ（Z座標）を計算
    // 平面方程式: ax + by + cz + d = 0 から z = -(ax + by + d) / c
    // Y=0（中央線）での高さを計算

    float y_center = 0.0f; // Y座標の中央値（必要に応じて調整）

    // 前方平面のX境界での高さ
    float z1 = -(a1 * x_division_boundary + b1 * y_center + d1) / c1;

    // 後方平面のX境界での高さ
    float z2 = -(a2 * x_division_boundary + b2 * y_center + d2) / c2;

    // 2つの平面間の距離（高さの差の絶対値）
    double distance = std::abs(z2 - z1);

    return distance;
}