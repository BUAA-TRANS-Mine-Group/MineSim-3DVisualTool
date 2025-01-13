### 根据 `width_ego`、`length_ego` 和 `height_ego` 动态配置 EGO 车辆模型的尺寸

要在 RViz 中根据 `width_ego`、`length_ego` 和 `height_ego` 动态调整 EGO 车辆的 3D 模型尺寸，您需要正确设置 `visualization_msgs::Marker` 中的 `scale` 属性。以下是实现这一目标的详细步骤和代码示例。

#### 1. 理解 `scale` 属性

`scale` 属性在 `visualization_msgs::Marker` 中用于缩放模型。它是一个比例因子，按 X、Y 和 Z 轴分别缩放模型的尺寸。因此，`scale.x`、`scale.y` 和 `scale.z` 分别对应模型在每个轴上的缩放比例。

#### 2. 确定原始模型尺寸

为了正确缩放模型，使其在 RViz 中显示为实际车辆尺寸，您需要知道 3D 模型的原始尺寸（即模型在没有缩放时的长度、宽度和高度）。这通常可以通过以下几种方式获得：

- **查看模型文件**：打开 `.obj` 文件或其他 3D 模型文件，查看顶点坐标以确定模型的尺寸。
- **文档或元数据**：查看模型的文档或相关元数据，可能会提供模型的实际尺寸。
- **测量工具**：使用 3D 建模工具（如 Blender）导入模型并测量其尺寸。

假设您已经知道模型的原始尺寸：

```cpp
const double ORIGINAL_LENGTH = 13.4; // 模型原始长度（单位：米）
const double ORIGINAL_WIDTH  = 6.7;  // 模型原始宽度（单位：米）
const double ORIGINAL_HEIGHT = 6.9;  // 模型原始高度（单位：米）
```

// # === 模型信息 ===
// # 顶点数量: 81166
// # 面数量: 81008
// # 长度 (X 轴) : 1169.141 米
// # 宽度 (Y 轴) : 997.701 米
// # 高度 (Z 轴) : 740.219 米

> **注意**：请根据您的模型实际尺寸进行调整。

#### 3. 计算缩放比例

根据车辆的实际尺寸和模型的原始尺寸，计算缩放比例：

```cpp
double scale_x = length_ego / ORIGINAL_LENGTH;
double scale_y = width_ego / ORIGINAL_WIDTH;
double scale_z = height_ego / ORIGINAL_HEIGHT;
```

#### 4. 修改 EGO 车辆的 Marker 配置

将计算得到的缩放比例应用到 EGO 车辆的 Marker 中：

```cpp
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

// 假设您已经定义了 Vehicle 结构体和其他必要的代码

int main(int argc, char **argv) {
    ros::init(argc, argv, "file2enu");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // Publisher 定义
    ros::Publisher pubVehicle0 = nh_private.advertise<visualization_msgs::Marker>("vehicle_model_ego", 1, true);

    // 读取配置和车辆尺寸
    double width_ego, length_ego, height_ego;
    // ... 读取 width_ego, length_ego, height_ego 的代码 ...

    // 定义原始模型尺寸（根据实际模型调整）
    const double ORIGINAL_LENGTH = 13.4; // 米
    const double ORIGINAL_WIDTH  = 6.7;  // 米
    const double ORIGINAL_HEIGHT = 6.9;  // 米

    // 计算缩放比例
    double scale_x = length_ego / ORIGINAL_LENGTH;
    double scale_y = width_ego / ORIGINAL_WIDTH;
    double scale_z = height_ego / ORIGINAL_HEIGHT;

    // 创建 Marker
    visualization_msgs::Marker vehicle_marker_0_;
    vehicle_marker_0_.header.frame_id = frame_id;
    vehicle_marker_0_.header.stamp = current_time;
    vehicle_marker_0_.ns = "vehicle";
    vehicle_marker_0_.id = 0;
    vehicle_marker_0_.action = visualization_msgs::Marker::ADD;
    vehicle_marker_0_.type = visualization_msgs::Marker::MESH_RESOURCE;

    // 设置 mesh 资源路径
    string mesh_resource_0 = "package://common/materials/vehicle_model/Mining_Truck_v1_L1.123c6431771f-f361-4a1f-8727-590a5c800ce0/16747_Mining_Truck_v1.obj";
    vehicle_marker_0_.mesh_resource = mesh_resource_0;

    // 设置缩放比例
    vehicle_marker_0_.scale.x = scale_x;
    vehicle_marker_0_.scale.y = scale_y;
    vehicle_marker_0_.scale.z = scale_z;

    // 设置颜色（根据需要）
    vehicle_marker_0_.color.r = 0.7;
    vehicle_marker_0_.color.g = 0.66;
    vehicle_marker_0_.color.b = 0.66;
    vehicle_marker_0_.color.a = 0.99;

    // 设置位置
    vehicle_marker_0_.pose.position.x = x_ego;
    vehicle_marker_0_.pose.position.y = y_ego;
    vehicle_marker_0_.pose.position.z = alt_ego + (height_ego / 2.0); // 模型中心在底部上方一半高度

    // 设置方向
    geometry_msgs::Quaternion tf_vehicle_marker_0 = tf::createQuaternionMsgFromYaw(corAngle_ego);
    vehicle_marker_0_.pose.orientation = tf_vehicle_marker_0;

    // 设置生命周期（永久显示）
    vehicle_marker_0_.lifetime = ros::Duration();

    // 发布 Marker
    pubVehicle0.publish(vehicle_marker_0_);

    ros::spin();
    return 0;
}
```

#### 5. 完整优化后的代码示例

结合您之前的需求和优化要求，以下是一个完整的优化示例，包括动态处理任意数量的车辆和根据尺寸动态调整模型缩放：

```cpp
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <boost/filesystem.hpp>
#include <cmath>
#include <cstdlib>
#include <flann/flann.hpp>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <gps/geodetic_conv.hpp>
#include <gps/json.hpp>
#include <iomanip>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <view_controller_msgs/CameraPlacement.h>

#include "common/basics/basics.h"
#include "common/basics/colormap.h"

using namespace std;
using Json = nlohmann::json; // 解析 json 文件

#define PI 3.141592

// 检查文件是否存在
bool fileExists(const std::string &path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

// 读取并解析配置文件以获取参数
void readConfig(bool &altInfoVisualization,  // 是否启用可视化（alternative info visualization）
               double &x_eye,               // 相机初始位置x
               double &y_eye,               // 相机初始位置y
               double &z_eye,               // 相机初始位置z
               double &x_foc,               // 相机焦点位置x
               double &y_foc,               // 相机焦点位置y
               double &z_foc,               // 相机焦点位置z
               double &update_v,            // 更新速度
               const string &config_file) { // 配置文件路径
    // 检查文件路径是否存在
    if (!fileExists(config_file)) {
        std::cerr << "#log# Error: Unable to open config file: " << config_file << std::endl;
        throw runtime_error("#log# 配置文件路径无效或文件不存在: " + config_file);
    }

    // 打开配置文件
    std::fstream fs(config_file);
    if (!fs.is_open()) {
        ROS_ERROR_STREAM(std::string("#log# Failed to open config_file: ") + config_file);
        throw runtime_error("#log# config_file open failed.");
    }

    // 读取文件内容
    Json jsonData;
    try {
        fs >> jsonData;
    } catch (const Json::parse_error &e) {
        ROS_ERROR_STREAM(std::string("#log# Error to parse error config_file: ") + config_file);
        throw;
    }

    // 解析字段: alt_info_visualization
    if (jsonData.contains("alt_info_visualization")) {
        if (jsonData["alt_info_visualization"].is_boolean()) {
            altInfoVisualization = jsonData["alt_info_visualization"].get<bool>();
        } else if (jsonData["alt_info_visualization"].is_string()) {
            string valueStr = jsonData["alt_info_visualization"];
            altInfoVisualization = (valueStr == "true");
        } else {
            cerr << "#log# alt_info_visualization字段类型错误" << endl;
            throw runtime_error("#log# 配置文件格式错误");
        }
    } else {
        cerr << "#log# 缺少alt_info_visualization字段" << endl;
        throw runtime_error("#log# 配置文件缺少必要字段");
    }

    // 解析相机参数：eye_position和focus_position
    try {
        x_eye = jsonData["eye_position"]["x"].get<double>();
        y_eye = jsonData["eye_position"]["y"].get<double>();
        z_eye = jsonData["eye_position"]["z"].get<double>();
        x_foc = jsonData["focus_position"]["x"].get<double>();
        y_foc = jsonData["focus_position"]["y"].get<double>();
        z_foc = jsonData["focus_position"]["z"].get<double>();
    } catch (const Json::out_of_range &e) {
        cerr << "#log# 解析eye_position或focus_position字段时出错: " << e.what() << endl;
        throw;
    }

    // 解析更新速度：update_v
    if (jsonData.contains("update_v")) {
        try {
            update_v = jsonData["update_v"].get<double>();
        } catch (...) {
            cerr << "#log# 解析update_v字段时出错" << endl;
            throw runtime_error("update_v字段格式错误");
        }
    } else {
        cerr << "#log# 缺少update_v字段" << endl;
        throw runtime_error("配置文件缺少必要字段");
    }
}

// 修正角度
double CorrectionAngle(double log_yaw_rad, double log_HeadingAngle_start) {
    double to_start_HeadingAngle = log_yaw_rad;
    // 修正至 [-PI, PI]
    while (to_start_HeadingAngle > PI) to_start_HeadingAngle -= 2 * PI;
    while (to_start_HeadingAngle < -PI) to_start_HeadingAngle += 2 * PI;
    to_start_HeadingAngle += PI;
    return to_start_HeadingAngle;
}

// 读取车辆信息并返回一个包含所有车辆数据的结构
struct Vehicle {
    string id;
    double x;
    double y;
    double yaw_rad;
    double v_mps;
    double acc_mpss;
    double width;
    double length;
    double height;
    double alt;
    double pitch_angle;
    string vehicle_type;
};

// 解析车辆信息
vector<Vehicle> parseVehicleInfo(const Json& vehicle_info_json) {
    vector<Vehicle> vehicles;
    for (auto it = vehicle_info_json.begin(); it != vehicle_info_json.end(); ++it) {
        string key = it.key();
        if (key == "ego") continue; // 跳过 ego 车辆

        Vehicle veh;
        veh.id = key;
        veh.x = it.value()["x"].get<double>();
        veh.y = it.value()["y"].get<double>();
        veh.yaw_rad = it.value()["yaw_rad"].get<double>();
        veh.v_mps = it.value()["v_mps"].get<double>();
        veh.acc_mpss = it.value()["acc_mpss"].get<double>();
        veh.width = it.value()["shape"]["width"].get<double>();
        veh.length = it.value()["shape"]["length"].get<double>();
        veh.height = it.value()["shape"]["height"].get<double>();
        veh.alt = 0; // 根据需要设置
        veh.pitch_angle = it.value()["yawrate_radps"].get<double>();
        veh.vehicle_type = it.value()["shape"]["vehicle_type"].get<string>();
        vehicles.push_back(veh);
    }
    return vehicles;
}

// 创建车辆盒子标记
visualization_msgs::Marker createVehicleMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map,
                                               const std::map<std::string, Vec3f>& scale_map,
                                               double ORIGINAL_LENGTH, double ORIGINAL_WIDTH, double ORIGINAL_HEIGHT,
                                               const string& frame_id, double corAngle) {
    visualization_msgs::Marker veh_marker;
    veh_marker.header.frame_id = frame_id;
    veh_marker.header.stamp = ros::Time::now();
    veh_marker.ns = "vehicles";
    veh_marker.id = id;
    veh_marker.type = visualization_msgs::Marker::CUBE;
    veh_marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色
    if (veh.width > 6.5) { // 例如，宽度大于6.5为 NTE200
        veh_marker.color = color_map.at("surrounding_truck_NTE200");
    } else if (veh.width > 4.0) { // 宽度大于4.0为 NTE240
        veh_marker.color = color_map.at("surrounding_truck_NTE240");
    } else { // 其他为 Pickup
        veh_marker.color = color_map.at("surrounding_car_pickup");
    }

    // 计算缩放比例
    double scale_x = veh.length / ORIGINAL_LENGTH;
    double scale_y = veh.width / ORIGINAL_WIDTH;
    double scale_z = veh.height / ORIGINAL_HEIGHT;

    // 设置缩放
    veh_marker.scale.x = scale_x;
    veh_marker.scale.y = scale_y;
    veh_marker.scale.z = scale_z;

    // 设置位置
    veh_marker.pose.position.x = veh.x;
    veh_marker.pose.position.y = veh.y;
    veh_marker.pose.position.z = veh.alt + (veh.height / 2.0); // 盒子中心在底部上方一半高度

    // 设置方向
    veh_marker.pose.orientation = tf::createQuaternionMsgFromYaw(corAngle);

    // 设置生命周期
    veh_marker.lifetime = ros::Duration();

    return veh_marker;
}

// 创建文本标记（ID + 速度）
visualization_msgs::Marker createTextMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map,
                                           const string& frame_id) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "vehicle_texts";
    text_marker.id = id;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;

    // 设置文本内容
    std::ostringstream text_ss;
    text_ss << "ID:" << veh.id << " V:" << fixed << setprecision(1) << veh.v_mps;
    text_marker.text = text_ss.str();

    // 设置缩放（字体大小）
    text_marker.scale.x = 1.0;
    text_marker.scale.y = 1.0;
    text_marker.scale.z = 1.0;

    // 设置颜色
    text_marker.color = color_map.at("surrounding_car_truck_text");

    // 设置位置
    text_marker.pose.position.x = veh.x;
    text_marker.pose.position.y = veh.y;
    text_marker.pose.position.z = veh.alt + veh.height + 0.5; // 文本位于车辆上方

    // 设置方向
    text_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // 设置生命周期
    text_marker.lifetime = ros::Duration();

    return text_marker;
}

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "file2enu");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // Publisher 定义
    ros::Publisher pubVehicle0 = nh_private.advertise<visualization_msgs::Marker>("vehicle_model_ego", 1, true);
    ros::Publisher vehicle_markers_pub = nh_private.advertise<visualization_msgs::MarkerArray>("vehicle_markers", 1, true);
    ros::Publisher path_pub = nh_private.advertise<nav_msgs::Path>("/track_path", 1, true);
    ros::Publisher ego_pathArea_pub = nh_private.advertise<visualization_msgs::MarkerArray>("ego_pathArea", 1, true);
    ros::Publisher rviz_camera_pub = nh_private.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);

    // 读取参数
    string track_file, frame_id, link_path_file, map_file, config_file;
    nh_private.param<string>("track_file", track_file, "default_track_file.log");
    nh_private.param<string>("frame_id", frame_id, "/map");
    nh_private.param<string>("link_path_file", link_path_file, "default_link_path.json");
    nh_private.param<string>("map_file", map_file, "default_map.json");
    nh_private.param<string>("config_file", config_file, "default_config.json");

    // 读取配置文件
    bool altInfoVisualization = false;
    double x_eye, y_eye, z_eye, x_foc, y_foc, z_foc, update_v;
    readConfig(altInfoVisualization, x_eye, y_eye, z_eye, x_foc, y_foc, z_foc, update_v, config_file);

    // 打开日志文件
    ifstream inFile(track_file.c_str(), ios::in);
    if (!inFile) {
        ROS_ERROR_STREAM("#log# Unable to open track file: " << track_file);
        return -1;
    }
    cout << "打开轨迹文件成功!" << endl;

    // 定义颜色和缩放映射
    std::map<std::string, common::ColorARGB> color_map_minesim{
        {"ego_ref_path", common::ColorARGB(0.7, 0.0, 171.0 / 255.0, 134.0 / 255.0)},
        {"ego_mine_truck", common::ColorARGB(0.7, 0.66, 0.66, 0.66)},
        {"surrounding_car_pickup", common::ColorARGB(0.8, 0.0, 0.0, 0.0)},
        {"surrounding_truck_NTE240", common::ColorARGB(0.6, 0.0, 60.0 / 255.0, 1.0)},
        {"surrounding_truck_NTE200", common::ColorARGB(0.6, 0.0, 60.0 / 255.0, 1.0)},
        {"surrounding_car_truck_text", common::ColorARGB(1.0, 1.0, 0.0, 0.0)},
    };

    std::map<std::string, Vec3f> object_scale_map_minesim{
        {"ego_ref_path", Vec3f(1.2, 0.4, 0.2)},
        {"ego_mine_truck", Vec3f(0.1, 0.1, 0.1)},
        {"surrounding_car_pickup", Vec3f(0.6, 0.6, 1.2)},
        {"surrounding_truck_NTE240", Vec3f(0.6, 0.6, 1.2)},
        {"surrounding_truck_NTE200", Vec3f(0.6, 0.6, 1.2)},
        {"surrounding_car_truck_text", Vec3f(1.5, 1.5, 1.5)},
    };

    // 定义路径消息类型
    nav_msgs::Path track_path;
    track_path.header.stamp = ros::Time::now();
    track_path.header.frame_id = frame_id;

    // 初始化相机位置
    double update_eye_x = x_eye;
    double update_eye_y = y_eye;

    ros::Rate loop_rate(10); // 10 Hz

    // 定义 MarkerArray
    visualization_msgs::MarkerArray vehicle_markers;

    // 原始模型尺寸（根据实际模型调整）
    const double ORIGINAL_LENGTH = 13.4; // 米
    const double ORIGINAL_WIDTH  = 6.7;  // 米
    const double ORIGINAL_HEIGHT = 6.9;  // 米

    while (ros::ok() && getline(inFile, line_track_file)) {
        // 解析当前行的 JSON
        Json jsonData = Json::parse(line_track_file);

        // 提取时间
        double time_s_num = jsonData["time_s"].get<double>();
        std::ostringstream oss;
        oss.precision(2);
        oss << std::fixed << time_s_num;
        string abs_time = oss.str();
        printf("abs_time: %s\n", abs_time.c_str());

        // 提取主车信息
        double x_ego = jsonData["details"]["vehicle_info"]["ego"]["x"].get<double>();
        double y_ego = jsonData["details"]["vehicle_info"]["ego"]["y"].get<double>();
        double yaw_rad_ego = jsonData["details"]["vehicle_info"]["ego"]["yaw_rad"].get<double>();
        double v_mps_ego = jsonData["details"]["vehicle_info"]["ego"]["v_mps"].get<double>();
        double acc_mpss_ego = jsonData["details"]["vehicle_info"]["ego"]["acc_mpss"].get<double>();
        double width_ego = jsonData["details"]["vehicle_info"]["ego"]["shape"]["width"].get<double>();
        double length_ego = jsonData["details"]["vehicle_info"]["ego"]["shape"]["length"].get<double>();
        double height_ego = jsonData["details"]["vehicle_info"]["ego"]["shape"]["height"].get<double>();
        double PitchAngle_ego = jsonData["details"]["vehicle_info"]["ego"]["yawrate_radps"].get<double>();

        // 坐标转换和角度修正
        ros::Time current_time = ros::Time::now();
        double corAngle_ego = CorrectionAngle(yaw_rad_ego, 0.0); // 假设 log_HeadingAngle_start 为 0

        // 发布路径信息
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = x_ego;
        this_pose_stamped.pose.position.y = y_ego;
        this_pose_stamped.pose.position.z = 0.0; // 根据需要设置
        this_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(corAngle_ego);
        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = frame_id;
        track_path.header.stamp = current_time;
        track_path.poses.push_back(this_pose_stamped);
        path_pub.publish(track_path);

        // 解析其他车辆信息
        Json vehicle_info_json = jsonData["details"]["vehicle_info"];
        vector<Vehicle> vehicles = parseVehicleInfo(vehicle_info_json);

        // 清空之前的 MarkerArray
        vehicle_markers.markers.clear();

        // 生成车辆标记
        int marker_id = 0;
        for (const auto& veh : vehicles) {
            // 创建车辆盒子标记
            visualization_msgs::Marker veh_marker = createVehicleMarker(veh, marker_id, color_map_minesim, object_scale_map_minesim,
                                                                         ORIGINAL_LENGTH, ORIGINAL_WIDTH, ORIGINAL_HEIGHT,
                                                                         frame_id, CorrectionAngle(veh.yaw_rad, 0.0));
            vehicle_markers.markers.push_back(veh_marker);

            // 创建文本标记（ID + 速度）
            visualization_msgs::Marker text_marker = createTextMarker(veh, marker_id, color_map_minesim, frame_id);
            vehicle_markers.markers.push_back(text_marker);

            marker_id++;
        }

        // 发布所有车辆标记
        vehicle_markers_pub.publish(vehicle_markers);

        // 配置和发布 EGO 车辆的 3D 模型
        visualization_msgs::Marker vehicle_marker_0_;
        vehicle_marker_0_.header.frame_id = frame_id;
        vehicle_marker_0_.header.stamp = current_time;
        vehicle_marker_0_.ns = "vehicle";
        vehicle_marker_0_.id = 0;
        vehicle_marker_0_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_0_.type = visualization_msgs::Marker::MESH_RESOURCE;

        // 设置 mesh 资源路径
        string mesh_resource_0 = "package://common/materials/vehicle_model/Mining_Truck_v1_L1.123c6431771f-f361-4a1f-8727-590a5c800ce0/16747_Mining_Truck_v1.obj";
        vehicle_marker_0_.mesh_resource = mesh_resource_0;

        // 计算缩放比例
        double scale_x_ego = length_ego / ORIGINAL_LENGTH;
        double scale_y_ego = width_ego / ORIGINAL_WIDTH;
        double scale_z_ego = height_ego / ORIGINAL_HEIGHT;

        // 设置缩放比例
        vehicle_marker_0_.scale.x = scale_x_ego;
        vehicle_marker_0_.scale.y = scale_y_ego;
        vehicle_marker_0_.scale.z = scale_z_ego;

        // 设置颜色
        vehicle_marker_0_.color = color_map_minesim.at("ego_mine_truck");

        // 设置位置
        vehicle_marker_0_.pose.position.x = x_ego;
        vehicle_marker_0_.pose.position.y = y_ego;
        vehicle_marker_0_.pose.position.z = 0.0 + (height_ego / 2.0); // 根据需要调整

        // 设置方向
        vehicle_marker_0_.pose.orientation = tf::createQuaternionMsgFromYaw(corAngle_ego);

        // 设置生命周期（永久显示）
        vehicle_marker_0_.lifetime = ros::Duration();

        // 发布 Marker
        pubVehicle0.publish(vehicle_marker_0_);

        // 更新相机位置
        view_controller_msgs::CameraPlacement camera;
        geometry_msgs::Point pt_eye;
        pt_eye.x = update_eye_x;
        pt_eye.y = update_eye_y;
        pt_eye.z = z_eye;
        camera.eye.point = pt_eye;
        camera.eye.header.frame_id = "base_link";

        geometry_msgs::Point pt_foc;
        pt_foc.x = x_foc;
        pt_foc.y = y_foc;
        pt_foc.z = z_foc;
        camera.focus.point = pt_foc;
        camera.focus.header.frame_id = "base_link";

        // 设置相机的上方向
        geometry_msgs::Vector3 up;
        up.x = 0;
        up.y = 0;
        up.z = 10;
        camera.up.vector = up;
        camera.up.header.frame_id = "base_link";

        camera.time_from_start = ros::Duration();
        rviz_camera_pub.publish(camera);

        // 更新相机位置
        update_eye_x += update_v * cos(corAngle_ego + PI);
        update_eye_y += update_v * sin(corAngle_ego + PI);

        loop_rate.sleep();
        ros::spinOnce();
    }

    // 关闭打开的文件
    inFile.close();
    cout << "file2enu 停止!" << endl;
    return 0;
}
```

#### 6. 关键优化点解析

##### a. 动态计算缩放比例

通过已知的车辆实际尺寸和模型原始尺寸，动态计算每个轴的缩放比例，并将其应用到 Marker 的 `scale` 属性中。

```cpp
// 计算缩放比例
double scale_x_ego = length_ego / ORIGINAL_LENGTH;
double scale_y_ego = width_ego / ORIGINAL_WIDTH;
double scale_z_ego = height_ego / ORIGINAL_HEIGHT;

// 设置缩放比例
vehicle_marker_0_.scale.x = scale_x_ego;
vehicle_marker_0_.scale.y = scale_y_ego;
vehicle_marker_0_.scale.z = scale_z_ego;
```

##### b. 使用结构体管理车辆信息

定义 `Vehicle` 结构体来存储每辆车的相关信息，便于管理和扩展。

```cpp
struct Vehicle {
    string id;
    double x;
    double y;
    double yaw_rad;
    double v_mps;
    double acc_mpss;
    double width;
    double length;
    double height;
    double alt;
    double pitch_angle;
    string vehicle_type;
};
```

##### c. 封装标记创建逻辑

将创建车辆盒子标记和文本标记的逻辑封装为独立函数，提高代码可读性和可维护性。

```cpp
visualization_msgs::Marker createVehicleMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map,
                                               const std::map<std::string, Vec3f>& scale_map,
                                               double ORIGINAL_LENGTH, double ORIGINAL_WIDTH, double ORIGINAL_HEIGHT,
                                               const string& frame_id, double corAngle) {
    // ... 创建 Marker 逻辑 ...
}

visualization_msgs::Marker createTextMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map,
                                           const string& frame_id) {
    // ... 创建 Text Marker 逻辑 ...
}
```

##### d. 动态处理任意数量的车辆

通过遍历 JSON 中的 `vehicle_info`，自动处理非 `ego` 车辆的信息，并生成相应的标记。

```cpp
// 解析其他车辆信息
Json vehicle_info_json = jsonData["details"]["vehicle_info"];
vector<Vehicle> vehicles = parseVehicleInfo(vehicle_info_json);

// 生成车辆标记
int marker_id = 0;
for (const auto& veh : vehicles) {
    // 创建车辆盒子标记
    visualization_msgs::Marker veh_marker = createVehicleMarker(veh, marker_id, color_map_minesim, object_scale_map_minesim,
                                                                 ORIGINAL_LENGTH, ORIGINAL_WIDTH, ORIGINAL_HEIGHT,
                                                                 frame_id, CorrectionAngle(veh.yaw_rad, 0.0));
    vehicle_markers.markers.push_back(veh_marker);

    // 创建文本标记（ID + 速度）
    visualization_msgs::Marker text_marker = createTextMarker(veh, marker_id, color_map_minesim, frame_id);
    vehicle_markers.markers.push_back(text_marker);

    marker_id++;
}
```

#### 7. 注意事项

1. **确保模型原始尺寸准确**：缩放比例的准确性依赖于模型的原始尺寸。请确保 `ORIGINAL_LENGTH`、`ORIGINAL_WIDTH` 和 `ORIGINAL_HEIGHT` 正确反映了模型的实际尺寸。

2. **单位一致性**：确保车辆尺寸和模型尺寸的单位一致（通常为米）。

3. **模型中心位置**：在设置位置时，考虑模型的原点位置。通常，3D 模型的原点位于底部中心，但这取决于模型的设计。如果原点位置不同，可能需要调整 `pose.position.z` 以正确显示模型。

4. **性能优化**：如果有大量车辆，确保 MarkerArray 的大小不会影响 RViz 的性能。适时调整发布频率或优化标记的生命周期。

5. **错误处理**：在实际应用中，增加更多的错误处理和边界检查，以确保代码的鲁棒性。

#### 8. 总结

通过上述步骤，您可以根据 `width_ego`、`length_ego` 和 `height_ego` 动态调整 EGO 车辆的 3D 模型在 RViz 中的显示尺寸。关键在于正确计算缩放比例并应用到 Marker 的 `scale` 属性中，同时确保模型原始尺寸的准确性。封装和优化代码逻辑也有助于提高代码的可维护性和扩展性。

如果您有更多问题或需要进一步的帮助，请随时告知！
