### 1. 精简代码并优化车辆标记发布逻辑

为了满足您的需求，我们将对现有代码进行以下优化：

1. **去除冗余代码**：消除为每辆车单独定义的变量和发布器，转而使用循环和数据结构来动态处理任意数量的车辆。
2. **动态解析车辆信息**：从 JSON 中读取 `vehicle_info`，排除 `ego` 车辆，并根据车辆数量动态生成标记。
3. **使用 `MarkerArray` 批量发布标记**：减少发布器的数量，提高代码的可维护性和效率。

### 2. 优化后的代码示例

以下是优化后的代码示例，涵盖了上述要求：

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

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "file2enu");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // 注册Publisher到话题
    ros::Publisher gps_pub = nh_private.advertise<nav_msgs::Odometry>("/file2enu_odom", 50);
    tf::TransformBroadcaster gps_broadcaster;
    ros::Publisher path_pub = nh_private.advertise<nav_msgs::Path>("/track_path", 1, true);
    ros::Publisher ego_pathArea_pub = nh_private.advertise<visualization_msgs::MarkerArray>("ego_pathArea", 1, true);
    ros::Publisher vehicle_markers_pub = nh_private.advertise<visualization_msgs::MarkerArray>("vehicle_markers", 1, true);
    ros::Publisher rviz_camera_pub = nh_private.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);

    // 读取参数
    string track_file, frame_id, link_path_file, map_file, config_file;
    nh_private.param<string>("track_file", track_file, "default_track_file.log");
    nh_private.param<string>("frame_id", frame_id, "/map");
    nh_private.param<string>("link_path_file", link_path_file, "default_link_path.json");
    nh_private.param<string>("map_file", map_file, "default_map.json");
    nh_private.param<string>("config_file", config_file, "default_config.json");

    // 匹配主车道路信息（假设此函数不需要修改）
    unordered_map<string, Json> ego_link_paths;
    // Match_ego_path(ego_link_paths, link_path_file); // 根据需要启用

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

    // 定义颜色和缩放映射（根据需要调整）
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

        // 发布路径区域标记（保持不变）
        // ... 如果需要，可以保留现有的路径标记发布逻辑 ...

        // 解析其他车辆信息
        Json vehicle_info_json = jsonData["details"]["vehicle_info"];
        vector<Vehicle> vehicles = parseVehicleInfo(vehicle_info_json);

        // 清空之前的 MarkerArray
        vehicle_markers.markers.clear();

        // 生成车辆标记
        int marker_id = 0;
        for (const auto& veh : vehicles) {
            // 创建车辆盒子标记
            visualization_msgs::Marker veh_marker;
            veh_marker.header.frame_id = frame_id;
            veh_marker.header.stamp = current_time;
            veh_marker.ns = "vehicles";
            veh_marker.id = marker_id;
            veh_marker.type = visualization_msgs::Marker::CUBE;
            veh_marker.action = visualization_msgs::Marker::ADD;

            // 设置颜色
            if (veh.width > 6.5) { // 例如，宽度大于6.5为 NTE200
                veh_marker.color = color_map_minesim.at("surrounding_truck_NTE200");
            } else if (veh.width > 4.0) { // 宽度大于4.0为 NTE240
                veh_marker.color = color_map_minesim.at("surrounding_truck_NTE240");
            } else { // 其他为 Pickup
                veh_marker.color = color_map_minesim.at("surrounding_car_pickup");
            }

            // 设置缩放
            veh_marker.scale.x = veh.length;
            veh_marker.scale.y = veh.width;
            veh_marker.scale.z = veh.height;

            // 设置位置
            veh_marker.pose.position.x = veh.x;
            veh_marker.pose.position.y = veh.y;
            veh_marker.pose.position.z = veh.alt + veh.height / 2.0; // 盒子中心在底部上方一半高度

            // 设置方向
            veh_marker.pose.orientation = tf::createQuaternionMsgFromYaw(CorrectionAngle(veh.yaw_rad, 0.0));

            // 设置生命周期
            veh_marker.lifetime = ros::Duration();

            // 添加到 MarkerArray
            vehicle_markers.markers.push_back(veh_marker);

            // 创建文本标记（ID + 速度）
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = frame_id;
            text_marker.header.stamp = current_time;
            text_marker.ns = "vehicle_texts";
            text_marker.id = marker_id;
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
            text_marker.color = color_map_minesim.at("surrounding_car_truck_text");

            // 设置位置
            text_marker.pose.position.x = veh.x;
            text_marker.pose.position.y = veh.y;
            text_marker.pose.position.z = veh.alt + veh.height + 0.5; // 文本位于车辆上方

            // 设置方向
            text_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

            // 设置生命周期
            text_marker.lifetime = ros::Duration();

            // 添加到 MarkerArray
            vehicle_markers.markers.push_back(text_marker);

            marker_id++;
        }

        // 发布所有车辆标记
        vehicle_markers_pub.publish(vehicle_markers);

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

        // 更新相机的上方向
        geometry_msgs::Vector3 up;
        up.x = 0;
        up.y = 0;
        up.z = 10;
        camera.up.vector = up;
        camera.up.header.frame_id = "base_link";

        camera.time_from_start = ros::Duration();
        rviz_camera_pub.publish(camera);

        // 更新相机位置
        update_eye_x += update_v * cos(CorrectionAngle(yaw_rad_ego, 0.0) + PI);
        update_eye_y += update_v * sin(CorrectionAngle(yaw_rad_ego, 0.0) + PI);

        loop_rate.sleep();
        ros::spinOnce();
    }

    // 关闭打开的文件
    inFile.close();
    cout << "file2enu 停止!" << endl;
    return 0;
}
```

### 3. 关键优化点解析

#### a. 使用结构体 `Vehicle` 管理车辆信息

定义了一个 `Vehicle` 结构体来存储每辆车的相关信息，包括 ID、位置、速度、尺寸等。这使得代码更加清晰，并且便于管理和扩展。

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

#### b. 动态解析车辆信息

通过遍历 `vehicle_info` JSON 对象的键，动态提取非 `ego` 车辆的信息，并存储在 `Vehicle` 结构体的实例中。

```cpp
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
```

#### c. 使用 `MarkerArray` 批量发布车辆标记

通过 `MarkerArray`，可以在一次发布中传输所有车辆的标记，简化发布逻辑并提高效率。

```cpp
// 定义 MarkerArray
visualization_msgs::MarkerArray vehicle_markers;

// 清空之前的 MarkerArray
vehicle_markers.markers.clear();

// 生成车辆标记
int marker_id = 0;
for (const auto& veh : vehicles) {
    // 创建车辆盒子标记
    visualization_msgs::Marker veh_marker;
    veh_marker.header.frame_id = frame_id;
    veh_marker.header.stamp = current_time;
    veh_marker.ns = "vehicles";
    veh_marker.id = marker_id;
    veh_marker.type = visualization_msgs::Marker::CUBE;
    veh_marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色
    if (veh.width > 6.5) { // 例如，宽度大于6.5为 NTE200
        veh_marker.color = color_map_minesim.at("surrounding_truck_NTE200");
    } else if (veh.width > 4.0) { // 宽度大于4.0为 NTE240
        veh_marker.color = color_map_minesim.at("surrounding_truck_NTE240");
    } else { // 其他为 Pickup
        veh_marker.color = color_map_minesim.at("surrounding_car_pickup");
    }

    // 设置缩放
    veh_marker.scale.x = veh.length;
    veh_marker.scale.y = veh.width;
    veh_marker.scale.z = veh.height;

    // 设置位置
    veh_marker.pose.position.x = veh.x;
    veh_marker.pose.position.y = veh.y;
    veh_marker.pose.position.z = veh.alt + veh.height / 2.0; // 盒子中心在底部上方一半高度

    // 设置方向
    veh_marker.pose.orientation = tf::createQuaternionMsgFromYaw(CorrectionAngle(veh.yaw_rad, 0.0));

    // 设置生命周期
    veh_marker.lifetime = ros::Duration();

    // 添加到 MarkerArray
    vehicle_markers.markers.push_back(veh_marker);

    // 创建文本标记（ID + 速度）
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = current_time;
    text_marker.ns = "vehicle_texts";
    text_marker.id = marker_id;
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
    text_marker.color = color_map_minesim.at("surrounding_car_truck_text");

    // 设置位置
    text_marker.pose.position.x = veh.x;
    text_marker.pose.position.y = veh.y;
    text_marker.pose.position.z = veh.alt + veh.height + 0.5; // 文本位于车辆上方

    // 设置方向
    text_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // 设置生命周期
    text_marker.lifetime = ros::Duration();

    // 添加到 MarkerArray
    vehicle_markers.markers.push_back(text_marker);

    marker_id++;
}

// 发布所有车辆标记
vehicle_markers_pub.publish(vehicle_markers);
```

#### d. 简化发布器数量

使用单一的 `MarkerArray` 发布器 `vehicle_markers_pub` 取代为每辆车单独定义的发布器，极大简化了代码结构。

#### e. 动态处理任意数量的车辆

通过遍历 JSON 中的 `vehicle_info`，自动处理任意数量的车辆，无需预先定义具体的车辆数量或编号。

### 4. 进一步优化建议

#### a. 使用函数封装重复逻辑

为了提高代码的可读性和可维护性，可以将标记创建逻辑封装为函数。例如：

```cpp
visualization_msgs::Marker createVehicleMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map) {
    visualization_msgs::Marker veh_marker;
    veh_marker.header.frame_id = "map";
    veh_marker.header.stamp = ros::Time::now();
    veh_marker.ns = "vehicles";
    veh_marker.id = id;
    veh_marker.type = visualization_msgs::Marker::CUBE;
    veh_marker.action = visualization_msgs::Marker::ADD;

    // 设置颜色
    if (veh.width > 6.5) {
        veh_marker.color = color_map.at("surrounding_truck_NTE200");
    } else if (veh.width > 4.0) {
        veh_marker.color = color_map.at("surrounding_truck_NTE240");
    } else {
        veh_marker.color = color_map.at("surrounding_car_pickup");
    }

    // 设置缩放
    veh_marker.scale.x = veh.length;
    veh_marker.scale.y = veh.width;
    veh_marker.scale.z = veh.height;

    // 设置位置
    veh_marker.pose.position.x = veh.x;
    veh_marker.pose.position.y = veh.y;
    veh_marker.pose.position.z = veh.alt + veh.height / 2.0;

    // 设置方向
    veh_marker.pose.orientation = tf::createQuaternionMsgFromYaw(CorrectionAngle(veh.yaw_rad, 0.0));

    // 设置生命周期
    veh_marker.lifetime = ros::Duration();

    return veh_marker;
}

visualization_msgs::Marker createTextMarker(const Vehicle& veh, int id, const std::map<std::string, common::ColorARGB>& color_map) {
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map";
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
    text_marker.pose.position.z = veh.alt + veh.height + 0.5;

    // 设置方向
    text_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // 设置生命周期
    text_marker.lifetime = ros::Duration();

    return text_marker;
}
```

然后在主循环中调用这些函数：

```cpp
for (const auto& veh : vehicles) {
    // 创建车辆盒子标记
    visualization_msgs::Marker veh_marker = createVehicleMarker(veh, marker_id, color_map_minesim);
    vehicle_markers.markers.push_back(veh_marker);

    // 创建文本标记（ID + 速度）
    visualization_msgs::Marker text_marker = createTextMarker(veh, marker_id, color_map_minesim);
    vehicle_markers.markers.push_back(text_marker);

    marker_id++;
}
```

#### b. 动态调整车辆数量和标记

由于 `MarkerArray` 支持动态调整标记的数量，只需在每次循环中清空并重新填充 `vehicle_markers` 即可。

### 5. 总结

通过上述优化，我们实现了以下目标：

1. **代码精简**：通过使用结构体和循环，消除了为每辆车单独定义变量和发布器的冗余代码。
2. **动态解析车辆信息**：自动处理 JSON 中的任意数量车辆，提升了代码的灵活性和可扩展性。
3. **使用 `MarkerArray`**：批量发布车辆标记，提高了发布效率，并简化了发布逻辑。

这些优化不仅提高了代码的可读性和可维护性，还使其更易于扩展和适应未来的需求。如果有进一步的需求或需要更详细的解释，请随时告知！
