#include "visualization_msgs/Marker.h"
#include <boost/filesystem.hpp> // 包含Boost的filesystem库
#include <cmath>
#include <cstdlib> //string转化为double
#include <flann/flann.hpp>
#include <fstream>
#include <geometry_msgs/TransformStamped.h>
#include <gps/geodetic_conv.hpp> //坐标转换文件
#include <gps/json.hpp>
#include <iomanip> //保留有效小数
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
#include <visualization_msgs/MarkerArray.h>

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

// 读取log文件
void ReadLogFile_Json(std::string &jsonString, string &abs_time, double &x_ego, double &y_ego, double &yaw_rad_ego, double &v_mps_ego,
                      double &acc_mpss_ego, double &width_ego, double &length_ego, double &alt_ego, double &PitchAngle_ego, double &x_1, double &y_1,
                      double &yaw_rad_1, double &v_mps_1, double &acc_mpss_1, double &width_1, double &length_1, double &alt_1, double &PitchAngle_1,
                      double &x_2, double &y_2, double &yaw_rad_2, double &v_mps_2, double &acc_mpss_2, double &width_2, double &length_2,
                      double &alt_2, double &PitchAngle_2, double &x_3, double &y_3, double &yaw_rad_3, double &v_mps_3, double &acc_mpss_3,
                      double &width_3, double &length_3, double &alt_3, double &PitchAngle_3, double &x_4, double &y_4, double &yaw_rad_4,
                      double &v_mps_4, double &acc_mpss_4, double &width_4, double &length_4, double &alt_4, double &PitchAngle_4) {
    double dt = 0.1;
    nlohmann::json jsonData = Json::parse(jsonString);

    double time_s_num = jsonData["time_s"].get<double>();
    // / 使用字符串流将数字转换为保留两位小数的字符串
    std::ostringstream oss;
    oss.precision(2);
    oss << std::fixed << time_s_num;
    abs_time = oss.str();

    // 输出时直接打印保留两位小数的字符串
    printf("abs_time: %s\n", abs_time.c_str());

    x_ego = jsonData["details"]["vehicle_info"]["ego"]["x"].get<double>();
    y_ego = jsonData["details"]["vehicle_info"]["ego"]["y"].get<double>();
    yaw_rad_ego = jsonData["details"]["vehicle_info"]["ego"]["yaw_rad"].get<double>();
    v_mps_ego = jsonData["details"]["vehicle_info"]["ego"]["v_mps"].get<double>();
    acc_mpss_ego = jsonData["details"]["vehicle_info"]["ego"]["acc_mpss"].get<double>();
    width_ego = jsonData["details"]["vehicle_info"]["ego"]["shape"]["width"].get<double>();
    length_ego = jsonData["details"]["vehicle_info"]["ego"]["shape"]["length"].get<double>();
    alt_ego = 0;
    PitchAngle_ego = jsonData["details"]["vehicle_info"]["ego"]["yawrate_radps"].get<double>();

    if (jsonData["details"]["vehicle_info"].contains("1")) {
        x_1 = jsonData["details"]["vehicle_info"]["1"]["x"].get<double>();
        y_1 = jsonData["details"]["vehicle_info"]["1"]["y"].get<double>();
        yaw_rad_1 = jsonData["details"]["vehicle_info"]["1"]["yaw_rad"].get<double>();
        v_mps_1 = jsonData["details"]["vehicle_info"]["1"]["v_mps"].get<double>();
        acc_mpss_1 = jsonData["details"]["vehicle_info"]["1"]["acc_mpss"].get<double>();
        width_1 = jsonData["details"]["vehicle_info"]["1"]["shape"]["width"].get<double>();
        length_1 = jsonData["details"]["vehicle_info"]["1"]["shape"]["length"].get<double>();
        alt_1 = 0;
        PitchAngle_1 = jsonData["details"]["vehicle_info"]["1"]["yawrate_radps"].get<double>();
    }

    if (jsonData["details"]["vehicle_info"].contains("2")) {
        x_2 = jsonData["details"]["vehicle_info"]["2"]["x"].get<double>();
        y_2 = jsonData["details"]["vehicle_info"]["2"]["y"].get<double>();
        yaw_rad_2 = jsonData["details"]["vehicle_info"]["2"]["yaw_rad"].get<double>();
        v_mps_2 = jsonData["details"]["vehicle_info"]["2"]["v_mps"].get<double>();
        acc_mpss_2 = jsonData["details"]["vehicle_info"]["2"]["acc_mpss"].get<double>();
        width_2 = jsonData["details"]["vehicle_info"]["2"]["shape"]["width"].get<double>();
        length_2 = jsonData["details"]["vehicle_info"]["2"]["shape"]["length"].get<double>();
        alt_2 = 0;
        PitchAngle_2 = PitchAngle_1 = jsonData["details"]["vehicle_info"]["2"]["yawrate_radps"].get<double>();
    }

    if (jsonData["details"]["vehicle_info"].contains("3")) {
        x_3 = jsonData["details"]["vehicle_info"]["3"]["x"].get<double>();
        y_3 = jsonData["details"]["vehicle_info"]["3"]["y"].get<double>();
        yaw_rad_3 = jsonData["details"]["vehicle_info"]["3"]["yaw_rad"].get<double>();
        v_mps_3 = jsonData["details"]["vehicle_info"]["3"]["v_mps"].get<double>();
        acc_mpss_3 = jsonData["details"]["vehicle_info"]["3"]["acc_mpss"].get<double>();
        width_3 = jsonData["details"]["vehicle_info"]["3"]["shape"]["width"].get<double>();
        length_3 = jsonData["details"]["vehicle_info"]["3"]["shape"]["length"].get<double>();
        alt_3 = 0;
        PitchAngle_3 = jsonData["details"]["vehicle_info"]["3"]["yawrate_radps"].get<double>();
    }

    if (jsonData["details"]["vehicle_info"].contains("4")) {
        x_4 = jsonData["details"]["vehicle_info"]["4"]["x"].get<double>();
        y_4 = jsonData["details"]["vehicle_info"]["4"]["y"].get<double>();
        yaw_rad_4 = jsonData["details"]["vehicle_info"]["4"]["yaw_rad"].get<double>();
        v_mps_4 = jsonData["details"]["vehicle_info"]["4"]["v_mps"].get<double>();
        acc_mpss_4 = jsonData["details"]["vehicle_info"]["4"]["acc_mpss"].get<double>();
        width_4 = jsonData["details"]["vehicle_info"]["4"]["shape"]["width"].get<double>();
        length_4 = jsonData["details"]["vehicle_info"]["4"]["shape"]["length"].get<double>();
        alt_4 = 0;
        PitchAngle_4 = jsonData["details"]["vehicle_info"]["4"]["yawrate_radps"].get<double>();
    }
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
    std::fstream fs(config_file);                                                          // 创建文件输入流
    if (!fs.is_open()) {                                                                   // 检查文件是否打开成功
        ROS_ERROR_STREAM(std::string("#log# Failed to open config_file: ") + config_file); // 输出错误信息
        throw runtime_error("#log# config_file open failed.");                             // 抛出异常
    }

    // 读取文件内容
    Json jsonData; // JSON对象
    try {
        fs >> jsonData;                                                                          // 解析JSON文件
    } catch (const Json::parse_error &e) {                                                       // 捕获JSON解析错误
        ROS_ERROR_STREAM(std::string("#log# Error to parse error config_file: ") + config_file); // 输出错误信息
        throw;                                                                                   // 继续抛出异常
    }

    // 检查并解析字段: alt_info_visualization
    if (jsonData.contains("alt_info_visualization")) {         // 检查字段是否存在
        if (jsonData["alt_info_visualization"].is_boolean()) { // 如果是布尔类型
            altInfoVisualization = jsonData["alt_info_visualization"].get<bool>();
        } else if (jsonData["alt_info_visualization"].is_string()) { // 如果是字符串类型
            string valueStr = jsonData["alt_info_visualization"];
            altInfoVisualization = (valueStr == "true"); // 转换为布尔值
        } else {
            cerr << "#log# alt_info_visualization字段类型错误" << endl; // 输出错误信息
            throw runtime_error("#log# 配置文件格式错误");              // 抛出异常
        }
    } else {
        cerr << "#log# 缺少alt_info_visualization字段" << endl; // 输出错误信息
        throw runtime_error("#log# 配置文件缺少必要字段");      // 抛出异常
    }

    // 检查并解析相机参数：eye_position和focus_position
    try {
        x_eye = jsonData["eye_position"]["x"].get<double>();   // 获取eye位置x
        y_eye = jsonData["eye_position"]["y"].get<double>();   // 获取eye位置y
        z_eye = jsonData["eye_position"]["z"].get<double>();   // 获取eye位置z
        x_foc = jsonData["focus_position"]["x"].get<double>(); // 获取focus位置x
        y_foc = jsonData["focus_position"]["y"].get<double>(); // 获取focus位置y
        z_foc = jsonData["focus_position"]["z"].get<double>(); // 获取focus位置z
    } catch (const Json::out_of_range &e) {                    // 捕获字段缺失错误
        cerr << "#log# 解析eye_position或focus_position字段时出错: " << e.what() << endl;
        throw; // 抛出异常
    }

    // 检查并解析更新速度：update_v
    if (jsonData.contains("update_v")) { // 检查字段是否存在
        try {
            update_v = jsonData["update_v"].get<double>(); // 获取更新速度
        } catch (...) {
            cerr << "#log# 解析update_v字段时出错" << endl; // 输出错误信息
            throw runtime_error("update_v字段格式错误");    // 抛出异常
        }
    } else {
        cerr << "#log# 缺少update_v字段" << endl;    // 输出错误信息
        throw runtime_error("配置文件缺少必要字段"); // 抛出异常
    }
}

// 获取点所在区域
int isPointInPolygonSimplified(double px, double py, std::vector<std::vector<double>> link_waypoints) {
    int i = 0;
    int j = link_waypoints.size();
    int wn = 0;
    for (i; i < (j - 1); i++) {
        // 获取当前点和下一个点的坐标
        double xi = link_waypoints[i][0], yi = link_waypoints[i][1];
        double xi_next = link_waypoints[i + 1][0], yi_next = link_waypoints[i + 1][1];
        // 计算叉积以确定点P与边的关系
        if (yi <= py) {
            if (yi_next > py) {
                if (((xi - px) * (yi_next - py) - (yi - py) * (xi_next - px)) > 0) {
                    ++wn;
                }
            }
        } else {
            if (yi_next <= py) {
                if (((xi - px) * (yi_next - py) - (yi - py) * (xi_next - px)) < 0) {
                    --wn;
                }
            }
        }
    }
    return wn == 0 ? 0 : 1;
}

// 获取多边形信息
void getPolygon(double &x, double &y, const string &map_file, string &polygon_file) {
    std::fstream fs(map_file);
    Json root;
    fs >> root;
    Json polygon_all_json = root["polygon"];
    Json node_all_json = root["node"];
    for (auto &polygon_json : polygon_all_json) {
        std::vector<std::vector<double>> link_waypoints;
        // std::cout << "polygon_json" << polygon_json["token"] << std::endl;
        for (auto &node_json : polygon_json["link_node_tokens"]) {
            for (auto &node_json_nodes : node_all_json) {
                if (node_json_nodes["token"] == node_json) {
                    // std::cout << node_json_nodes["token"] << std::endl;
                    std::vector<double> point = {node_json_nodes["x"].get<double>(), node_json_nodes["y"].get<double>()};
                    link_waypoints.push_back(point);
                }
            }
        }
        if (isPointInPolygonSimplified(x, y, link_waypoints)) {
            polygon_file = polygon_json["token"].get<string>();
            link_waypoints.clear();
            break;
        }
    }
}

int getNumber(const string &polygon_file) {
    size_t dashPos = polygon_file.find('-');
    std::string numberStr = polygon_file.substr(dashPos + 1);
    int number = std::stoi(numberStr);
    return number;
}

// 提取主车道路信息,link_paths:所有log中涉及到的道路信息，ego_link_paths:存储主车道路信息，link_path_file：当前log涉及到的车道线
void Match_ego_path(unordered_map<string, Json> &ego_link_paths, const string &link_path_file) {
    std::fstream fs_link_path(link_path_file);
    Json root_link_path;
    fs_link_path >> root_link_path;

    Json link_path = root_link_path["route_planner_info"]["refline_smooth"];
    for (const auto &point : link_path) {
        vector<double> xyz = {point[0], point[1], point[2]};
        ego_link_paths["refline_smooth"].push_back(xyz);
    }
}

// 修正角度
double CorrectionAngle(double &log_yaw_rad, double &log_HeadingAngle_start) {
    double to_start_HeadingAngle = 0;

    // to_start_HeadingAngle = (360-log_yaw_rad+90-log_HeadingAngle_start)/180*PI;//TODO
    to_start_HeadingAngle = log_yaw_rad;
    if (to_start_HeadingAngle > 2 * PI) // 角度误差修正至【-2PI，2PI】
    {
        to_start_HeadingAngle = to_start_HeadingAngle - 2 * PI;
    } else if (to_start_HeadingAngle < -2 * PI) {
        to_start_HeadingAngle = to_start_HeadingAngle + 2 * PI;
    }
    if (to_start_HeadingAngle > PI) // 角度误差修正至【-PI，PI】
    {
        to_start_HeadingAngle = to_start_HeadingAngle - 2 * PI;
    } else if (to_start_HeadingAngle < -PI) {
        to_start_HeadingAngle = to_start_HeadingAngle + 2 * PI;
    }
    to_start_HeadingAngle = to_start_HeadingAngle + PI;

    return to_start_HeadingAngle;
}

// 以写模式打开文件
ofstream outfile;
ros::Time current_time, last_time;

int main(int argc, char **argv) {
    // 初始化节点
    ros::init(argc, argv, "file2enu");
    // 声明节点句柄
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
    // 注册Publisher到话题GPS_sim

    ros::Publisher gps_pub = nh_private.advertise<nav_msgs::Odometry>("/file2enu_odom", 50);
    tf::TransformBroadcaster gps_broadcaster;
    tf::Transform ego_view;
    ros::Publisher path_pub = nh_private.advertise<nav_msgs::Path>("/track_path", 1, true);
    ros::Publisher ego_pathArea_pub = nh_private.advertise<visualization_msgs::MarkerArray>("ego_pathArea", 1, true);
    ros::Publisher pubVehicle0 = nh_private.advertise<visualization_msgs::Marker>("vehicl_model_ego", 1, true);
    ros::Publisher pubVehicle1 = nh_private.advertise<visualization_msgs::Marker>("vehicl_model_1", 1, true);
    ros::Publisher markerPub1 = nh_private.advertise<visualization_msgs::Marker>("ID_v_1", 1, true);
    ros::Publisher pubVehicle2 = nh_private.advertise<visualization_msgs::Marker>("vehicl_model_2", 1, true);
    ros::Publisher markerPub2 = nh_private.advertise<visualization_msgs::Marker>("ID_v_2", 1, true);
    ros::Publisher pubVehicle3 = nh_private.advertise<visualization_msgs::Marker>("vehicl_model_3", 1, true);
    ros::Publisher markerPub3 = nh_private.advertise<visualization_msgs::Marker>("ID_v_3", 1, true);
    ros::Publisher pubVehicle4 = nh_private.advertise<visualization_msgs::Marker>("vehicl_model_4", 1, true);
    ros::Publisher markerPub4 = nh_private.advertise<visualization_msgs::Marker>("ID_v_4", 1, true);
    ros::Publisher rviz_camera_pub = nh_private.advertise<view_controller_msgs::CameraPlacement>("/rviz/camera_placement", 1);

    // 车辆路径文件
    string track_file;
    if (!nh_private.getParam("track_file", track_file)) {
        ROS_INFO_STREAM("#log# No input parameter for track_file, using default file!!!!!!!!!");
    }
    // 车辆log对应的log信息
    string frame_id;
    if (!nh_private.getParam("frame_id", frame_id)) {
        frame_id = "/map"; // 默认frame_id
        ROS_INFO_STREAM("#log# No input parameter for frame_id, using default value: map !!!!!!!!!");
    }

    // 读取涉及到的路径集的token
    string link_path_file;
    if (!nh_private.getParam("link_path_file", link_path_file)) {
        ROS_INFO_STREAM("#log# No input parameter for link_path_file, using default file!!!!!!!!!");
    }

    // 匹配主车道路信息
    unordered_map<string, Json> ego_link_paths;
    Match_ego_path(ego_link_paths, link_path_file);

    // 读取地图json信息
    string map_file;
    if (!nh_private.getParam("map_file", map_file)) {
        ROS_INFO_STREAM("#log# No input parameter for map_file, using default file!!!!!!!!!");
    }

    bool tf_pub;
    if (!nh_private.getParam("tf_pub", tf_pub)) {
        tf_pub = true; // 默认frame_id
        ROS_INFO_STREAM("#log# No input parameter for tf_pub, using default value: true !!!!!!!!!");
    }

    ifstream inFile(track_file.c_str(), ios::in);
    if (inFile) {
        cout << "open path file!" << endl;
    }

    // int i = 0;
    string line_track_file;

    string config_file;
    if (!nh_private.getParam("config_file", config_file)) {
        ROS_INFO_STREAM("#log# input parameter for config_file, using default file!!!!!!!!!");
    }

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    double dt = 0;

    // log变量声明
    string abs_time_load;
    // 主车信息
    double x_ego, y_ego, yaw_rad_ego, v_mps_ego, acc_mpss_ego, width_ego, length_ego, alt_ego, PitchAngle_ego;
    // 周围车辆信息(定义周围车辆)
    double x_1, y_1, yaw_rad_1, v_mps_1, acc_mpss_1, width_1, length_1, alt_1, PitchAngle_1;
    double x_2, y_2, yaw_rad_2, v_mps_2, acc_mpss_2, width_2, length_2, alt_2, PitchAngle_2;
    double x_3, y_3, yaw_rad_3, v_mps_3, acc_mpss_3, width_3, length_3, alt_3, PitchAngle_3;
    double x_4, y_4, yaw_rad_4, v_mps_4, acc_mpss_4, width_4, length_4, alt_4, PitchAngle_4;
    std::string Polygon_ego, Polygon_1, Polygon_2, Polygon_3, Polygon_4;
    std::string Polygon_ego_str, Polygon_1_str, Polygon_2_str, Polygon_3_str, Polygon_4_str;

    bool altInfoVisualization = false;
    double x_eye, y_eye, z_eye;
    double x_foc, y_foc, z_foc;
    double update_v;
    readConfig(altInfoVisualization, x_eye, y_eye, z_eye, x_foc, y_foc, z_foc, update_v, config_file);

    outfile.open("gps.txt");

    // # === 模型信息 ===
    // # 顶点数量: 81166
    // # 面数量: 81008
    // # 长度 (X轴) : 1169.141 米
    // # 宽度 (Y轴) : 997.701 米
    // # 高度 (Z轴) : 740.219 米
    const double ORIGINAL_LENGTH = 1169.141; // 模型原始长度（单位：米）
    const double ORIGINAL_WIDTH = 997.701;   // 模型原始宽度（单位：米）
    const double ORIGINAL_HEIGHT = 740.219;  // 模型原始高度（单位：米）

    // 定义周围车辆颜色信息
    common::ColorARGB color_route_path = common::color_map_minesim.at("ego_ref_path");
    Vec3f scale_route_path = common::object_scale_map_minesim.at("ego_ref_path");

    common::ColorARGB color_ego = common::color_map_minesim.at("ego_mine_truck");
    Vec3f scale_ego = common::object_scale_map_minesim.at("ego_mine_truck");

    common::ColorARGB color_pickup = common::color_map_minesim.at("surrounding_car_pickup");
    Vec3f scale_pickup = common::object_scale_map_minesim.at("surrounding_car_pickup");

    common::ColorARGB color_truck_NTE240 = common::color_map_minesim.at("surrounding_truck_NTE240");
    Vec3f scale_truck_NTE240 = common::object_scale_map_minesim.at("surrounding_truck_NTE240");

    common::ColorARGB color_truck_NTE200 = common::color_map_minesim.at("surrounding_truck_NTE200");
    Vec3f scale_truck_NTE200 = common::object_scale_map_minesim.at("surrounding_truck_NTE200");

    common::ColorARGB color_car_truck_text = common::color_map_minesim.at("surrounding_car_truck_text");
    Vec3f scale_car_truck_text = common::object_scale_map_minesim.at("surrounding_car_truck_text");

    // 定义path消息类型
    nav_msgs::Path track_path;
    track_path.header.stamp = current_time;
    track_path.header.frame_id = frame_id;

    double update_eye_x = x_eye;
    double update_eye_y = y_eye;
    // 设定原点信息
    double log_lat_start = 0, log_lon_start = 0, log_HeadingAngle_start = 0; // 418359,4.41314e+06
    ros::Rate loop_rate(10);                                                 // 模拟GPS消息发送频率(rviz仿真频率)
    while (ros::ok() && (getline(inFile, line_track_file))) {
        ReadLogFile_Json(line_track_file, abs_time_load, x_ego, y_ego, yaw_rad_ego, v_mps_ego, acc_mpss_ego, width_ego, length_ego, alt_ego,
                         PitchAngle_ego, x_1, y_1, yaw_rad_1, v_mps_1, acc_mpss_1, width_1, length_1, alt_1, PitchAngle_1, x_2, y_2, yaw_rad_2,
                         v_mps_2, acc_mpss_2, width_2, length_2, alt_2, PitchAngle_2, x_3, y_3, yaw_rad_3, v_mps_3, acc_mpss_3, width_3, length_3,
                         alt_3, PitchAngle_3, x_4, y_4, yaw_rad_4, v_mps_4, acc_mpss_4, width_4, length_4, alt_4, PitchAngle_4);

        // 2.坐标转换
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();

        // 修正角度
        double corAngle_ego = CorrectionAngle(yaw_rad_ego, log_HeadingAngle_start);
        double corAngle_1 = CorrectionAngle(yaw_rad_1, log_HeadingAngle_start);
        double corAngle_2 = CorrectionAngle(yaw_rad_2, log_HeadingAngle_start);
        double corAngle_3 = CorrectionAngle(yaw_rad_3, log_HeadingAngle_start);
        double corAngle_4 = CorrectionAngle(yaw_rad_4, log_HeadingAngle_start);

        // 3.发布数据
        // 可视化局部车道:ego 车辆 搜索得到的 route 参考路径
        visualization_msgs::MarkerArray ego_lane_markers;
        int marker_id = 0;
        for (const auto &ego_path : ego_link_paths) {
            Json lane_coordinates = ego_path.second;                 // 所有的坐标数据
            int num_pts = static_cast<int>(lane_coordinates.size()); // 坐标点数量
            visualization_msgs::Marker lane_marker;
            lane_marker.header.frame_id = frame_id;
            lane_marker.header.stamp = current_time;
            lane_marker.id = marker_id;
            marker_id++;
            lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
            lane_marker.action = visualization_msgs::Marker::MODIFY;
            for (int k = 0; k < num_pts; ++k) {
                geometry_msgs::Point pt;
                pt.x = lane_coordinates[k][0].get<double>();
                pt.y = lane_coordinates[k][1].get<double>();
                pt.z = altInfoVisualization ? lane_coordinates[k][3].get<double>() - 0.3 : 1.0;
                lane_marker.points.push_back(pt);
            }
            // route 参考路径:颜色,大小

            lane_marker.color.a = color_route_path.a;
            lane_marker.color.r = color_route_path.r;
            lane_marker.color.g = color_route_path.g;
            lane_marker.color.b = color_route_path.b;
            // lane_marker.color.a = 0.2;
            // lane_marker.color.r = 220 / 255;
            // lane_marker.color.g = 6 / 255;
            // lane_marker.color.b = 255 / 255;

            lane_marker.scale.x = scale_route_path(0);
            lane_marker.scale.y = scale_route_path(1);
            lane_marker.scale.z = scale_route_path(2);
            // lane_marker.scale.x = 2.0;
            // lane_marker.scale.y = 0.1;
            // lane_marker.scale.z = 0.1;
            ego_lane_markers.markers.push_back(lane_marker);
        }
        ego_pathArea_pub.publish(ego_lane_markers);

        // 发布path信息
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = x_ego;
        this_pose_stamped.pose.position.y = y_ego;
        // cout << "x:" << x_ego << "   y:" << y_ego << endl;
        this_pose_stamped.pose.position.z = alt_ego;
        geometry_msgs::Quaternion goal_quat_load = tf::createQuaternionMsgFromYaw(corAngle_ego);
        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = frame_id;
        track_path.poses.push_back(this_pose_stamped);
        path_pub.publish(track_path);

        // ================ 4构建车辆模型 ================
        // 4.0主车
        visualization_msgs::Marker vehicle_marker_0_;
        vehicle_marker_0_.header.frame_id = frame_id;
        vehicle_marker_0_.header.stamp = current_time;
        vehicle_marker_0_.ns = "vehicle";
        vehicle_marker_0_.id = 0;
        vehicle_marker_0_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_0_.type = visualization_msgs::Marker::MESH_RESOURCE;
        string mesh_resource_0 =
            "package://common/materials/vehicle_model/Mining_Truck_v1_L1.123c6431771f-f361-4a1f-8727-590a5c800ce0/16747_Mining_Truck_v1.obj";

        // # === 模型信息 ===
        // # 顶点数量: 81166
        // # 面数量: 81008
        // # 长度 (X轴) : 1169.141 米
        // # 宽度 (Y轴) : 997.701 米
        // # 高度 (Z轴) : 740.219 米
        double height_ego = 1.675;
        if (abs(width_ego - 4) < 0.2) {
            height_ego = 3.5;
        }
        if (abs(width_ego - 6.7) < 0.2) {
            height_ego = 6.9;
        }

        double scale_x = length_ego / ORIGINAL_LENGTH;
        double scale_y = width_ego / ORIGINAL_WIDTH;
        double scale_z = height_ego / ORIGINAL_HEIGHT;

        vehicle_marker_0_.scale.x = scale_x;
        vehicle_marker_0_.scale.y = scale_y;
        vehicle_marker_0_.scale.z = scale_z;

        vehicle_marker_0_.color.r = color_ego.r;
        vehicle_marker_0_.color.g = color_ego.g;
        vehicle_marker_0_.color.b = color_ego.b;
        vehicle_marker_0_.color.a = color_ego.a;
        // vehicle_marker_0_.color.r = 0.85; // 1
        // vehicle_marker_0_.color.g = 0.64; // 0.5
        // vehicle_marker_0_.color.b = 0.12; // 0
        // vehicle_marker_0_.color.a = 1;

        vehicle_marker_0_.mesh_resource = mesh_resource_0;
        vehicle_marker_0_.pose.position.x = x_ego;
        vehicle_marker_0_.pose.position.y = y_ego;
        vehicle_marker_0_.pose.position.z = alt_ego + 0.4;
        geometry_msgs::Quaternion tf_vehicle_marker_0 = tf::createQuaternionMsgFromYaw(corAngle_ego);
        vehicle_marker_0_.pose.orientation.x = tf_vehicle_marker_0.x;
        vehicle_marker_0_.pose.orientation.y = tf_vehicle_marker_0.y;
        vehicle_marker_0_.pose.orientation.z = tf_vehicle_marker_0.z;
        vehicle_marker_0_.pose.orientation.w = tf_vehicle_marker_0.w;
        pubVehicle0.publish(vehicle_marker_0_);

        // ego_view.setOrigin( tf::Vector3(vehicle_marker_0_.pose.position.x, vehicle_marker_0_.pose.position.y,
        // vehicle_marker_0_.pose.position.z)); ego_view.setRotation(
        // tf::Quaternion(vehicle_marker_0_.pose.orientation.x,vehicle_marker_0_.pose.orientation.y ,vehicle_marker_0_.pose.orientation.z,
        // vehicle_marker_0_.pose.orientation.w) ); gps_broadcaster.sendTransform(tf::StampedTransform(ego_view, ros::Time::now(), "world",
        // "corrat1"));

        // 4.1 周围车辆1
        visualization_msgs::Marker vehicle_marker_1_;
        vehicle_marker_1_.header.frame_id = frame_id;
        vehicle_marker_1_.header.stamp = current_time;
        vehicle_marker_1_.ns = "vehicle";
        vehicle_marker_1_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_1_.type = visualization_msgs::Marker::CUBE;
        vehicle_marker_1_.id = 1;
        // decide the color of the marker
        vehicle_marker_1_.color.a = color_pickup.a;
        vehicle_marker_1_.color.r = color_pickup.r;
        vehicle_marker_1_.color.g = color_pickup.g;
        vehicle_marker_1_.color.b = color_pickup.b;

        double height_1 = 1.675;
        if (abs(width_1 - 4) < 0.2) {
            height_1 = 3.5;
            vehicle_marker_1_.color.a = color_truck_NTE240.a;
            vehicle_marker_1_.color.r = color_truck_NTE240.r;
            vehicle_marker_1_.color.g = color_truck_NTE240.g;
            vehicle_marker_1_.color.b = color_truck_NTE240.b;
        }
        if (abs(width_1 - 6.7) < 0.2) {
            height_1 = 6.9;
            vehicle_marker_1_.color.a = color_truck_NTE200.a;
            vehicle_marker_1_.color.r = color_truck_NTE200.r;
            vehicle_marker_1_.color.g = color_truck_NTE200.g;
            vehicle_marker_1_.color.b = color_truck_NTE200.b;
        }
        vehicle_marker_1_.scale.x = length_1;
        vehicle_marker_1_.scale.y = width_1;
        vehicle_marker_1_.scale.z = height_1;
        // set marker position
        vehicle_marker_1_.pose.position.x = x_1;
        vehicle_marker_1_.pose.position.y = y_1;
        vehicle_marker_1_.pose.position.z = alt_1 + height_1 + 0.4;
        geometry_msgs::Quaternion tf_vehicle_marker_1 = tf::createQuaternionMsgFromYaw(corAngle_1);
        vehicle_marker_1_.pose.orientation.x = tf_vehicle_marker_1.x;
        vehicle_marker_1_.pose.orientation.y = tf_vehicle_marker_1.y;
        vehicle_marker_1_.pose.orientation.z = tf_vehicle_marker_1.z;
        vehicle_marker_1_.pose.orientation.w = tf_vehicle_marker_1.w;
        // set marker action
        vehicle_marker_1_.lifetime = ros::Duration(); //(sec,nsec),0 forever(取消自动删除)
        pubVehicle1.publish(vehicle_marker_1_);

        // 4.1.2 周围车辆1相关信息 text="ID + speed"
        visualization_msgs::Marker ID_marker_1_;
        ID_marker_1_.header.frame_id = frame_id;
        ID_marker_1_.header.stamp = current_time;
        ID_marker_1_.ns = "basic_shapes";
        ID_marker_1_.action = visualization_msgs::Marker::ADD;
        ID_marker_1_.pose.orientation.w = 1.0;
        ID_marker_1_.id = 1;
        ID_marker_1_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ID_marker_1_.scale.x = scale_car_truck_text(0);
        ID_marker_1_.scale.y = scale_car_truck_text(1);
        ID_marker_1_.scale.z = scale_car_truck_text(2);
        ID_marker_1_.color.b = color_car_truck_text.b;
        ID_marker_1_.color.g = color_car_truck_text.g;
        ID_marker_1_.color.r = color_car_truck_text.r;
        ID_marker_1_.color.a = color_car_truck_text.a;
        geometry_msgs::Pose pose_1;
        pose_1.position.x = x_1;
        pose_1.position.y = y_1;
        pose_1.position.z = alt_1 + height_1 + 3.2;
        ostringstream str_1;
        str_1 << "ID:1 " << "V:" << fixed << setprecision(1) << v_mps_1;
        ID_marker_1_.text = str_1.str();
        ID_marker_1_.pose = pose_1;
        markerPub1.publish(ID_marker_1_);

        // 4.2  周围车辆2
        visualization_msgs::Marker vehicle_marker_2_;
        vehicle_marker_2_.header.frame_id = frame_id;
        vehicle_marker_2_.header.stamp = current_time;
        vehicle_marker_2_.ns = "vehicle";
        vehicle_marker_2_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_2_.type = visualization_msgs::Marker::CUBE;
        vehicle_marker_2_.id = 2;
        // decide the color of the marker
        vehicle_marker_2_.color.a = color_pickup.a;
        vehicle_marker_2_.color.r = color_pickup.r;
        vehicle_marker_2_.color.g = color_pickup.g;
        vehicle_marker_2_.color.b = color_pickup.b;
        double height_2 = 1.675;
        if (abs(width_2 - 4) < 0.2) {
            height_2 = 3.5;
            vehicle_marker_2_.color.a = color_truck_NTE240.a;
            vehicle_marker_2_.color.r = color_truck_NTE240.r;
            vehicle_marker_2_.color.g = color_truck_NTE240.g;
            vehicle_marker_2_.color.b = color_truck_NTE240.b;
        }
        if (abs(width_2 - 6.7) < 0.2) {
            height_2 = 6.9;
            vehicle_marker_2_.color.a = color_truck_NTE200.a;
            vehicle_marker_2_.color.r = color_truck_NTE200.r;
            vehicle_marker_2_.color.g = color_truck_NTE200.g;
            vehicle_marker_2_.color.b = color_truck_NTE200.b;
        }
        vehicle_marker_2_.scale.x = length_2;
        vehicle_marker_2_.scale.y = width_2;
        vehicle_marker_2_.scale.z = height_2;
        // set marker position
        vehicle_marker_2_.pose.position.x = x_2;
        vehicle_marker_2_.pose.position.y = y_2;
        vehicle_marker_2_.pose.position.z = alt_2 + height_2 + 0.4;
        geometry_msgs::Quaternion tf_vehicle_marker_2 = tf::createQuaternionMsgFromYaw(corAngle_2);
        vehicle_marker_2_.pose.orientation.x = tf_vehicle_marker_2.x;
        vehicle_marker_2_.pose.orientation.y = tf_vehicle_marker_2.y;
        vehicle_marker_2_.pose.orientation.z = tf_vehicle_marker_2.z;
        vehicle_marker_2_.pose.orientation.w = tf_vehicle_marker_2.w;
        // set marker action
        vehicle_marker_2_.lifetime = ros::Duration(); //(sec,nsec),0 forever(取消自动删除)
        pubVehicle2.publish(vehicle_marker_2_);

        // 4.2.2 周围车辆2相关信息
        visualization_msgs::Marker ID_marker_2_;
        ID_marker_2_.header.frame_id = frame_id;
        ID_marker_2_.header.stamp = current_time;
        ID_marker_2_.ns = "basic_shapes";
        ID_marker_2_.action = visualization_msgs::Marker::ADD;
        ID_marker_2_.pose.orientation.w = 1.0;
        ID_marker_2_.id = 2;
        ID_marker_2_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ID_marker_2_.scale.x = scale_car_truck_text(0);
        ID_marker_2_.scale.y = scale_car_truck_text(1);
        ID_marker_2_.scale.z = scale_car_truck_text(2);
        ID_marker_2_.color.b = color_car_truck_text.b;
        ID_marker_2_.color.g = color_car_truck_text.g;
        ID_marker_2_.color.r = color_car_truck_text.r;
        ID_marker_2_.color.a = color_car_truck_text.a;
        geometry_msgs::Pose pose_2;
        pose_2.position.x = x_2;
        pose_2.position.y = y_2;
        pose_2.position.z = alt_2 + height_2 + 3.2;
        ostringstream str_2;
        str_2 << "ID:2 " << "V:" << fixed << setprecision(1) << v_mps_2;
        ID_marker_2_.text = str_2.str();
        ID_marker_2_.pose = pose_2;
        markerPub2.publish(ID_marker_2_);

        // 4.3  周围车辆3
        visualization_msgs::Marker vehicle_marker_3_;
        vehicle_marker_3_.header.frame_id = frame_id;
        vehicle_marker_3_.header.stamp = current_time;
        vehicle_marker_3_.ns = "vehicle";
        vehicle_marker_3_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_3_.type = visualization_msgs::Marker::CUBE;
        vehicle_marker_3_.id = 3;
        // decide the color of the marker
        vehicle_marker_3_.color.a = color_pickup.a;
        vehicle_marker_3_.color.r = color_pickup.r;
        vehicle_marker_3_.color.g = color_pickup.g;
        vehicle_marker_3_.color.b = color_pickup.b;
        double height_3 = 1.675;
        if (abs(width_3 - 4) < 0.2) {
            height_3 = 3.5;
            vehicle_marker_3_.color.a = color_truck_NTE240.a;
            vehicle_marker_3_.color.r = color_truck_NTE240.r;
            vehicle_marker_3_.color.g = color_truck_NTE240.g;
            vehicle_marker_3_.color.b = color_truck_NTE240.b;
        }
        if (abs(width_3 - 6.7) < 0.2) {
            height_3 = 6.9;
            vehicle_marker_3_.color.a = color_truck_NTE200.a;
            vehicle_marker_3_.color.r = color_truck_NTE200.r;
            vehicle_marker_3_.color.g = color_truck_NTE200.g;
            vehicle_marker_3_.color.b = color_truck_NTE200.b;
        }
        vehicle_marker_3_.scale.x = length_3;
        vehicle_marker_3_.scale.y = width_3;
        vehicle_marker_3_.scale.z = height_3;
        // set marker position
        vehicle_marker_3_.pose.position.x = x_3;
        vehicle_marker_3_.pose.position.y = y_3;
        vehicle_marker_3_.pose.position.z = alt_3 + height_3 + 0.4;
        geometry_msgs::Quaternion tf_vehicle_marker_3 = tf::createQuaternionMsgFromYaw(corAngle_3);
        vehicle_marker_3_.pose.orientation.x = tf_vehicle_marker_3.x;
        vehicle_marker_3_.pose.orientation.y = tf_vehicle_marker_3.y;
        vehicle_marker_3_.pose.orientation.z = tf_vehicle_marker_3.z;
        vehicle_marker_3_.pose.orientation.w = tf_vehicle_marker_3.w;
        // set marker action
        vehicle_marker_3_.lifetime = ros::Duration(); //(sec,nsec),0 forever(取消自动删除)
        pubVehicle3.publish(vehicle_marker_3_);

        // 4.3.2 周围车辆3相关信息
        visualization_msgs::Marker ID_marker_3_;
        ID_marker_3_.header.frame_id = frame_id;
        ID_marker_3_.header.stamp = current_time;
        ID_marker_3_.ns = "basic_shapes";
        ID_marker_3_.action = visualization_msgs::Marker::ADD;
        ID_marker_3_.pose.orientation.w = 1.0;
        ID_marker_3_.id = 3;
        ID_marker_3_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ID_marker_3_.scale.x = scale_car_truck_text(0);
        ID_marker_3_.scale.y = scale_car_truck_text(1);
        ID_marker_3_.scale.z = scale_car_truck_text(2);
        ID_marker_3_.color.b = color_car_truck_text.b;
        ID_marker_3_.color.g = color_car_truck_text.g;
        ID_marker_3_.color.r = color_car_truck_text.r;
        ID_marker_3_.color.a = color_car_truck_text.a;
        geometry_msgs::Pose pose_3;
        pose_3.position.x = x_3;
        pose_3.position.y = y_3;
        pose_3.position.z = alt_3 + height_3 + 3.2;
        ostringstream str_3;
        str_3 << "ID:3 " << "V:" << fixed << setprecision(1) << v_mps_3;
        ID_marker_3_.text = str_3.str();
        ID_marker_3_.pose = pose_3;
        markerPub3.publish(ID_marker_3_);

        // 4.4  周围车辆4
        visualization_msgs::Marker vehicle_marker_4_;
        vehicle_marker_4_.header.frame_id = frame_id;
        vehicle_marker_4_.header.stamp = current_time;
        vehicle_marker_4_.ns = "vehicle";
        vehicle_marker_4_.action = visualization_msgs::Marker::ADD;
        vehicle_marker_4_.type = visualization_msgs::Marker::CUBE;
        vehicle_marker_4_.id = 4;
        // decide the color of the marker
        vehicle_marker_4_.color.a = color_pickup.a;
        vehicle_marker_4_.color.r = color_pickup.r;
        vehicle_marker_4_.color.g = color_pickup.g;
        vehicle_marker_4_.color.b = color_pickup.b;

        double height_4 = 1.675;
        if (abs(width_1 - 4) < 0.2) {
            height_4 = 3.5;
            vehicle_marker_4_.color.a = color_truck_NTE240.a;
            vehicle_marker_4_.color.r = color_truck_NTE240.r;
            vehicle_marker_4_.color.g = color_truck_NTE240.g;
            vehicle_marker_4_.color.b = color_truck_NTE240.b;
        }
        if (abs(width_4 - 6.7) < 0.2) {
            height_4 = 6.9;
            vehicle_marker_4_.color.a = color_truck_NTE200.a;
            vehicle_marker_4_.color.r = color_truck_NTE200.r;
            vehicle_marker_4_.color.g = color_truck_NTE200.g;
            vehicle_marker_4_.color.b = color_truck_NTE200.b;
        }
        vehicle_marker_4_.scale.x = length_4;
        vehicle_marker_4_.scale.y = width_4;
        vehicle_marker_4_.scale.z = height_4;
        // set marker position
        vehicle_marker_4_.pose.position.x = x_4;
        vehicle_marker_4_.pose.position.y = y_4;
        vehicle_marker_4_.pose.position.z = alt_4 + height_4 + 0.4;
        geometry_msgs::Quaternion tf_vehicle_marker_4 = tf::createQuaternionMsgFromYaw(corAngle_4);
        vehicle_marker_4_.pose.orientation.x = tf_vehicle_marker_4.x;
        vehicle_marker_4_.pose.orientation.y = tf_vehicle_marker_4.y;
        vehicle_marker_4_.pose.orientation.z = tf_vehicle_marker_4.z;
        vehicle_marker_4_.pose.orientation.w = tf_vehicle_marker_4.w;
        // set marker action
        vehicle_marker_4_.lifetime = ros::Duration(); //(sec,nsec),0 forever(取消自动删除)
        // ROS_INFO("Publishing Marker with ID: %d", vehicle_marker_4_.id);
        // ROS_INFO("Marker Position: (%f, %f, %f)", vehicle_marker_4_.pose.position.x, vehicle_marker_4_.pose.position.y,
        // vehicle_marker_4_.pose.position.z);

        pubVehicle4.publish(vehicle_marker_4_);

        // 4.4.2 周围车辆4相关信息
        visualization_msgs::Marker ID_marker_4_;
        ID_marker_4_.header.frame_id = frame_id;
        ID_marker_4_.header.stamp = current_time;
        ID_marker_4_.ns = "basic_shapes";
        ID_marker_4_.action = visualization_msgs::Marker::ADD;
        ID_marker_4_.pose.orientation.w = 1.0;
        ID_marker_4_.id = 4;
        ID_marker_4_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ID_marker_4_.scale.x = scale_car_truck_text(0);
        ID_marker_4_.scale.y = scale_car_truck_text(1);
        ID_marker_4_.scale.z = scale_car_truck_text(2);
        ID_marker_4_.color.b = color_car_truck_text.b;
        ID_marker_4_.color.g = color_car_truck_text.g;
        ID_marker_4_.color.r = color_car_truck_text.r;
        ID_marker_4_.color.a = color_car_truck_text.a;
        geometry_msgs::Pose pose_4;
        pose_4.position.x = x_4;
        pose_4.position.y = y_4;
        pose_4.position.z = alt_4 + height_4 + 3.2;
        ostringstream str_4;
        str_4 << "ID:4 " << "V:" << fixed << setprecision(1) << v_mps_4;
        ID_marker_4_.text = str_4.str();
        ID_marker_4_.pose = pose_4;
        markerPub4.publish(ID_marker_4_);

        view_controller_msgs::CameraPlacement camera;
        geometry_msgs::Point pt;
        pt.x = update_eye_x;
        pt.y = update_eye_y;
        pt.z = z_eye;
        camera.eye.point = pt;
        camera.eye.header.frame_id = "base_link";
        pt.x = x_foc;
        pt.y = y_foc;
        pt.z = z_foc;
        camera.focus.point = pt;
        camera.focus.header.frame_id = "base_link";
        update_eye_x = update_eye_x + update_v * cos(corAngle_ego + PI);
        update_eye_y = update_eye_y + update_v * sin(corAngle_ego + PI);

        geometry_msgs::Vector3 up;
        up.x = 0;
        up.y = 0;
        up.z = 10;
        camera.up.vector = up;
        camera.up.header.frame_id = "base_link";
        camera.time_from_start = ros::Duration();
        rviz_camera_pub.publish(camera);

        loop_rate.sleep();
        ros::spinOnce();
    }

    // 关闭打开的文件
    inFile.close();
    outfile.close();

    cout << "file2enu stop!" << endl;
    return 0;
}