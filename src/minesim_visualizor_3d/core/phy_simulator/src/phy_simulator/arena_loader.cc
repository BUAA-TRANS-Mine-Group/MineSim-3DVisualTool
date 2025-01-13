/**
 * @file arena_loader.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#include "phy_simulator/arena_loader.h"
#include <ros/console.h> // 包含 ROS_*_STREAM 宏

namespace phy_simulator {

using Json = nlohmann::json; // 解析 json 文件

ArenaLoader::ArenaLoader() {}

ArenaLoader::ArenaLoader(const std::string &vehicle_set_path, const std::string &map_path, const std::string &lane_net_path)
    : vehicle_set_path_(vehicle_set_path), map_path_(map_path), lane_net_path_(lane_net_path) {}

bool ArenaLoader::ParseVehicleSet(common::VehicleSet *p_vehicle_set) {
    std::cout << "#log# [ArenaLoader]Loading vehicle set file from path: " << vehicle_set_path_ << "\n" << std::endl;

    // 1. **检查文件路径和存在性**
    std::fstream fs(vehicle_set_path_);
    if (!fs.is_open()) {
        ROS_ERROR_STREAM("#log# Failed to open vehicle set file: " + vehicle_set_path_);
        return false; // 如果文件无法打开，返回错误
    }

    // 3. **JSON 解析检查** // 确保 JSON 格式正确。如果格式错误，捕获解析错误并输出错误信息。
    Json root;
    try {
        fs >> root; // 尝试读取文件内容
    } catch (const Json::parse_error &e) {
        ROS_ERROR_STREAM("#log# Failed to parse vehicle set JSON: " + std::string(e.what()));
        return false; // 如果解析失败，捕获异常并返回错误
    }

    // 加载车辆信息
    Json vehicles_json = root["vehicles"]["info"];
    if (!vehicles_json.is_array()) {
        ROS_ERROR_STREAM("#log# Invalid format: vehicles->info should be an array.");
        return false; // 检查 "info" 字段是否为数组
    }

    for (int i = 0; i < static_cast<int>(vehicles_json.size()); ++i) {
        common::Vehicle vehicle;
        vehicle.set_id(vehicles_json[i]["id"].get<int>());
        vehicle.set_subclass(vehicles_json[i]["subclass"].get<std::string>());
        vehicle.set_type(vehicles_json[i]["type"].get<std::string>());

        Json state_json = vehicles_json[i]["init_state"];
        common::State state;
        state.vec_position(0) = state_json["x"].get<double>();
        state.vec_position(1) = state_json["y"].get<double>();
        state.height = state_json["angle"].get<double>();
        state.angle = state_json["angle"].get<double>();
        state.curvature = state_json["curvature"].get<double>();
        state.velocity = state_json["velocity"].get<double>();
        state.acceleration = state_json["acceleration"].get<double>();
        state.steer = state_json["steer"].get<double>();
        vehicle.set_state(state);

        // 设置车辆参数
        Json params_json = vehicles_json[i]["params"];
        common::VehicleParam param;
        param.set_width(params_json["width"].get<double>());
        param.set_length(params_json["length"].get<double>());
        param.set_wheel_base(params_json["wheel_base"].get<double>());
        param.set_front_suspension(params_json["front_suspension"].get<double>());
        param.set_rear_suspension(params_json["rear_suspension"].get<double>());
        auto max_steering_angle = params_json["max_steering_angle"].get<double>();
        param.set_max_steering_angle(max_steering_angle * kPi / 180.0);
        param.set_max_longitudinal_acc(params_json["max_longitudinal_acc"].get<double>());
        param.set_max_lateral_acc(params_json["max_lateral_acc"].get<double>());

        param.set_d_cr(param.length() / 2 - param.rear_suspension());
        vehicle.set_param(param);

        p_vehicle_set->vehicles.insert(std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
    }

    fs.close();
    printf("#log# [ArenaLoader] Load vehicle set successful.\n");
    return true;
}

ErrorType ArenaLoader::ParseMapInfo(common::ObstacleSet *p_border_set, bool use_actual_heights) {
    std::cout << "#log# [ArenaLoader]Loading map file from path: " << map_path_ << "\n" << std::endl;

    // 1. **检查文件路径和存在性**
    std::fstream fs(map_path_);
    if (!fs.is_open()) {
        ROS_ERROR_STREAM(std::string("#log# Failed to open map file: ") + map_path_);
        return kErrorFileOpenFailed; // 如果地图文件无法打开，返回错误
    }

    // 2. **文件内容检查** 确保文件不是空的。如果文件为空，直接返回错误。
    // std::string file_content((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());
    // if (file_content.empty()) {
    //     ROS_ERROR_STREAM(std::string("#log# Map file is empty: ") + map_path_);
    //     return kErrorFileEmpty; // 如果文件内容为空，返回错误
    // }

    // 3. **JSON 解析检查** // 确保 JSON 格式正确。如果格式错误，捕获解析错误并输出错误信息。
    Json root;  // Json对象
    if (true) { // flase,不做检查，好像有点问题； true,做检查
        try {
            fs >> root; // 解析 JSON 数据
        } catch (const Json::parse_error &e) {
            ROS_ERROR_STREAM("#log# Failed to parse map JSON: " + std::string(e.what()));
            return kErrorJsonParseFailed; // 如果解析失败，返回错误
        }
    } else
        fs >> root; // 解析 JSON 数据

    // 4. **格式检查：`borderline`** ；确保 `"borderline"` 字段是一个数组，并且其中的每个元素都有正确的结构。
    Json borders_net_json = root["borderline"]; // borderline字段
    if (!borders_net_json.is_array()) {
        ROS_ERROR_STREAM("#log# Invalid format: borderline should be an array.");
        return kErrorInvalidFormat; // 如果 "borderline" 不是数组，返回错误
    }
    for (int i = 0; i < static_cast<int>(borders_net_json.size()); ++i) {
        Json border_json = borders_net_json[i]; // 找到其中的某条边界
        // 确保 borderpoints 存在且格式正确
        if (!border_json.contains("borderpoints")) {
            ROS_ERROR_STREAM("#log# Missing 'borderpoints' field in borderline entry.");
            return kErrorInvalidFormat;
        }

        common::PolygonObstacle poly; // poly存储边界信息
        std::vector<std::string> tmp_id;
        common::SplitString(border_json["token"], "-", &tmp_id);
        std::string com_id = "100" + tmp_id[1]; // 边界（border）id用200前缀表示
        poly.id = std::stoi(com_id);            // 返回id为int型

        Json coord = border_json["borderpoints"];
        if (!coord.is_array()) {
            ROS_ERROR_STREAM("#log# 'borderpoints' should be an array.");
            return kErrorInvalidFormat;
        }
        int num_pts = static_cast<int>(coord.size());
        for (int k = 0; k < num_pts; ++k) {
            if (coord[k].size() < 2) { // 检查每个边界点是否有至少两个坐标
                ROS_ERROR_STREAM("#log# Invalid borderpoint: expected at least 2 coordinates.");
                return kErrorInvalidFormat;
            }

            common::Point point(coord[k][0].get<double>(), coord[k][1].get<double>());
            double height = use_actual_heights ? coord[k][2].get<double>() : 1.0;
            poly.polygon.points.emplace_back(point);   // 坐标点
            poly.polygon.heights.emplace_back(height); // 高度信息
        }
        p_border_set->obs_polygon.insert(std::pair<int, common::PolygonObstacle>(poly.id, poly)); // 将处理好的边界对象添加到边界集合
    }
    // p_border_set->print();

    fs.close();
    printf("#log# [ArenaLoader] Load  map file successful.\n");
    return kSuccess;
}

ErrorType ArenaLoader::ParseLaneNetInfo(common::LaneNet *p_lane_net, bool use_actual_heights) {
    std::cout << "#log# [ArenaLoader]Loading lane_net file from path: " << lane_net_path_ << "\n" << std::endl;

    // 1. **检查文件路径和存在性**
    std::fstream fs(lane_net_path_);
    if (!fs.is_open()) {
        ROS_ERROR_STREAM("#log# Failed to open lane net file: " + lane_net_path_);
        return kErrorFileOpenFailed; // 如果文件无法打开，返回错误
    }

    Json root;
    try {
        fs >> root; // 解析 JSON 数据
    } catch (const Json::parse_error &e) {
        ROS_ERROR_STREAM("#log# Failed to parse lane net JSON: " + std::string(e.what()));
        return kErrorJsonParseFailed; // 如果解析失败，返回错误
    }

    Json lane_net_json = root["reference_path"]; // 所有路线集合
    if (!lane_net_json.is_array()) {
        ROS_ERROR_STREAM("#log# Invalid format: reference_path should be an array.");
        return kErrorInvalidFormat; // 如果 "reference_path" 不是数组，返回错误
    }

    for (int i = 0; i < static_cast<int>(lane_net_json.size()); ++i) {
        common::LaneRaw lane_raw;
        Json lane_json = lane_net_json[i]; // 找到其中某条路线

        std::vector<std::string> tmp_id;
        common::SplitString(lane_json["token"], "-", &tmp_id);
        std::string com_id = "100" + tmp_id[1]; // 道路（road）id用100前缀表示，连接区域（connectivity）用101表示
        lane_raw.id = std::stoi(com_id);        // 返回id为int型
        // printf("#log# -Lane id %d.\n", lane_raw.id);

        Json lane_coordinates = lane_json["waypoints"];          // 所有的坐标数据
        int num_pts = static_cast<int>(lane_coordinates.size()); // 坐标点数量
        // decimal_t lane_coordinates_tmp_height = 0;
        for (int k = 0; k < num_pts; ++k) {
            Vec2f pt(lane_coordinates[k][0].get<double>(), // 获取坐标点第1个坐标
                     lane_coordinates[k][1].get<double>()  // 获取坐标点第2个坐标
            );
            double height = use_actual_heights ? lane_coordinates[k][3].get<double>() : 1.0;
            lane_raw.heights.emplace_back(height);
            lane_raw.lane_points.emplace_back(pt);
            // lane_coordinates_tmp_height -= 0.01;
        }
        lane_raw.start_point = *(lane_raw.lane_points.begin());  // 该段路径的开始坐标点
        lane_raw.final_point = *(lane_raw.lane_points.rbegin()); // 该段路径的结束坐标点
        p_lane_net->lane_set.insert(std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
    }
    // p_lane_net->print();

    fs.close();
    printf("#log# [ArenaLoader] Load lane_net file successful.\n");
    return kSuccess;
}

} // namespace phy_simulator
