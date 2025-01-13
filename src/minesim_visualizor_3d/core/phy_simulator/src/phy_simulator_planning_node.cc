#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/decoder.h"

#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"
#include "phy_simulator/ros_adapter.h"
#include "phy_simulator/visualizer.h"

using namespace phy_simulator;

DECLARE_BACKWARD;
const double simulation_rate = 500.0;
const double gt_msg_rate = 100.0;
const double gt_static_msg_rate = 10.0;
const double visualization_msg_rate = 20.0;

common::VehicleControlSignalSet _signal_set;
std::vector<ros::Subscriber> _ros_sub;

Vec3f initial_state(0, 0, 0);
bool flag_rcv_initial_state = false;

Vec3f goal_state(0, 0, 0);
bool flag_rcv_goal_state = false;

void CtrlSignalCallback(const vehicle_msgs::ControlSignal::ConstPtr &msg, int index) {
    common::VehicleControlSignal ctrl;
    vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
    _signal_set.signal_set[index] = ctrl;
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    // 初始坐标在ros坐标系下的3d坐标转换
    common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose, &initial_state);
    flag_rcv_initial_state = true;
}

void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state); // 目标位置在ros坐标系下的3d坐标转换
    flag_rcv_goal_state = true;
}

// 读取 `config_file。json`中的 参数
void readConfig(bool &altInfoVisualization, const std::string &config_file_param) {
    // 打印传入的config_file路径参数
    printf("#log# Provided config_file param: %s\n", config_file_param.c_str());

    // 检查传入路径是否为空
    if (config_file_param.empty()) {
        std::cerr << "#log# Error: config_file parameter is empty.\n";
        throw std::runtime_error("Empty config_file parameter.");
    }

    // 转换传入的路径为boost的path
    boost::filesystem::path config_file_path(config_file_param);
    if (!boost::filesystem::exists(config_file_path)) {
        std::cerr << "#log# Error: Config file does not exist at: " << config_file_param << std::endl;
        throw std::runtime_error("Config file not found.");
    }

    // 打开JSON文件并检查是否成功打开
    std::ifstream fs(config_file_path.string());
    if (!fs.is_open()) {
        std::cerr << "#log# Error: Failed to open config file: " << config_file_path.string() << std::endl;
        throw std::runtime_error("Failed to open config file.");
    }

    // 解析JSON内容
    nlohmann::json jsonData;
    fs >> jsonData;

    // 读取并设置 altInfoVisualization
    if (jsonData.contains("alt_info_visualization")) {
        if (jsonData["alt_info_visualization"].is_boolean()) {
            altInfoVisualization = jsonData["alt_info_visualization"].get<bool>();
        } else if (jsonData["alt_info_visualization"].is_string()) {
            std::string valueStr = jsonData["alt_info_visualization"];
            if (valueStr == "true") {
                altInfoVisualization = true;
            } else if (valueStr == "false") {
                altInfoVisualization = false;
            } else {
                std::cerr << "#log# Error: Invalid value for alt_info_visualization in config file.\n";
                throw std::runtime_error("Invalid value for alt_info_visualization.");
            }
        } else {
            std::cerr << "#log# Error: alt_info_visualization has an unsupported type in config file.\n";
            throw std::runtime_error("Unsupported type for alt_info_visualization.");
        }
    } else {
        std::cerr << "#log# Warning: alt_info_visualization not found in config file. Defaulting to false.\n";
        altInfoVisualization = false;
    }

    // 打印解析结果，指示是否启用高度信息可视化
    // 如果 altInfoVisualization 为 true，则输出 "true"，否则输出 "false"
    printf("#log# [PhySimulation]Success parse altInfoVisualization (is visualize altitude info on map): %s\n",
           altInfoVisualization ? "true" : "false");
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "~");
    ros::NodeHandle nh("~");

    // 获取几个JSON文件的路径
    std::string vehicle_info_path;
    // 获取参数“vehicle_info_path”的value，写入到vehicle_info_path上
    if (!nh.getParam("vehicle_info_path", vehicle_info_path)) {
        ROS_ERROR("#log# Failed to get param %s", vehicle_info_path.c_str());
        assert(false);
    }
    std::string map_path;
    if (!nh.getParam("map_path", map_path)) {
        ROS_ERROR("#log# Failed to get param %s", map_path.c_str());
        assert(false);
    }
    std::string lane_net_path;
    if (!nh.getParam("lane_net_path", lane_net_path)) {
        ROS_WARN("#log# Failed to get param %s", lane_net_path.c_str());
        assert(false);
    }

    std::string config_file;
    if (!nh.getParam("config_file", config_file)) {
        ROS_WARN("#log# Failed to get param %s", config_file.c_str());
        assert(false);
    }

    // 初始化配置参数
    bool altInfoVisualization = false;
    // 读取配置文件
    try {
        readConfig(altInfoVisualization, config_file);
    } catch (const std::exception &e) {
        ROS_ERROR("#log# Error reading config file: %s", e.what());
        return -1;
    }

    // 初始化仿真器
    // 模拟器数据类实例化，加载json配置文件
    PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path, altInfoVisualization);

    // 配置ROS适配器
    RosAdapter ros_adapter(nh);
    ros_adapter.set_phy_sim(&phy_sim);

    // 设置可视化工具
    Visualizer visualizer(nh); // 创建arena_info_static和arena_info_dynamic广播句柄
    visualizer.set_phy_sim(&phy_sim);

    // 配置车辆控制信号订阅
    auto vehicle_ids = phy_sim.vehicle_ids(); // 存有车辆id的vector数组
    int num_vehicles = static_cast<int>(vehicle_ids.size());
    _ros_sub.resize(num_vehicles); // 分配容器的内存大小

    // 打印每辆车的订阅话题
    for (int i = 0; i < num_vehicles; i++) {
        auto vehicle_id = vehicle_ids[i];
        std::string topic_name = std::string("/ctrl/agent_") + std::to_string(vehicle_id);
        // c_str()就是将C++的string转化为C的字符串数组，c_str()生成一个const char *指针，指向字符串的首地址
        printf("#log# subscribing to vehicle_id, %s\n", topic_name.c_str());
        _ros_sub[i] = nh.subscribe<vehicle_msgs::ControlSignal>(topic_name, 10, boost::bind(CtrlSignalCallback, _1, vehicle_id));
    }

    // 给车辆控制信号集插入车辆id和加速度，转角变化率等信息
    for (auto &vehicle_id : vehicle_ids) { // C++基于范围的for循环：for(auto a:b)
        common::VehicleControlSignal default_signal;
        _signal_set.signal_set.insert(std::pair<int, common::VehicleControlSignal>(vehicle_id, default_signal));
    }

    ros::Subscriber ini_pos_sub = nh.subscribe("/initialpose", 10, InitialPoseCallback);
    ros::Subscriber goal_pos_sub = nh.subscribe("/move_base_simple/goal", 10, NavGoalCallback);

    ros::Rate rate(simulation_rate);
    ros::Time next_gt_pub_time = ros::Time::now(); // 下一个**发布的时间的时间
    ros::Time next_gt_static_pub_time = next_gt_pub_time;
    ros::Time next_vis_pub_time = ros::Time::now();

    std::cout << "#log# [PhySimulation] Initialization finished, waiting for callback" << std::endl;

    int gt_msg_counter = 0;
    while (ros::ok()) {
        ros::spinOnce();

        phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate); // 利用控制信号更新仿真器

        ros::Time tnow = ros::Time::now(); // 程序当前时间
        if (tnow >= next_gt_pub_time) {
            next_gt_pub_time += ros::Duration(1.0 / gt_msg_rate); // 当前时刻+0.01s
            ros_adapter.PublishDynamicDataWithStamp(tnow);
        }

        if (tnow >= next_gt_static_pub_time) {
            next_gt_static_pub_time += ros::Duration(1.0 / gt_static_msg_rate); // 静态障碍物发布频率
            ros_adapter.PublishStaticDataWithStamp(tnow);
        }

        if (tnow >= next_vis_pub_time) {
            next_vis_pub_time += ros::Duration(1.0 / visualization_msg_rate); // 可视化程序发布频率
            visualizer.VisualizeDataWithStamp(tnow);
            // std::cout << "#log# [PhySimulation] updae visualize." << std::endl;
        }

        rate.sleep();
    }

    _ros_sub.clear();
    ros::shutdown();
    return 0;
}
