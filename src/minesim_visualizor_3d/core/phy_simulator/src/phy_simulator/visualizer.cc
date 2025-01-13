#include "phy_simulator/visualizer.h" // 引入物理仿真器可视化类头文件

#include <tf/tf.h>                    // 引入ROS的tf库，用于坐标变换
#include <tf/transform_broadcaster.h> // 引入ROS的TransformBroadcaster类，用于广播坐标变换

namespace phy_simulator { // 定义phy_simulator命名空间

// 可视化类构造函数，初始化ROS节点句柄
Visualizer::Visualizer(ros::NodeHandle nh) : nh_(nh) {
    // 初始化车辆集合的可视化发布器，发布到"vis/vehicle_set_vis"话题
    vehicle_set_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/vehicle_set_vis", 10);
    // 初始化车道网络的可视化发布器，发布到"vis/lane_net_vis"话题
    lane_net_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/lane_net_vis", 10);
    // 初始化障碍物集合的可视化发布器，发布到"vis/border_set_vis"话题
    obstacle_set_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("vis/border_set_vis", 10);
}

// 可视化数据的主函数
void Visualizer::VisualizeData() {
    auto time_stamp = ros::Time::now(); // 获取当前时间戳
    VisualizeDataWithStamp(time_stamp); // 调用带时间戳的可视化函数
    // SendTfWithStamp(time_stamp);  // 发送坐标变换信息
}

// 使用时间戳可视化数据
void Visualizer::VisualizeDataWithStamp(const ros::Time &stamp) {
    // 可视化车辆集合
    VisualizeVehicleSet(stamp, p_phy_sim_->vehicle_set());
    // 可视化车道网络
    VisualizeLaneNet(stamp, p_phy_sim_->lane_net());
    // 可视化障碍物集合(障碍物集合 包括道路边界)
    VisualizeObstacleSet(stamp, p_phy_sim_->obstacle_set());
}

//   用于发送tf变换信息的函数
// void Visualizer::SendTfWithStamp(const ros::Time &stamp) {
//   auto vehicle_set = p_phy_sim_->vehicle_set();
//   for (auto iter = vehicle_set.vehicles.begin();
//        iter != vehicle_set.vehicles.end(); ++iter) {
//     static tf::TransformBroadcaster tf_broadcaster;
//     Vec3f state = iter->second.Ret3DofState();
//     geometry_msgs::Pose pose;
//     common::VisualizationUtil::GetRosPoseFrom3DofState(state, &pose);

//     std::string tf_name = std::string("body_") + std::to_string(iter->first);
//     tf_broadcaster.sendTransform(tf::StampedTransform(
//         tf::Transform(
//             tf::Quaternion(pose.orientation.x, pose.orientation.y,
//                            pose.orientation.z, pose.orientation.w),
//             tf::Vector3(pose.position.x, pose.position.y, pose.position.z)),
//         stamp, "map", tf_name));
//   }
// }

// 使用时间戳可视化车辆集合
void Visualizer::VisualizeVehicleSet(const ros::Time &stamp, const common::VehicleSet &vehicle_set) {
    visualization_msgs::MarkerArray vehicle_marker;      // 创建一个Marker数组，用于存储可视化信息
    common::ColorARGB color_obb(1.0, 0.8, 0.8, 0.8);     // 定义物体颜色（透明灰色） rgb(156, 34, 34)
    common::ColorARGB color_vel_vec(1.0, 1.0, 0.0, 0.0); // 定义速度向量颜色（黄色）rgb(255, 0, 0)
    common::ColorARGB color_steer(1.0, 1.0, 1.0, 1.0);   // 定义转向颜色（白色）rgb(255, 255, 255)
    for (auto iter = vehicle_set.vehicles.begin(); iter != vehicle_set.vehicles.end(); ++iter) {
        // 获取车辆的Marker数组，并使用指定的颜色进行标记
        common::VisualizationUtil::GetRosMarkerArrayUsingVehicle(iter->second, common::ColorARGB(1, 1, 0, 0), color_vel_vec, color_steer,
                                                                 7 * iter->first, &vehicle_marker);
    }
    // 填充时间戳信息到Marker数组
    common::VisualizationUtil::FillStampInMarkerArray(stamp, &vehicle_marker);
    // 发布车辆集合的可视化信息
    vehicle_set_pub_.publish(vehicle_marker);
}

// 使用时间戳可视化车道网络
void Visualizer::VisualizeLaneNet(const ros::Time &stamp, const common::LaneNet &lane_net) {
    visualization_msgs::MarkerArray lane_net_marker; // 创建Marker数组用于存储车道网络可视化信息
    int id_cnt = 0;                                  // 初始化Marker ID计数器
    for (auto iter = lane_net.lane_set.begin(); iter != lane_net.lane_set.end(); ++iter) {
        visualization_msgs::Marker lane_marker; // 创建一个Marker用于表示车道
        // 生成车道的可视化Marker  color rgb(51,153,255)
        common::VisualizationUtil::GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
            iter->second.lane_points, common::color_map_minesim.at("lane_centerline"), common::object_scale_map_minesim.at("lane_centerline"),
            iter->second.heights, iter->second.id, &lane_marker);
        lane_marker.header.stamp = stamp;               // 设置Marker的时间戳
        lane_marker.header.frame_id = "map";            // 设置Marker的坐标系为“map”
        lane_marker.id = id_cnt++;                      // 设置Marker的唯一ID
        lane_net_marker.markers.push_back(lane_marker); // 将车道Marker加入车道网络Marker数组

        // 可视化车道的起点和终点，以及车道ID文本
        visualization_msgs::Marker start_point_marker, end_point_marker, lane_id_text_marker;
        {
            // 设置起点Marker
            start_point_marker.header.stamp = stamp;
            start_point_marker.header.frame_id = "map";
            Vec2f pt = *(iter->second.lane_points.begin());

            common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
                Vec3f(pt(0), pt(1), iter->second.heights.front()), common::color_map_minesim.at("lane_centerline_start_end_point"),
                common::object_scale_map_minesim.at("lane_centerline_start_end_point"), id_cnt++, &start_point_marker);

            lane_id_text_marker.header.stamp = stamp;
            lane_id_text_marker.header.frame_id = "map";
            common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
                Vec3f(pt(0), pt(1), iter->second.heights.front() + 0.5), std::to_string(iter->second.id),
                common::color_map_minesim.at("lane_centerline_start_end_point_text"),
                common::object_scale_map_minesim.at("lane_centerline_start_end_point_text"), id_cnt++, &lane_id_text_marker);
        }
        // {
        // 设置终点Marker
        // end_point_marker.header.stamp = stamp;
        // end_point_marker.header.frame_id = "map";
        // Vec2f pt = *(iter->second.lane_points.rbegin());
        // a, r, g, b = 1.0, 0.2, 0.6, 1.0 # rgb(51, 153, 255)
        // common::VisualizationUtil::GetRosMarkerSphereUsingPoint(
        //     Vec3f(pt(0), pt(1), iter->second.heights.back()), common::color_map_minesim.at("lane_centerline_start_end_point"),
        //     common::object_scale_map_minesim.at("lane_centerline_start_end_point"), id_cnt++, &end_point_marker);
        // }

        lane_net_marker.markers.push_back(start_point_marker); // 将起点Marker加入Marker数组
        // lane_net_marker.markers.push_back(end_point_marker);    // 将终点Marker加入Marker数组
        lane_net_marker.markers.push_back(lane_id_text_marker); // 将车道ID文本Marker加入Marker数组
    }
    // 发布车道网络的可视化信息
    lane_net_pub_.publish(lane_net_marker);
}

// 使用时间戳可视化障碍物集合,包括道路边界
void Visualizer::VisualizeObstacleSet(const ros::Time &stamp, const common::ObstacleSet &Obstacle_set) {
    visualization_msgs::MarkerArray Obstacles_marker; // 创建Marker数组用于存储障碍物可视化信息
    common::VisualizationUtil::GetRosMarkerUsingObstacleSet(Obstacle_set, &Obstacles_marker); // 获取障碍物的Marker数组
    common::VisualizationUtil::FillStampInMarkerArray(stamp, &Obstacles_marker);              // 填充时间戳到Marker数组
    obstacle_set_pub_.publish(Obstacles_marker);                                              // 发布障碍物集合的可视化信息
}

} // namespace phy_simulator
