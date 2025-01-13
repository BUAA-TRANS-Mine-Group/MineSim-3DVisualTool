/**
 * @file colormap.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/colormap.h"
namespace common {
std::map<std::string, ColorARGB> cmap{
    {"black", ColorARGB(1.0, 0.0, 0.0, 0.0)},               // rgb(0, 0, 0)
    {"white", ColorARGB(1.0, 1.0, 1.0, 1.0)},               // rgb(255, 255, 255)
    {"red", ColorARGB(1.0, 1.0, 0.0, 0.0)},                 // rgb(255, 0, 0)
    {"hot pink", ColorARGB(1.0, 1.0, 0.44, 0.70)},          // rgb(255, 112, 178)
    {"green", ColorARGB(1.0, 0.0, 1.0, 0.0)},               // rgb(0, 255, 0)
    {"blue", ColorARGB(1.0, 0.0, 0.0, 1.0)},                // rgb(0, 0, 255)
    {"aqua marine", ColorARGB(1.0, 0.5, 1.0, 0.83)},        // rgb(128, 255, 212)
    {"yellow", ColorARGB(1.0, 1.0, 1.0, 0.0)},              // rgb(255, 255, 0)
    {"cyan", ColorARGB(1.0, 0.0, 1.0, 1.0)},                // rgb(0, 255, 255)
    {"magenta", ColorARGB(1.0, 1.0, 0.0, 1.0)},             // rgb(255, 0, 255)
    {"violet", ColorARGB(1.0, 0.93, 0.43, 0.93)},           // rgb(237, 110, 237)
    {"orange red", ColorARGB(1.0, 1.0, 0.275, 0.0)},        // rgb(255, 70, 0)
    {"orange", ColorARGB(1.0, 1.0, 0.65, 0.0)},             // rgb(255, 165, 0)
    {"dark orange", ColorARGB(1.0, 1.0, 0.6, 0.0)},         // rgb(255, 153, 0)
    {"gold", ColorARGB(1.0, 1.0, 0.84, 0.0)},               // rgb(255, 214, 0)
    {"green yellow", ColorARGB(1.0, 0.5, 1.0, 0.0)},        // rgb(128, 255, 0)
    {"forest green", ColorARGB(1.0, 0.13, 0.545, 0.13)},    // rgb(34, 139, 34)
    {"spring green", ColorARGB(1.0, 0.0, 1.0, 0.5)},        // rgb(0, 255, 128)
    {"sky blue", ColorARGB(1.0, 0.0, 0.749, 1.0)},          // rgb(0, 191, 255)
    {"medium orchid", ColorARGB(1.0, 0.729, 0.333, 0.827)}, // rgb(186, 85, 211)
    {"grey", ColorARGB(1.0, 0.5, 0.5, 0.5)}                 // rgb(128, 128, 128)
};

// jet map
std::map<decimal_t, ColorARGB> jet_map{
    {0.0, ColorARGB(1.0, 0.0, 0.0, 0.6667)},       // rgb(0, 0, 170)
    {0.1, ColorARGB(1.0, 0.0, 0.0, 1.0000)},       // rgb(0, 0, 255)
    {0.2, ColorARGB(1.0, 0.0, 0.3333, 1.0000)},    // rgb(0, 0, 255)
    {0.3, ColorARGB(1.0, 0.0, 0.6667, 1.0000)},    // rgb(0, 0, 255)
    {0.4, ColorARGB(1.0, 0.0, 1.0000, 1.0000)},    // rgb(0, 0, 255)
    {0.5, ColorARGB(1.0, 0.3333, 1.0000, 0.6667)}, // rgb(85, 255, 170)
    {0.6, ColorARGB(1.0, 0.6667, 1.0000, 0.3333)}, // rgb(170, 255, 85)
    {0.7, ColorARGB(1.0, 1.0000, 1.0000, 0.0)},    // rgb(255, 255, 0)
    {0.8, ColorARGB(1.0, 1.0000, 0.6667, 0.0)},    // rgb(255, 170, 0)
    {0.9, ColorARGB(1.0, 1.0000, 0.3333, 0.0)},    // rgb(255, 85, 0)
    {1.0, ColorARGB(1.0, 1.0000, 0.0, 0.0)}        // rgb(255, 0, 0)
};

// autumn map
std::map<decimal_t, ColorARGB> autumn_map{{1.0, ColorARGB(0.5, 1.0, 0.0, 0.0)},   // rgb(128, 0, 0)
                                          {0.95, ColorARGB(0.5, 1.0, 0.05, 0.0)}, // rgb(128, 13, 0)
                                          {0.9, ColorARGB(0.5, 1.0, 0.1, 0.0)},   // rgb(128, 26, 0)
                                          {0.85, ColorARGB(0.5, 1.0, 0.15, 0.0)}, // rgb(128, 38, 0)
                                          {0.8, ColorARGB(0.5, 1.0, 0.2, 0.0)},   // rgb(128, 51, 0)
                                          {0.75, ColorARGB(0.5, 1.0, 0.25, 0.0)}, // rgb(128, 64, 0)
                                          {0.7, ColorARGB(0.5, 1.0, 0.3, 0.0)},   // rgb(128, 77, 0)
                                          {0.65, ColorARGB(0.5, 1.0, 0.35, 0.0)}, // rgb(128, 90, 0)
                                          {0.6, ColorARGB(0.5, 1.0, 0.4, 0.0)},   // rgb(128, 102, 0)
                                          {0.55, ColorARGB(0.5, 1.0, 0.45, 0.0)}, // rgb(128, 115, 0)
                                          {0.5, ColorARGB(0.5, 1.0, 0.5, 0.0)},   // rgb(128, 128, 0)
                                          {0.45, ColorARGB(0.5, 1.0, 0.55, 0.0)}, // rgb(128, 140, 0)
                                          {0.4, ColorARGB(0.5, 1.0, 0.6, 0.0)},   // rgb(128, 153, 0)
                                          {0.35, ColorARGB(0.5, 1.0, 0.65, 0.0)}, // rgb(128, 166, 0)
                                          {0.3, ColorARGB(0.5, 1.0, 0.7, 0.0)},   // rgb(128, 179, 0)
                                          {0.25, ColorARGB(0.5, 1.0, 0.75, 0.0)}, // rgb(128, 191, 0)
                                          {0.2, ColorARGB(0.5, 1.0, 0.8, 0.0)},   // rgb(128, 204, 0)
                                          {0.15, ColorARGB(0.5, 1.0, 0.85, 0.0)}, // rgb(128, 217,195)
                                          {0.1, ColorARGB(0.5, 1.0, 0.9, 0.0)},   //
                                          {0.05, ColorARGB(0.5, 1.0, 0.95, 0.0)}, //
                                          {0.0, ColorARGB(0.5, 1.0, 1.0, 0.0)}};  //

// TODO ==================== 此处用于 配置ros rviz 仿真器中的元素 粗细等; Vec3f(0.3, 0.3, 0.1)
// TODO ==================== RosMarker: scale 设置 ====================
//  传入的 `scale` 是一个 `Vec3f` * 类型的对象，它包含了三个浮动值，分别控制物体或线条在 X、Y、Z轴的 缩放系数,:1表示1米。
//  - `scale.x = 0.1`  影响物体或线条的 宽度（X轴方向），例如，绘制线条时控制线条的粗细。
//  - `scale.y = 0.1`  影响物体或线条的 高度（Y轴方向），通常与 scale.x 配合影响可视化的二维物体形态。
//  - `scale.z = 0.0` 影响物体的 深度（Z轴方向），例如，绘制线条时有时也用来影响线条的厚度（有时设置为1或者0来表示标准深度）。
// 取值范围[0,inf)
// common::ColorARGB(1.0, 0.2, 0.6, 1.0)颜色设置;ARGB模式;取值(0.0,1.0),表示 透明度,R,G,B值
//  ==================== RosMarker: scale 设置 ====================
// [using method] common::object_scale_map_minesim.at("lane_centerline")
std::map<std::string, Vec3f> object_scale_map_minesim{
    {"ego_mine_truck", Vec3f(0.1, 0.1, 0.1)},                       // object scales
    {"ego_ref_path", Vec3f(1.2, 0.4, 0.1)},                         // object scales:ego_ref_path高度为 lane_centerline一半合适
    {"lane_centerline", Vec3f(0.4, 0.4, 0.2)},                      // object scales
    {"lane_centerline_start_end_point", Vec3f(0.8, 0.8, 0.4)},      // object scales
    {"lane_centerline_start_end_point_text", Vec3f(1.0, 1.0, 1.5)}, // object scales
    {"border_line", Vec3f(0.6, 0.6, 1.2)},                          // object scales
    {"surrounding_car_pickup", Vec3f(0.6, 0.6, 1.2)},               // object scales
    {"surrounding_truck_NTE240", Vec3f(0.6, 0.6, 1.2)},             // object scales
    {"surrounding_truck_NTE200", Vec3f(0.6, 0.6, 1.2)},             // object scales
    {"surrounding_car_truck_text", Vec3f(1.5, 1.5, 1.5)},           // object scales

};

// TODO ==================== 此处用于 配置ros rviz 仿真器中的元素 颜色等;
// [using method] common::color_map_minesim.at("lane_centerline")
// 除法 205 / 255 等会进行整数除法，结果会是 0（如果没有强制转换），从而导致颜色值错误。你需要确保使用浮点数除法（即 205.0 / 255.0 等）。
std::map<std::string, ColorARGB> color_map_minesim{
    {"ego_mine_truck", ColorARGB(0.7, 205 / 255.0, 159 / 255.0, 0.0 / 255.0)},                      // rgba(205, 159, 0, 0.7)
    {"ego_ref_path", ColorARGB(0.7, 0.0 / 255.0, 150 / 255.0, 100 / 255.0)},                        // rgba(0, 150, 100, 0.7)
    {"lane_centerline", ColorARGB(0.9, 0.0 / 255.0, 200 / 255.0, 150 / 255.0)},                     // rgba(0, 200, 150, 0.9)
    {"lane_centerline_start_end_point", ColorARGB(0.8, 0.0 / 255.0, 100 / 255.0, 40 / 255.0)},      // rgba(0, 100, 40, 0.8)
    {"lane_centerline_start_end_point_text", ColorARGB(0.9, 0.0 / 255.0, 100 / 255.0, 40 / 255.0)}, // rgba(0, 100, 40, 0.9)
    {"border_line", ColorARGB(0.8, 70.0 / 255.0, 70.0 / 255.0, 70.0 / 255.0)},                      // rgba(70, 70, 70, 0.99)
    {"surrounding_car_pickup", ColorARGB(0.8, 0.0 / 225.0, 50.0 / 225.0, 200 / 255.0)},             // rgba(0, 50, 200, 0.99)
    {"surrounding_truck_NTE240", ColorARGB(0.6, 0.0 / 225.0, 50 / 225.0, 180 / 255.0)},             // rgba(0, 50, 180,0.99)
    {"surrounding_truck_NTE200", ColorARGB(0.6, 0.0 / 225.0, 65 / 225.0, 180 / 255.0)},             // rgba(0, 65, 180, 0.99)
    {"surrounding_car_truck_text", ColorARGB(1.0, 0.0 / 225.0, 65.0 / 225.0, 255 / 255.0)},         // rgba(0, 65, 255, 0.99)

};
// 定义周围车辆颜色信息
// 皮卡
// double color_pickup_a = 0.5, sur_pickup_r = 0, sur_pickup_g = 0, sur_pickup_b = 1;
// // NTE240
// double color_NTE240_a = 0.5, sur_NTE240_r = 0, sur_NTE240_g = 1, sur_NTE240_b = 0;
// // NTE200
// double color_NTE200_a = 0.5, sur_NTE200_r = 1, sur_NTE200_g = 0, sur_NTE200_b = 0;

ColorARGB GetColorByValue(const decimal_t val, const std::map<decimal_t, ColorARGB> &m) {
    auto it = m.upper_bound(val);
    return it->second;
}

ColorARGB GetJetColorByValue(const decimal_t val_in, const decimal_t vmax, const decimal_t vmin) {
    decimal_t val = val_in;
    if (val < vmin)
        val = vmin;
    if (val > vmax)
        val = vmax;
    double dv = vmax - vmin;

    ColorARGB c(1.0, 1.0, 1.0, 1.0);
    if (val < (vmin + 0.25 * dv)) {
        c.r = 0;
        c.g = 4.0 * (val - vmin) / dv;
    } else if (val < (vmin + 0.5 * dv)) {
        c.r = 0;
        c.b = 1.0 + 4.0 * (vmin + 0.25 * dv - val) / dv;
    } else if (val < (vmin + 0.75 * dv)) {
        c.r = 4.0 * (val - vmin - 0.5 * dv) / dv;
        c.b = 0.0;
    } else {
        c.g = 1 + 4.0 * (vmin + 0.75 * dv - val) / dv;
        c.b = 0.0;
    }
    return c;
}

} // namespace common