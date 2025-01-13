# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 2.8)

# 设置项目名称
project(phy_simulator)

# 设置CMake的详细输出模式（在构建时显示更多信息）
set(CMAKE_VERBOSE_MAKEFILE "true")

# 设置构建类型为Release模式
set(CMAKE_BUILD_TYPE "Release")

# 设置C++编译选项，启用C++11标准并生成调试信息
set(CMAKE_CXX_FLAGS "-std=c++11  -g")

# 设置Release模式下的C++编译选项，启用优化并显示所有警告
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

# 将Release模式的编译选项与常规选项合并
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# 查找catkin依赖的包，确保所需的ROS包存在
find_package(catkin REQUIRED COMPONENTS
    roscpp                    # ROS C++客户端库
    visualization_msgs         # 可视化消息包
    sensor_msgs               # 传感器消息包
    geometry_msgs             # 几何消息包
)

# 查找Eigen3库，这是一个用于矩阵运算的数学库
find_package(Eigen3 REQUIRED)

# 查找common库
find_package(common REQUIRED)

# 查找vehicle_model库，这是用于车辆模型的库
find_package(vehicle_model REQUIRED)

# 查找vehicle_msgs库，这是与车辆消息相关的库
find_package(vehicle_msgs REQUIRED)

# 如果该catkin包的头文件被其他包使用，使用catkin_package来声明该包的包含目录
catkin_package(
    # 这里可以指定其他包的依赖，例如：LIBRARIES, CATKIN_DEPENDS 等
)

# 包含当前项目的头文件目录以及其他依赖包的头文件目录
include_directories(
    inc                                # 当前包的头文件目录
    ${common_INCLUDE_DIRS}             # common库的头文件目录
    ${vehicle_model_INCLUDE_DIRS}      # vehicle_model库的头文件目录
    ${vehicle_msgs_INCLUDE_DIRS}       # vehicle_msgs库的头文件目录
    ${semantics_msgs_INCLUDE_DIRS}     # semantics_msgs库的头文件目录（未在find_package中声明，可能是遗漏）
    ${catkin_INCLUDE_DIRS}             # catkin包的头文件目录
    ${EIGEN3_INCLUDE_DIR}              # Eigen3库的头文件目录
)

# 创建phy_simulator_lib库，并指定其源文件
add_library(phy_simulator_lib
    src/phy_simulator/arena_loader.cc       # 物理模拟器的arena_loader实现文件
    src/phy_simulator/phy_simulator.cc      # 物理模拟器的主逻辑实现文件
)

# 链接所需的库到phy_simulator_lib
target_link_libraries(phy_simulator_lib
    hkust_pl_vehicle_model    # hkust_pl_vehicle_model库
    hkust_pl_common           # hkust_pl_common库
)

# 创建phy_simulator_planning_node可执行文件，并指定其源文件
add_executable(phy_simulator_planning_node
    src/phy_simulator_planning_node.cc   # 物理模拟器规划节点的主文件
    src/phy_simulator/visualizer.cc      # 可视化器实现文件
    src/phy_simulator/ros_adapter.cc     # ROS适配器实现文件
)

# 将所需的库链接到phy_simulator_planning_node可执行文件
target_link_libraries(phy_simulator_planning_node
    ${catkin_LIBRARIES}                 # 链接catkin依赖的库
    phy_simulator_lib                   # 链接phy_simulator_lib库
)

# 注释掉的安装部分，说明如何安装头文件
# 安装当前包的头文件，方便其他包引用
#install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}    # 安装目标路径为catkin的包包含路径
 #  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
 #  FILES_MATCHING PATTERN "*.h"                          # 安装所有.h文件
 #  PATTERN ".svn" EXCLUDE                                # 排除.svn目录
#)
