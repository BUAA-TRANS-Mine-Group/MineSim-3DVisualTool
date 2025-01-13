# 设置CMake的最小版本要求
cmake_minimum_required(VERSION 3.0.2)

# 设置项目名
project(minesim_log_vis)

# 查找catkin的依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp          # ROS C++客户端库
  rospy           # ROS Python客户端库
  std_msgs        # 标准消息
  sensor_msgs    # 传感器消息
  tf              # 坐标变换
  nav_msgs        # 导航消息
  cmake_modules   # CMake模块
  view_controller_msgs  # 自定义消息
)

# 查找Boost库，并确保Boost的filesystem模块可用
find_package(Boost REQUIRED filesystem)
include_directories(${Boost_INCLUDE_DIRS})  # 确保Boost头文件路径被包含
add_definitions(${Boost_DEFINITIONS})  # 如果需要的话，添加Boost的编译定义

# 查找Eigen库（线性代数库）
find_package(Eigen REQUIRED)

# 查找common库
find_package(common REQUIRED)

# 查找LZ4库
# set(LZ4_ROOT "/home/lz/anaconda3/lib")  # 设置LZ4库的实际路径
# set(LZ4_LIBRARY "${LZ4_ROOT}/liblz4.so")  # 根据你的系统，这可能是.so（动态）或.a（静态）

# 添加LZ4库到链接器搜索路径
# link_directories(${LZ4_ROOT})

# 定义catkin包的配置
catkin_package(
  # INCLUDE_DIRS include  # 可以启用此行来指定要导出的头文件目录
  # LIBRARIES gps  # 可以启用此行来指定要导出的库
  # CATKIN_DEPENDS roscpp rospy std_msgs view_controller_msgs  # 指定catkin包的依赖
  # DEPENDS system_lib  # 指定其他系统库的依赖
)

# 包含目录：
# 1. 包含项目自己的头文件
# 2. 包含catkin包的头文件
# 3. 添加common库的头文件路径
# 4. 添加Eigen库的头文件路径
include_directories(
  include     # 包含项目自己的头文件
  ${catkin_INCLUDE_DIRS}          # Catkin的依赖头文件路径
  ${common_INCLUDE_DIRS}  # 上面必须添加 find_package(common REQUIRED)# 查找common库
  ${PROJECT_SOURCE_DIR}/src/minesim_visualizor_3d/core/common/inc/common  # 添加common头文件路径
  /home/czf/project_czf/20240901-MineSim/MineSim-3DVisualTool-Dev/src/minesim_visualizor_3d/core/common/inc/common # 添加common头文件路径
  /home/czf/project_czf/20240901-MineSim/MineSim-3DVisualTool-Dev/src/minesim_visualizor_3d/core/common/inc/common # 添加common头文件路径
  ${Eigen_INCLUDE_DIRS}           # 添加Eigen库的头文件路径
)

# 添加可执行文件，gps2sim.cpp将被编译成一个可执行文件
add_executable(gps2sim
  src/gps2sim.cpp
)

# 设置gps2sim依赖的其他目标
add_dependencies(gps2sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 链接gps2sim可执行文件所需要的库，包括catkin库和Boost库
target_link_libraries(gps2sim
  hkust_pl_common
  ${catkin_LIBRARIES}  # 链接Catkin相关的库
  ${Boost_LIBRARIES}   # 链接Boost的filesystem库
)
