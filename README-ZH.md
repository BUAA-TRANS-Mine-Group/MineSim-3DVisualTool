<p align="center">
    <img src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-web-figures/logo_minesim_big.png" alt="Logo" width="500">
    <h1 align="center">A 3D visualization tool for the MineSim project</h1>

</p>

<br/>

> **MineSim: Scenario-Based Simulator for Autonomous Truck Planning in Open-Pit Mines** <br>
> <br>

# MineSim-3DVisualTool-Dev

简介：MineSim 项目的三维可视化工具，基于 ROS 开发。

功能特性：

- 可以对场景 simulation 的结果进行三维场景可视化；
- 支持增加场景 simulation 过程的信息；

> [!CAUTION]
>
> 更多信息参考：[MineSim 项目主页 Link](https://buaa-trans-mine-group.github.io/minesim/)

---

## Demo

MineSim-3DVisualTool 的使用示例

<p align="center">
    <strong>Video: Usage example of MineSim-3DVisualTool</strong>
    <video style="display:block; width:100%; height:auto;" autoplay="autoplay" muted loop="loop" controls playsinline>
        <source
        src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-3d-visualizer-tool/minesim-3dvisualize-tool-using-method.mp4"
        type="video/mp4" />
</p>

MineSim-3DVisualTool 的可视化场景`jiangtong_intersection_9_3_2`的示例

<p align="center">
    <strong>Video: Example of the visualization scenario </strong>
    <video style="display:block; width:100%; height:auto;" autoplay="autoplay" muted loop="loop" controls playsinline>
        <source
        src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-3d-visualizer-tool/minesim-3dvisualize-tool-scenario-demo-jiangtong_intersection_9_3_2_2024-09-27.mp4"
        type="video/mp4" />
</p>

<p align="center">
  <strong>Figure: scenario ID jiangtong_intersection_9_3_2</strong>
  <img style="display:block; width:100%; height:auto;"
    src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-3d-visualizer-tool/FIG-minesim-3dvisualize-tool-scenario-demo-jiangtong_intersection_9_3_2_2024-09-27.png"
    alt="Example png">
</p>

---

## :truck: 使用说明

### 1.准备工作

> 注：当前已经测试的环境：ubuntu 20.04 + [ROS Noetic Ninjemys](https://wiki.ros.org/cn/noetic/Installation)

- 下载项目代码；

- 安装 ROS ，**完整桌面版安装（Desktop-Full，推荐）**；https://wiki.ros.org/cn/ROS/Tutorials；

- 安装 ROS 依赖包：

  ```bash
  # noetic为对应的ros版本

  sudo apt-get install ros-noetic-rviz-animated-view-controller
  sudo apt-get install ros-noetic-joy

  source /opt/ros/noetic/setup.bash
  ```

### 2.编译项目 MineSim-3DVisualTool | Build on ROS

在`MineSim-3DVisualTool`项目根目录下，打开终端，执行 ros 编译命令。

```
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/HKUST-Aerial-Robotics/EPSILON.git
cd ..
catkin_make
source ~/${YOUR_WORKSPACE_PATH}/devel/setup.bash
catkin_make
```

编译成功后，既可以运行 3DVisualTool 回放仿真过程的 log；

!注意：每次执行一个新的终端，需要对 `./devel/setup.bash` 进行 source：`source ./devel/setup.bash`，确保编译后的项目被 ROS 获取；

### 3.运行

- 将 minesim 仿真闭环测试得到的 log.json 文件复制到`src/minesim_visualizor_3d/core/playgrounds/minesim_run_log`
- 打开新终端，运行 roscore：`roscore`
- 打开新终端，先 `source ./devel/setup.bash` ，再运行 phy_simulator_logging （`MineSim-3DVisualTool/launch/rviz`终端目录下运行）： `rviz -d phy_simulator_logging.rviz`
- 打开新终端，先 `source ./devel/setup.bash` ，再运行 phy_simulator_planning： `roslaunch phy_simulator phy_simulator_planning.launch`
- 打开新终端，先 `source ./devel/setup.bash` ，再运行 minesim_agents_log : `roslaunch minesim_log_vis minesim_agents_log.launch`

注：

```bash
# roslaunch可以用来启动定义在launch（启动）文件中的节点。
# 用法：
roslaunch [package] [filename.launch]
```

## :rocket:注意

MineSim-3DVisualTool 可视化工程，待改进：

- 自动解析 log.json 中的车辆数量，自动配置 rviz Marker
- 所有的 rviz Marker 的 color 和 scale 设置在外部文件配置`src/minesim_visualizor_3d/core/playgrounds/configuration/config.json`

## :tada:参考资料 | Acknowledgements

We would like to express sincere thanks to the authors of the following tools and packages:

- Autonomous Driving simulation tool EPSILON：**[EPSILON](https://github.com/HKUST-Aerial-Robotics/EPSILON)**

- Lock-free queue: [moodycamel](https://github.com/cameron314/concurrentqueue)

- Json parser: [JSON for modern C++](https://github.com/nlohmann/json)

- KD-Tree library: [nanoflann](https://github.com/jlblancoc/nanoflann)

## :tada:Licence

The source code is released under [MIT](https://opensource.org/licenses/MIT) license.

## :tada:Disclaimer

This is research code, it is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of merchantability or fitness for a particular purpose.
