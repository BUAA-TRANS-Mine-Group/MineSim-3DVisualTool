<p align="center">
    <img src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-web-figures/logo_minesim_big.png" alt="Logo" width="500">
    <h1 align="center">A 3D visualization tool for the MineSim project</h1>

</p>

> **MineSim: Scenario-Based Simulator for Autonomous Truck Planning in Open-Pit Mines** <br>

# MineSim-3DVisualTool

## :fire: TODO

- [x] Initial project, 2024-10-05
- [x] Release the project MineSim-3DVisualTool v1.0_zh, 2025-01-13
- [ ] Release the project MineSim-3DVisualTool v1.0_en, xxxx

## :truck: Introduction

A 3D visualization tool for the MineSim project, developed based on ROS.

## :truck: Features

- **3D Scene Visualization**: Visualize the results of simulation scenarios in a 3D environment.
- **Simulation Process Information**: Support adding information during the simulation process.

> [!CAUTION]
>
> For more information, refer to the [MineSim Project Homepage](https://buaa-trans-mine-group.github.io/minesim/).

---

## :truck: Demo

Usage example of MineSim-3DVisualTool

<p align="center">
    <strong>Video: Usage example of MineSim-3DVisualTool</strong>
    <video style="display:block; width:100%; height:auto;" autoplay="autoplay" muted loop="loop" controls playsinline>
        <source
        src="https://raw.githubusercontent.com/byChenZhifa/archive/main/minesim/minesim-3d-visualizer-tool/minesim-3dvisualize-tool-using-method.mp4"
        type="video/mp4" />
</p>

**Example Image of the Scenario:**
Example of the visualization scenario `jiangtong_intersection_9_3_2` for MineSim-3DVisualTool:

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

## :truck: Usage Instructions

### 1. Preparation

> **Note**: The tested environment is Ubuntu 20.04 + [ROS Noetic Ninjemys](https://wiki.ros.org/noetic/Installation).

- **Download Project Code**.

- **Install ROS**:

  - **Full Desktop Installation (Desktop-Full, recommended)**.
  - Follow the [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials) for installation.

- **Install ROS Dependencies**:

  ```bash
  # noetic corresponds to the ROS version

  sudo apt-get install ros-noetic-rviz-animated-view-controller
  sudo apt-get install ros-noetic-joy

  source /opt/ros/noetic/setup.bash
  ```

### 2. Build the MineSim-3DVisualTool Project | Build on ROS

In the root directory of the `MineSim-3DVisualTool` project, open a terminal and execute the ROS build commands.

```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/HKUST-Aerial-Robotics/EPSILON.git
cd ..
catkin_make
source ~/${YOUR_WORKSPACE_PATH}/devel/setup.bash
catkin_make
```

After a successful build, you can run the 3DVisualTool to replay the simulation process logs.

> **Note**: Every time you open a new terminal, you need to source `./devel/setup.bash` to ensure that the compiled project is recognized by ROS:

```bash
source ./devel/setup.bash
```

### 3. Running

- **Copy the `log.json` file**:
  Copy the `log.json` file obtained from the MineSim closed-loop simulation test to `src/minesim_visualizor_3d/core/playgrounds/minesim_run_log`

- **Start ROS Master**:

  Open a new terminal and run:

  ```bash
  roscore
  ```

- **Run Phy Simulator Logging**:

  Open a new terminal, first source the setup file, then run:

  ```bash
  source ./devel/setup.bash
  cd ./launch/rviz
  rviz -d phy_simulator_logging.rviz
  ```

- **Run Phy Simulator Planning**:

  Open a new terminal, first source the setup file, then run:

  ```bash
  source ./devel/setup.bash
  roslaunch phy_simulator phy_simulator_planning.launch
  ```

- **Run Minesim Agents Log Visualization**:

  Open a new terminal, first source the setup file, then run:

  ```bash
  source ./devel/setup.bash
  roslaunch minesim_log_vis minesim_agents_log.launch
  ```

**Note**:

```bash
# roslaunch is used to start nodes defined in launch files.
# Usage:
roslaunch [package] [filename.launch]
```

## :rocket: Important Notes

The MineSim-3DVisualTool visualization project needs improvement:

- **Automatic Vehicle Count Parsing**: Automatically parse the number of vehicles from `log.json` and configure RViz Markers accordingly.
- **External Configuration for Marker Settings**: All RViz Marker colors and scales are set in an external file `src/minesim_visualizor_3d/core/playgrounds/configuration/config.json`.

## :tada: Acknowledgements

We would like to express our sincere thanks to the authors of the following tools and packages:

- **Autonomous Driving Simulation Tool EPSILON**: [EPSILON](https://github.com/HKUST-Aerial-Robotics/EPSILON)
- **Lock-free Queue**: [moodycamel](https://github.com/cameron314/concurrentqueue)
- **JSON Parser**: [JSON for Modern C++](https://github.com/nlohmann/json)
- **KD-Tree Library**: [nanoflann](https://github.com/jlblancoc/nanoflann)

## :tada: License

The source code is released under the [MIT](https://opensource.org/licenses/MIT) license.

## :tada: Disclaimer

This is research code, distributed in the hope that it will be useful, but **WITHOUT ANY WARRANTY**; without even the implied warranty of merchantability or fitness for a particular purpose.
