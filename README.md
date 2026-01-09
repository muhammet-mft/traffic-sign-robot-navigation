# Traffic Sign Guided Autonomous Robot Navigation with SLAM

**Team 10 - Final Project**

A ROS-based autonomous mobile robot navigation system that uses computer vision for traffic sign detection and SLAM algorithms for mapping and localization in a simulated office environment.

> **Robot URDF Model**: [Pioneer 3-AT Description](https://github.com/MobileRobots/amr-ros-config/tree/master/description)

## 👥 Team Members
- Yunus Emre Kuru
- Akın Mırık
- Muhammet Müftüoğlu

## 📋 Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [System Architecture](#️-system-architecture)
- [Hardware Specifications](#-hardware-specifications)
- [Installation](#-installation)
- [Usage](#-usage)
- [Project Structure](#-project-structure)
- [SLAM Comparison Results](#-slam-comparison-results)
- [Traffic Sign Detection](#-traffic-sign-detection)
- [Demo Videos](#-demo-videos)
- [Troubleshooting](#-troubleshooting)
- [References](#-references)

## 🎯 Overview

This project implements an autonomous navigation system for a Pioneer 3-AT mobile robot in a Gazebo-simulated AWS Office environment. The robot uses:

- **Computer Vision** - Detects traffic signs (turn left, turn right, go straight, park) using OpenCV template matching
- **SLAM Algorithms** - Creates 2D occupancy grid maps using GMapping and Hector SLAM
- **Reactive Navigation** - Wall-following algorithm with LiDAR-based obstacle avoidance and traffic sign control
- **Sensor Fusion** - Integrates dual LiDAR sensors, RGB camera, and wheel odometry

### Project Objectives

1. ✅ Add traffic signs to Gazebo world (left, right, straight, park)
2. ✅ Implement traffic sign detector using RGB camera (OpenCV)
3. ✅ Guide robot through environment with global/local planner and tracker
4. ✅ Run SLAM algorithms to create 2D maps:
   - Hector SLAM
   - GMapping
5. ✅ Compare algorithms qualitatively and quantitatively:
   - Output map quality
   - Localization performance vs. ground truth

## ✨ Features

- **Autonomous Exploration**: Wall-following algorithm with dynamic obstacle avoidance
- **Traffic Sign Recognition**: Real-time detection with 65% confidence threshold
- **Dual SLAM Implementation**: Compare GMapping vs Hector SLAM performance
- **Reactive Navigation**: Distance-based speed adjustment (0.15-0.55 m/s)
- **Parking Mode**: Automated parking at 1.4m from detected obstacles
- **Performance Metrics**: Real-time RMSE calculation against ground truth
- **Multi-sensor Fusion**: Front/rear LiDAR + RGB camera + odometry

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                        │
│              (AWS Office World + Pioneer 3-AT)              │
└──────────────┬──────────────────────────────────────────────┘
               │
    ┌──────────┴──────────┬──────────────┐
    │                     │              │
┌───▼────┐        ┌───────▼──────┐  ┌───▼────┐
│ LiDAR  │        │  RGB Camera  │  │ Odom   │
│ F + R  │        │ (12.3 MP)    │  │        │
└───┬────┘        └──────┬───────┘  └───┬────┘
    │                    │              │
    │ /laser_scan        │ /image_raw   │
    │                    │              │
    │            ┌───────▼──────────┐   │
    │            │ Traffic Detector │   │
    │            │  (OpenCV)        │   │
    │            └───────┬──────────┘   │
    │                    │              │
    │                    │ /traffic_sign│
    │                    │              │
    ├──────────>┌───────▼────┐         │
    │           │    SLAM    │         │
    │           │  Gmapping/ │         │
    │           │   Hector   │         │
    │           └──────┬─────┘         │
    │                  │ /map          │
    │                  │               │
    └──────────>┌──────▼─────────┐    │
                │   Navigator    │◄───┤
                │(Wall-Following)│◄───┘ /odom
                └────────┬───────┘
                         │ /sim_p3at/cmd_vel
                ┌────────▼─────────┐
                │  Robot Motion    │
                └──────────────────┘
```

**Architecture Notes:**
- **LiDAR** provides data to three components in parallel:
  - SLAM Engines (for mapping)
  - Navigator (for wall-following & obstacle avoidance)
  - Traffic Detector (for context)
- **Navigator** receives inputs from:
  - LiDAR (`/front_laser/scan`) - obstacle detection
  - Traffic signs (`/traffic_sign`) - navigation commands
  - Odometry (`/odom`) - position tracking
- Navigator uses **wall-following algorithm** with direct sensor feedback (no path planner)
- SLAM provides mapping for visualization; navigator operates reactively

## 🤖 Hardware Specifications

### Robot Platform: Pioneer 3-AT

| Component | Specification |
|-----------|---------------|
| **Type** | All-Terrain Differential Drive Mobile Robot |
| **Dimensions** | 52cm (W) × 56cm (L) footprint |
| **Mass** | 21.5 kg |
| **Drive** | 4-wheel differential drive with skid steering |
| **Max Linear Speed** | 0.35 m/s (hardware limit)<br>0.15-0.55 m/s (software operational range) |
| **Max Angular Speed** | 0.8 rad/s |

### Sensors

| Sensor | Model/Specs | Purpose |
|--------|-------------|---------|
| **Front LiDAR** | 2D Laser Scanner<br>• 270° FOV<br>• 20m range<br>• 540 samples<br>• 0.5° resolution | Primary obstacle detection & SLAM |
| **Rear LiDAR** | Same as front | Rear obstacle detection & complete coverage |
| **RGB Camera** | Flir BlackflyS BFS-U3-122S6C-C<br>• 12.3 MP<br>• 23 FPS<br>• 60° FOV | Traffic sign detection |
| **Wheel Encoders** | Built-in | Odometry & dead reckoning |

### Simulation Environment

- **World**: AWS Office (from [mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps))
- **Physics Engine**: ODE (Open Dynamics Engine)
- **Update Rate**: 500 Hz
- **Environment Size**: ~30m × 30m indoor office space
- **Features**: Multiple rooms, hallways, corridors, office furniture, traffic signs

## 🔧 Installation

### Prerequisites

```bash
# Ubuntu 20.04 LTS
# ROS Noetic (or ROS Melodic for Ubuntu 18.04)
# Gazebo 11
# Python 3.8+
```

### Dependencies

```bash
# Install ROS Noetic (if not already installed)
sudo apt update
sudo apt install ros-noetic-desktop-full

# Install required ROS packages
sudo apt install ros-noetic-gazebo-ros-pkgs
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-slam-gmapping
sudo apt install ros-noetic-hector-slam
sudo apt install ros-noetic-move-base  # Required by launch file but not used by navigator
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-map-server

# Install Python dependencies
sudo apt install python3-pip
pip3 install opencv-python
pip3 install numpy
pip3 install rospy
```

### Setup

```bash
# Clone the repository
cd ~/Desktop
git clone https://github.com/muhammet-mft/traffic-sign-robot-navigation.git
cd traffic-sign-robot-navigation

# Create catkin workspace (if needed)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/Desktop/traffic-sign-robot-navigation/Files/amr-ros-config .

# Build the workspace
cd ~/catkin_ws
catkin_make

# Source the workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Copy Gazebo models to Gazebo model path
mkdir -p ~/.gazebo/models
cp -r Files/amr-ros-config/gazebo/models/* ~/.gazebo/models/
```

## 🚀 Usage

### Quick Start - Complete System (with Hector SLAM)

#### Terminal 1: Launch Gazebo World and Robot
```bash
roslaunch amr-ros-config project_world.launch
```

#### Terminal 2: Reset Robot Position
```bash
rosrun amr-ros-config reset_robot.py
```

#### Terminal 3: Start Traffic Sign Detector
```bash
rosrun amr-ros-config traffic_detector_2.py
```

#### Terminal 4: Launch Navigation Stack
```bash
roslaunch traffic_exploration.launch
```

#### Terminal 5: Launch Hector SLAM
```bash
roslaunch amr-ros-config slam_hector.launch
```

*Note: In RViz, Add -> Map and use topic `/map` to visualize mapping*

#### Terminal 6: Start Autonomous Navigator
```bash
rosrun amr-ros-config navigator_3.py
```

#### Terminal 7 (Optional): Monitor Position Accuracy
```bash
rosrun amr-ros-config coordinate_viewer.py
```

---

### Option A: Using GMapping

**See detailed instructions in:** `Files/gMapping/How to use codes.txt`

#### Terminal 1: Launch Gazebo World and Robot
```bash
roslaunch amr-ros-config project_world.launch
```

#### Terminal 2: Reset Robot Position
```bash
rosrun amr-ros-config reset_robot.py
```

#### Terminal 3: Start Traffic Sign Detector
```bash
rosrun amr-ros-config traffic_detector_2.py
```

#### Terminal 4: Launch GMapping SLAM
```bash
roslaunch amr-ros-config slam_gmapping.launch
```

*Note: In RViz, Add -> Map and use topic `/map` to visualize mapping*

#### Terminal 5: Start Autonomous Navigator
```bash
rosrun amr-ros-config navigator_3.py
```

#### Terminal 6 (Optional): Save Map
```bash
rosrun map_server map_saver -f gmapping_map
```

---

### Option B: Using Hector SLAM (Simplified - Without Navigation Stack)

**See detailed instructions in:** `Files/Hector/How to use codes.txt`

#### Terminal 1: Launch Gazebo World and Robot
```bash
roslaunch amr-ros-config project_world.launch
```

#### Terminal 2: Reset Robot Position
```bash
rosrun amr-ros-config reset_robot.py
```

#### Terminal 3: Start Traffic Sign Detector
```bash
rosrun amr-ros-config traffic_detector_2.py
```

#### Terminal 4: Launch Hector SLAM
```bash
roslaunch amr-ros-config slam_hector.launch
```

*Note: In RViz, Add -> Map and use topic `/map` to visualize mapping*

#### Terminal 5: Start Autonomous Navigator
```bash
rosrun amr-ros-config navigator_3.py
```

#### Terminal 6 (Optional): Save Map
```bash
rosrun map_server map_saver -f hector_map
```

---

### Visualization

RViz is automatically launched with the world. You can visualize:
- Robot model
- LaserScan data
- Generated maps
- Planned paths
- Costmaps
- Camera feed
- TF transforms

## 📁 Project Structure

```
traffic-sign-robot-navigation/
├── README.md
├── gmapping_haritam.pgm           # Generated GMapping map (16 MB)
├── hector_haritam.pgm             # Generated Hector map (4 MB)
├── Gmapping_Error.jpeg            # GMapping error logs
├── Hector_Error.jpeg              # Hector error logs
│
└── Files/
    ├── office.jpg                 # AWS Office environment reference image
    │
    ├── amr-ros-config/            # Main ROS package
    │   ├── launch/                # Launch files
    │   │   ├── project_world.launch         # Main world + robot
    │   │   ├── slam_gmapping.launch         # GMapping SLAM
    │   │   ├── slam_hector.launch           # Hector SLAM
    │   │   └── traffic_exploration.launch   # Navigation stack
    │   │
    │   ├── scripts/               # Python scripts
    │   │   ├── navigator_3.py               # Main navigation controller
    │   │   ├── traffic_detector_2.py        # Traffic sign detector
    │   │   ├── coordinate_viewer.py         # RMSE calculator
    │   │   ├── reset_robot.py               # Robot reset utility
    │   │   └── move_robot.py                # Movement test script
    │   │
    │   ├── nav_config/            # Navigation parameters
    │   │   ├── costmap_common_params.yaml
    │   │   ├── local_costmap_params.yaml
    │   │   ├── global_costmap_params.yaml
    │   │   └── base_local_planner_params.yaml
    │   │
    │   ├── description/           # Robot URDF models
    │   │   └── urdf/
    │   │       └── pioneer3at_lidar_rgb.urdf
    │   │
    │   └── gazebo/                # Gazebo simulation environment
    │       ├── final_Added.world           # Main simulation world (1.2 MB)
    │       ├── models/                     # 43 models total
    │       │   ├── sign_left/              # Traffic signs (4)
    │       │   ├── sign_right/
    │       │   ├── sign_straight/
    │       │   ├── sign_park/
    │       │   └── ... (39 office objects: furniture, actors, etc.)
    │       ├── map/                        # Ground truth maps
    │       └── media/                      # Textures and materials
    │
    ├── Signs_Models/             # Standalone traffic sign models
    │   ├── sign_left/
    │   ├── sign_right/
    │   ├── sign_straight/
    │   └── sign_park/
    │
    ├── gMapping/                 # GMapping-specific implementation
    │   ├── How to use codes.txt
    │   ├── navigator_3.py
    │   ├── traffic_detector_2.py
    │   ├── coordinate_viewer.py
    │   ├── reset_robot.py
    │   └── slam_gmapping.launch
    │
    └── Hector/                   # Hector-specific implementation
        ├── How to use codes.txt
        ├── navigator_3.py
        ├── traffic_detector_2.py
        ├── coordinate_viewer.py
        ├── reset_robot.py
        ├── slam_hector.launch
        └── traffic_exploration.launch
```

## 📊 SLAM Comparison Results

### Quantitative Analysis

| Metric | GMapping | Hector SLAM | Winner |
|--------|----------|-------------|--------|
| **Map Resolution** | 4000×4000 px | 2048×2048 px | **GMapping** ✓ |
| **Map File Size** | 16 MB | 4 MB | Hector (more efficient) |
| **Average RMSE** | ~0.403-0.409 | ~0.041 | **Hector** ✓ |
| **Computational Load** | Medium | Low | **Hector** ✓ |
| **Map Update Rate** | 1 Hz | Real-time | **Hector** ✓ |
| **Requires Odometry** | Yes | No | **Hector** ✓ |

### Qualitative Observations

#### GMapping
**Strengths:**
- Higher resolution maps (4000×4000)
- Better for larger environments
- More detailed obstacle representation
- Particle filter handles ambiguity well

**Weaknesses:**
- Higher computational cost (30 particles)
- Requires good odometry
- Slower map updates (1s intervals)
- Higher RMSE in our tests (~0.403-0.409)

#### Hector SLAM
**Strengths:**
- Excellent localization accuracy (RMSE ~0.041)
- No odometry required (scan-matching based)
- Real-time map updates
- Lower computational requirements
- Works well in structured environments

**Weaknesses:**
- Lower map resolution (2048×2048)
- May struggle in feature-poor areas
- Less robust to fast rotations
- Smaller map size limit

### Generated Maps

Below are the actual maps generated by both SLAM algorithms during our autonomous exploration:

#### GMapping Output
![GMapping Map](gmapping_haritam.pgm)
*GMapping generated map (16 MB, 4000×4000 px) - Higher resolution with more detail*

#### Hector SLAM Output
![Hector SLAM Map](hector_haritam.pgm)
*Hector SLAM generated map (4 MB, 2048×2048 px) - Lighter weight with excellent accuracy*

### Conclusion
For this office environment navigation task, **Hector SLAM significantly outperformed GMapping** with ~10× better RMSE (0.041 vs 0.403-0.409). Hector's scan-matching approach proved more suitable for structured indoor environments with good geometric features.

## 🚦 Traffic Sign Detection

### Sign Types

| Sign | Visual | Description | Robot Response |
|------|--------|-------------|----------------|
| **LEFT** | ![Left](Files/Signs_Models/sign_left/materials/textures/left.jpeg) | Blue circle, white left arrow | Turns left at next intersection |
| **RIGHT** | ![Right](Files/Signs_Models/sign_right/materials/textures/right.jpeg) | Blue circle, white right arrow | Turns right at next intersection |
| **STRAIGHT** | ![Straight](Files/Signs_Models/sign_straight/materials/textures/straight.jpeg) | Blue circle, white up arrow | Continues straight ahead |
| **PARK** | ![Park](Files/Signs_Models/sign_park/materials/textures/park.jpeg) | Blue circle, "P" symbol | Approaches to 1.4m and stops |

### Detection Algorithm

**Method**: OpenCV Template Matching
- **Algorithm**: `cv2.matchTemplate()` with `TM_CCOEFF_NORMED`
- **Confidence Threshold**: 0.65 (65%)
- **Input**: `/rgb_camera/image_raw` (640×480 @ 23 FPS)
- **Output**: `/traffic_sign` topic (String messages)
- **Processing**: Real-time with live video display

### Implementation Details

```python
# From traffic_detector_2.py
def image_callback(self, data):
    # Convert ROS Image to OpenCV format
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    detected_sign = "NONE"
    max_val_all = 0

    # Template matching for each sign type
    for name, template in self.templates.items():
        res = cv2.matchTemplate(gray_image, template, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        if max_val > 0.65:  # Confidence threshold
            if max_val > max_val_all:
                max_val_all = max_val
                detected_sign = name

    if detected_sign != "NONE":
        self.sign_pub.publish(detected_sign)
```

### Navigation Response

**Note**: The code below is simplified pseudocode for illustration. The actual implementation in `navigator_3.py` includes additional logic for LiDAR-based distance checking, turn duration control (3 seconds), stored command management, and real-time obstacle avoidance.

```python
# From navigator_3.py (Simplified)
if detected_sign == "LEFT":
    turn_left()
elif detected_sign == "RIGHT":
    turn_right()
elif detected_sign == "STRAIGHT":
    continue_forward()
elif detected_sign == "PARK":
    park_mode(distance=1.4)  # Stop at 1.4m
```

## 🎥 Demo Videos

**📹 [View All Demo Videos on Google Drive](https://drive.google.com/drive/folders/1C9OPpHTNZPf5NemMt2ehJV0YITXUk2D9)**

The demo folder includes:
- ✅ Full autonomous exploration with GMapping
- ✅ Full autonomous exploration with Hector SLAM
- ✅ Traffic sign detection demo
- ✅ Parking behavior demo
- ✅ SLAM comparison visualization

## 🐛 Troubleshooting

### Common Issues

#### Issue: "Unable to load Gazebo models"
**Solution:**
```bash
# Add model path to environment
export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH
echo "export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
```

#### Issue: "No LaserScan data received"
**Solution:**
- Check that Gazebo is fully loaded (wait 10-15 seconds)
- Verify sensor topics: `rostopic list | grep laser`
- Check sensor connections: `rosnode info /gazebo`

#### Issue: "Traffic sign detector shows no window"
**Solution:**
```bash
# Install OpenCV with GUI support (remove headless version)
pip3 uninstall opencv-python-headless
pip3 install opencv-python
pip3 install opencv-contrib-python
```

#### Issue: "Robot doesn't move"
**Solution:**
1. Reset robot: `rosrun amr-ros-config reset_robot.py`
2. Check velocity topic: `rostopic echo /cmd_vel`
3. Verify navigator is running: `rosnode list | grep navigator`

#### Issue: "SLAM map is empty or distorted"
**Solution:**
- Ensure robot is moving (SLAM needs motion)
- Check LaserScan range (should see data in RViz)
- Verify TF transforms: `rosrun tf view_frames`
- Increase particle count (GMapping) or decrease resolution (Hector)

#### Issue: "High localization error (RMSE)"
**Solution:**
- Run calibration longer (coordinate_viewer.py needs 5+ seconds)
- Ensure odometry is publishing correctly
- Check for wheel slippage in simulation
- Verify sensor synchronization

## 📚 References

### ROS Packages Used
- [navigation](http://wiki.ros.org/navigation) - ROS Navigation Stack
- [gmapping](http://wiki.ros.org/gmapping) - OpenSlam GMapping
- [hector_slam](http://wiki.ros.org/hector_slam) - Hector SLAM
- [gazebo_ros_pkgs](http://wiki.ros.org/gazebo_ros_pkgs) - Gazebo-ROS integration
- [pioneer_description](https://github.com/MobileRobots/amr-ros-config) - Pioneer robot models

### Resources
- [AWS Office World](https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps)
- [ROS Navigation Tuning Guide](http://wiki.ros.org/navigation/Tutorials)
- [OpenCV Template Matching](https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html)

### Papers & Documentation
- **GMapping**: Grisetti, G., Stachniss, C., & Burgard, W. (2007). "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters." *IEEE Transactions on Robotics*, 23(1), 34-46. [Paper](https://ieeexplore.ieee.org/document/4084563/)
- **Hector SLAM**: Kohlbrecher, S., von Stryk, O., Meyer, J., & Klingauf, U. (2011). "A Flexible and Scalable SLAM System with Full 3D Motion Estimation." *IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)*. [ROS Wiki](http://wiki.ros.org/hector_slam)
- **Pioneer 3-AT**: [ROS Pioneer 3-AT Documentation](https://robots.ros.org/pioneer-3-at/)

## 📄 License

This project was developed as an academic assignment. Feel free to use it for educational purposes.

## 🙏 Acknowledgments

- Course instructors and teaching assistants
- ROS community for excellent documentation
- AWS for the office world model
- OpenCV community for computer vision tools

---

**Project Status**: ✅ Completed (January 2026)

**Contact**: Team 10 Members (see above)

**Institution**: Istanbul Technical University

**Course**: KON 414E - Principles of Robot Autonomy
