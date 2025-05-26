## ✅ Prerequisites

Before you begin, make sure you're on **Ubuntu 22.04** and have installed **ROS 2 Humble**.

---

## Clone the repo
```bash
git clone https://github.com/Combat-Craft/NOVA.git
```
```bash
cd NOVA
```

## 🔧 Dependencies

Once you’ve cloned this repo, you’ll need a few additional packages installed.

```bash
sudo apt update
sudo apt install -y \
    python3-serial \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 ros-humble-nav2-core \
    python3-colcon-common-extensions
```

## Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
## Build the workspace
```bash
colcon build
```

---
After building, source the workspace and launch the controller:

## Source the workspace
```bash
source install/setup.bash
```
---
# Controlling the robot via the PS4 Gamepad
## Run motor controller
```bash
ros2 launch controller motor.launch.py
```

## Run joystick
In a different terminal, source the install folder and run:
```bash
ros2 launch controller joystick.launch.py
```

if you plan to have wireless control, make sure the following is done:
- Both computers are on the same network (don't use wifi and use a hotspot)
- if using a VM, ensure the network adapter is set to Bridged mode rather than NAT
- run the following lines (on both computers): `echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc` and `echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc`
- restart your terminals and try now

---
# Launching the Robot in sim

Launch just the robot with 

`ros2 launch nova_bringup gazebo.launch.py`

Alternatively, launch the entire nav2 stack with

`ros2 launch nova_bringup nav.launch.py`

RViz2 will open with a default view showing the laser scan map, robot model, and TF data.

For the GPS implementation and the dual Kalman filter, 

`ros2 launch nova_bringup gps.launch.py`

## Interacting in RViz

1. **Set the Initial Pose:**
    - In RViz, click the **"2D Pose Estimate"** tool (the green arrow).
    - Click on the map at your robot’s starting position.
    - Drag in the direction you want the robot to face and then release.
2. **Set a Navigation Goal:**
    - Click the **"2D Nav Goal"** tool (the blue arrow).
    - Click on the map where you want the robot to go.
    - Drag to set the orientation at the goal and release.
    - Nav2 will compute a path and, if everything is working, the robot should start moving toward the goal in Gazebo.

## Troubleshooting

If Gazebo claims you have processes running,  type 

`killall -9 gazebo gzserver gzclient`

Going forward: 
- All NAV2 goals fail. We need to fix this.  Your input is needed!
- The GPS simulation is not publishing any data.  Why is that?

Please continue with the ROS2 tutorials and Nav2 tutorials here 

https://docs.nav2.org/setup_guides/index.html#table-of-contents

Follow the list on the right that says “Gazebo classic”

Then do this one 

https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
