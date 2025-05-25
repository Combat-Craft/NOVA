## ✅ Prerequisites

Before you begin, make sure you're on **Ubuntu 22.04** and have installed **ROS 2 Humble**.

---

# Clone the repo
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
    python3-colcon-common-extensions
```

# Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```
# Build the workspace
```bash
colcon build
```

After building, source the workspace and launch the controller:

# Source the workspace
```bash
source install/setup.bash
```
# Run motor controller
```bash
ros2 launch controller motor.launch.py
```

# Run joystick
```bash
ros2 launch controller joystick.launch.py
```
