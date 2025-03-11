### **Step 1: Set Up Your ROS2 Workspace**

**Open a Terminal.**

**Create a Workspace with a src Folder:**

`mkdir -p ~/nova_ws/src`

`cd ~/nova_ws/src`

### **Step 2: Download the Robot Package from GitHub**

`git clone https://github.com/Combat-Craft/NOVA`

### **Step 3: Ensure all dependencies are met**

If necessary, input these commands to download necessary packages.

`sudo apt update`

`sudo apt install python3-colcon-common-extensions`

`sudo apt install ros-humble-navigation2 ros-humble-nav2-core`

### **Step 4: Build the Package and Source Your Workspace**

**Return to Your Workspace Root and Build:**

`cd ..`

`colcon build --symlink-install`

Using the —symlink-install option means you won’t have to build it again when we make changes, only when you add a new file.

Source the overlay.  IE. from the root of your workspace (in `nova_ws`) input

`source install/setup.bash` 

Alternatively, you can source the overlay from any folder by typing 

`source ~/nova_ws/install/setup.bash`

You can even echo it to your bashrc so you never have to type it again: 

`echo 'source ~/nova_ws/install/setup.bash' >> ~/.bashrc`

### **Step 4: Launch the Robot**

Launch just the robot with 

`ros2 launch nova_bringup gazebo.launch.py`

Alternatively, launch the entire nav2 stack with

`ros2 launch nova_bringup nav.launch.py`

RViz2 will open with a default view showing the laser scan map, robot model, and TF data.

For the GPS implementation and the dual Kalman filter, 

`ros2 launch nova_bringup gps.launch.py`

### **Interacting in RViz**

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

Please continue with the ROS2 and Nav2 tutorials here 

https://docs.nav2.org/setup_guides/index.html#table-of-contents

Follow the list on the right that says “Gazebo classic”
