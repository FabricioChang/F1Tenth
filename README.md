# F1TENTH Simulator for Autonomous Vehicle Competitions
The F1TENTH simulator for ROS 2 is a simulation environment designed to test control, navigation, and planning algorithms on a 1/10 scale autonomous vehicle.

**Features of the F1TENTH Simulator in ROS 2**
- Publishes and subscribes to ROS 2 topics, enabling interaction with virtual sensors and actuators.
- Simulates sensors such as LiDAR and odometry.
- Compatible with autonomous navigation algorithms.
- Supports 1 or 2 vehicles simultaneously.
- For a Spanish version of this file, click [here](README_es.md).

## 1. Simulator Installation

### 1.1 Install Dependencies and Simulator
- **ROS 2 Humble**: Follow the instructions [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **F1TENTH Gym**
```bash
  git clone https://github.com/f1tenth/f1tenth_gym
  sudo apt install python3-pip
  cd f1tenth_gym && pip3 install -e .
```

Install the simulator:
```bash
cd $HOME
git clone https://github.com/widegonz/F1Tenth-Repository.git
```

https://github.com/user-attachments/assets/984a755c-6259-4271-9f88-51f68e425af5

#### 1.2. Installation of simulator dependencies
Dependencies are installed using rosdep:

```bash
cd
sudo apt install python3-rosdep2
rosdep update
source /opt/ros/humble/setup.bash
cd ~/F1Tenth-Repository
rosdep install -i --from-path src --rosdistro humble -y
```

https://github.com/user-attachments/assets/4c3ea5fb-4486-459f-beb7-bb7e8efa2a34

#### 1.3. Update Map Path in sim.yaml

To ensure that the simulator can locate the maps correctly, we need to modify the default path in the `sim.yaml` file, which is as follows:

`/home/your_user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/levine`

In this case, the part that says `your_user` will be replaced with the username that has been set on your computer. Here’s how to do it:

https://github.com/user-attachments/assets/f66baff8-8d29-4306-a2e5-d5a50eb888f3

With the path modified, all we have to do is navigate to the main folder and compile our workspace.

```bash
cd ~/F1Tenth-Repository
colcon build
```

#### 1.4. Start the simulation

To start the simulation, make sure you have run `source` in your workspace
```bash
cd ~/F1Tenth-Repository
source install/setup.bash
```

Then we use the following command to load the simulator:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

The first time you run the command, the simulator will take a little while to load the robot model; subsequent times, it will start up more quickly.

https://github.com/user-attachments/assets/1791be99-73b6-4a22-acc2-6efa6fd40e5f

#### 1.4 Execution Issues
If you encounter an error when running the simulator's `.launch` file, you can use the following command to remove a library that causes errors on certain computers:
```bash
python3 -m pip uninstall -y coverage
sudo apt purge -y python3-coverage
```

Once the library has been removed, we recompile the repository:
```bash
cd ~/F1Tenth-Repository
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

And the simulator is restarted:
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## 2. Repository Structure

The repository provided was developed based on the various tasks that can be performed within the simulator. Additionally, it includes practical information provided by the competition organizers to help users learn key aspects of autonomous navigation.

<p align="center">
  <img src="img/repo.png" />
  <br />
  <em>Repository Contents</em>
</p>

#### 2.1. `Laboratories` Folder

This folder contains exercises provided by the competition organizers to help students learn how to use the simulator and the actual robot. There are a total of 7 exercises, covering the following topics:
- Lab 1: Introduction to ROS 2
- Lab 2: Automatic Emergency Braking
- Lab 3: Wall Follower (Control Algorithm)
- Lab 4: Follow the Gap (Control Algorithm)
- Lab 5: Pure Pursuit (Motion Planning)
- Lab 6: Rapidly-exploring Random Tree (RRT)
- Lab 7: Model Predictive Control (MPC)

#### 2.2. `Controllers` Package
As the name suggests, this folder is a ROS package containing several nodes designed to control the robot’s movement—that is, to send direction and speed messages to make the robot follow a specific behavior. It is important to note that these nodes are merely a set of basic functions and structures intended to serve as a guide for each student to write their own version of the code.

<p align="center">
  <img src="img/controllers.png" />
  <br />
  <em>Contents of the Controllers folder</em>
</p>

There are a total of 6 nodes, each corresponding to a lab exercise. The first lab exercise is an introduction to ROS, so it is not included in this folder.

- `ttc_node.py`: A node designed to contain the code necessary to ensure automatic emergency braking.
- `pid_node.py`: Node designed to contain a PID controller for tracking one of the walls on the map.
- `gap_node.py`: Node designed to contain the reactive `Follow the Gap` control and allow the robot to avoid obstacles.
- `purepursuit_node.py`: Node designed to contain the code necessary for the robot to follow a predefined path.
- `rrt_node.py`: Node designed to contain the code necessary to use the RRT algorithm and generate local trajectories that allow the robot to move and avoid obstacles.
- `mpc_node.py`: Node designed to contain the code necessary for the robot to use the `MPC` controller to move.

#### 2.3. `f1tenth_gym_ros` Package

This package contains everything needed for the `f1tenth_gym` simulator to work with ROS, allowing you to use the simulator with Rviz. Generally speaking, few changes will be made to this package, aside from the `maps` and `config` folders.

<p align="center">
  <img src="img/bridge.png" />
  <br />
  <em>Contents of the f1tenth_gym_ros folder</em>
</p>

**`maps` folder**

As the name suggests, this is where we’ll add all the maps we need for simulation. To add a map, we basically need two files:
- A `.png` file, which represents the image of our map.
- A `.yaml` file, which is a map configuration file in ROS 2 used for autonomous navigation.

Whenever a new map needs to be added, we need both files, which are generated using SLAM tools for simultaneous localization and mapping.

A basic structure of a `.yaml` file used for the simulator is as follows:

```bash
image: levineB.png
resolution: 0.050000
origin: [-25.0, -12.75, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196 que es este arhivo .yaml?
```

**Explanation of each parameter:**  
1. **`image: levineB.png`**  
   - This is the grayscale image representing the map of the environment.  
   - It is typically generated using tools such as **SLAM Toolbox**, **gmapping**, or **map_server**.  

2. **`resolution: 0.050000`**  
   - Defines the size of each pixel in **meters/pixel**.  
   - In this case, each pixel corresponds to **5 cm** in the real world.  

3. **`origin: [-25.0, -12.75, 0.000000]`**  
   - Defines the position of the map’s origin in the global coordinate system (X, Y, Theta).  
   - Here, the map origin is at (-25.0, -12.75) meters and **0 radians** of rotation.  

4. **`negate: 0`**  
   - Indicates whether the image color should be inverted.  
   - `0` means that **dark values** represent obstacles and **light values** are clear areas.  

5. **`occupied_thresh: 0.65`**  
   - Threshold limit for considering a pixel **occupied** (obstacle).  
   - If the pixel intensity is **greater than 65%**, it is considered an **obstacle**.  

6. **`free_thresh: 0.196`**  
   - Threshold limit for considering a pixel **free**.  
   - If the pixel intensity is **less than 19.6%**, it is considered a **navigable area**.  

**`config` folder**
This folder contains a single file named `sim.yaml`. This file contains the settings required for the simulator to function properly. Specifically, we will only modify three sections:

If you want to change the map being used, modify this line:
```bash
    map_path: '/home/israel/F1Tenth-Repository/src/f1tenth_gym_ros/maps/levine'
```

In this case, it uses the `levine.png` map. To ensure we use a different map, we need to place the PNG file in the maps folder and modify the end of the line mentioned above to include the name of our map.

https://github.com/user-attachments/assets/d1302653-d5e6-4022-9df9-a0113e9bdb12

The other section to be modified is:
```bash
    # opponent parameters
    num_agent: 1
```
Depending on whether we want 1 or 2 robots to appear in our simulation.

https://github.com/user-attachments/assets/a4a5d284-6bfa-42c0-ae58-5a4694da63ec

And finally:
```bash
    # ego starting pose on map
    sx: 0.0
    sy: 0.0
    stheta: 0.0

    # opp starting pose on map
    sx1: 9.5
    sy1: 8.5
    stheta1: 3.14
```
This defines the starting position of our robots on the map. If it is set to (0,0,0), the robot appears at the origin defined in the `.yaml` file. If we want it to appear at a different position, we assign values to `x` and `y` to change the position and to `tetha` to modify the orientation.

https://github.com/user-attachments/assets/6269e319-5558-458c-a9b0-5b7240a9e0cd

#### 2.4. `path_planning` Package

When developing autonomous navigation algorithms, it is necessary to include those that, based on a map, generate a global path while minimizing various parameters such as distance traveled, speed, and vehicle dynamics, among others.

This ROS package is designed to store all algorithms developed for the purpose of performing global route planning for our mobile robot.

<p align="center">
  <img src="img/path_planning.png" />
  <br />
  <em>Contents of the path_planning folder</em>
</p>

The package contains a file named `waypoint_recorder.py`, which serves as a template for a ROS 2 node for an odometry-based waypoint logger. Its purpose is to record the robot’s positions (x, y, orientation w) in a CSV file as it moves, with the goal of serving as the basis for developing a waypoint-based navigation system. It is one of the algorithms mentioned for the `Pure Pursuit` controller, which uses waypoints to follow a defined trajectory.

#### 2.5. `Waypoints` Folder

A folder designed to hold `.csv` files that represent the various waypoints needed to define an overall path for the mobile robot to follow.

**What are Waypoints?**  
**Waypoints** are reference points in space (generally expressed as (x, y, theta) coordinates) that indicate key positions a robot or vehicle must pass through during navigation.  

In the context of robotics and ROS 2, a waypoint is typically defined as:  
- **(x, y):** Position in the plane (or in 3D if aerial or underwater navigation is used).  
- **(theta) or (w):** The robot’s orientation at that point (in radians or quaternions).  

**Why are waypoints useful?**  
Waypoints are fundamental in multiple robotics and navigation applications:  

1. **Autonomous navigation:** They are used as waypoints to help a robot plan its path from a starting point to a destination.  

2. **Mapping and exploration:** In SLAM (Simultaneous Localization and Mapping), waypoints can mark explored locations or points of interest.  

3. **Trajectory tracking:** A drone or mobile robot can follow a series of predefined waypoints to move in a controlled manner.  

4. **Motion recording and analysis:** Waypoints can be stored to analyze a robot’s trajectory after a simulation or real-world test. 

## 3. Topics in the simulation
When we run the simulation with a single vehicle and use the `ros2 topic list` command, we get the following topics:

<p align="center">
  <img src="img/single_agent.png" />
  <br />
  <em>Topics with only one vehicle simulated</em>
</p>

The most important of these are:
- `/ego_racecar/odom`: This corresponds to the robot’s odometry, where we can obtain position and orientation data.
- `/scan`: A topic that sends messages from the LIDAR sensor, which provides information about the environment.
- `/map`: The simulation map, which contains `OccupancyGrip`-type messages.
- `/tf` and `/tf_static`: Publish the static and dynamic transformation trees present in the simulation.
- `/drive`: Where messages to move the robot are published, specifically `AckermannDriveStamped` messages.
- `/initialpose`: Topic that allows resetting the vehicle's position.

When we run the simulation with two vehicles and use the `ros2 topic list` command, we get the following topics:
<p align="center">
  <img src="img/two_agents.png" />
  <br />
  <em>Topics with 2 simulated vehicles</em>
</p>

In this case, we have the same topics as we did with a single vehicle in the simulation, and in addition, the topics for the opponent’s vehicle are added:
- `/opp_scan`: Messages from the opponent’s LIDAR.
- `/opp_racecar/odom`: The opponent’s odometry.
- `/opp_drive`: Where messages are published to move the opponent’s robot, such as `AckermannDriveStamped` messages.
- `/goal_pose`: A topic that allows you to reset the opponent’s vehicle position.

**Note:** When using two agents—that is, two vehicles in the simulation—we must simultaneously publish driving messages to the `/drive` and `/opp_drive` topics. If this is not done and commands are published to only one topic, neither of the two vehicles will move. The simulator detects that messages are being posted to both vehicle movement topics simultaneously, and only then does it allow movement for both; otherwise, it does not.

## 4. Keyboard Teleoperation
The `teleop_twist_keyboard` keyboard teleoperation node is also installed as part of the simulation dependencies. To enable keyboard teleoperation, set `kb_teleop` to True in `sim.yaml`. After launching the simulation, in another terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
To move the vehicle in the simulator, use the following keys:
- `i`: to move forward.
- `u`: to move forward and to the left.
- `o`: to move forward and to the right.
- `k`: to stop all movement.
- `,`: to move backward.
- `m`: to move backward and to the left.
- `.`: to move backward and to the right

https://github.com/user-attachments/assets/1961d6bf-369a-4c58-9654-8d4c7cdefe19
