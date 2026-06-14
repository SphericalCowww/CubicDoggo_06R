# CubicDoggo: Upgrade to Incorporate IMU/LiDAR

Cubic Doggo 06R (High Mobility Type, sort of) is upgraded from the base Cubic Doggo <a href="https://github.com/SphericalCowww/CubicDoggo">GitHub</a>. Copy <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">CADv1</a> under ``CubicDoggo_06R/tree/main/src/my_robot_description/mesh/``. Also, follow this chapter to set up the servos: ``https://github.com/SphericalCowww/CubicDoggo/tree/main#running-a-single-servo-on-ros2``

Demos: wobbly IMU (<a href="https://www.reddit.com/r/robotics/comments/1u1iql9/cubic_doggo_update_wobbly_imu/">Reddit</a>, <a href="https://www.reddit.com/r/ROS/comments/1u1j9yf/cubic_doggo_update_wobbly_imu/">Reddit</a>).

## Ingredients

### Hardware requirements

| device | models | count | specification |
| - | - | - | - |
| IMU | Adafruit <a href="https://www.adafruit.com/product/2472?srsltid=AfmBOopFaOJasrKIi1FkizYHaVd5CtUsoR6xX3qAALgU8sYoLY70Q55M">BNO055 [ADA2472]</a> | 1 | BNO055 has in-system Kalman Filter |

## Testing IMU/LiDAR

### Testing IMU

Following <a href="https://cdn-learn.adafruit.com/downloads/pdf/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black.pdf">link</a>. It's a bit outdated, so do change up a bit and actually connect via I2C instead for RaspPi > 3:

  * Vin to 1
  * GND to 9 
  * SDA to 3
  * SCL to 5
  * RST to 11

<img src="https://github.com/SphericalCowww/CubicDoggo_06R/blob/main/fig_IMUconnection.png" width="400">

Then do:

    sudo apt update && sudo apt install -y i2c-tools gpiod libgpiod-dev
    sudo gpiodetect
    sudo gpioset gpiochip4 17=1                                         # enable reset pin
    sudo i2cdetect -y 1                                                 # if 28 lights up, it's alive

    sudo usermod -aG i2c $USER
    sudo usermod -aG dialout $USER
    sudo reboot
    i2cdetect -y 1

To test BNO055 without ROS:
    
    sudo apt install -y python3-venv python3-pip python3-lgpio
    cd CubicDoggo_06R/python_test_scripts
    python3 -m venv env
    source env/bin/activate
    pip install adafruit-circuitpython-bno055 rpi-lgpio
    python bno_test.py
    
To test BNO055 with ROS:

    cd CubicDoggo_06R
    colcon build
    source install/setup.bash
    ros2 run my_robot_peripheral imu_bno055_node
    # on another terminal
    ros2 topic echo /imu/euler              # show IMU content
    ros2 topic hz /imu/euler                # show IMU read speed

### Checking RaspPi power

    for d in /sys/class/hwmon/hwmon*; do echo -n "$d: "; cat "$d/name"; done   # find the correct path for power alarm
    # update /home/cubicdoggo/Documents/CubicDoggo_06R/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    echo timer | sudo tee /sys/class/leds/ACT/trigger                          # trigger LED first
    sudo chmod -R 777 /sys/class/leds/ACT/                                     # enable LED to indicate battery condition
    sudo chmod -R 777 /sys/class/hwmon/hwmon*/                                 # enable reading of power alarm/voltage
    # test the node 
    echo 0 > /tmp/fake_alarm                                                   # create a fake alarm by
    ros2 run my_robot_peripheral rasp_pi_peripheral_node --ros-args -p alarm_path:=/tmp/fake_alarm
    echo 1 > /tmp/fake_alarm                                                   # create a fake alarm by

## Tracking the variables

Run the robot first:

    cd CubicDoggo_06R
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py

### Reading out variables from ROS

    # on another terminal for joint states
    ros2 topic echo /joint_states
    # on another terminal for IMU
    ros2 topic echo /imu/euler

### Tracking/plotting variables using PlotJuggler

To track the values (remember to connect the RaspPi to a monitor):

    # on another terminal for plotjuggler
    sudo apt install ros-jazzy-plotjuggler-ros
    ros2 run plotjuggler plotjuggler
    # load CubicDoggo_06R/monitor_plot.xml 
    # or starting over again:
    ## start => locate the following
    ### /imu/data
    ### /imu/euler
    ### /joint_states
    ## OK
    ## on the left panel, expand /imu and /joint_states and drag the variable to the center for live plotting

<img src="https://github.com/SphericalCowww/CubicDoggo_06R/blob/main/plotJuggler0.png" width="600">

## Running full robot

### Assembly and Launching URDF

Run the following to view the moving parts:

    cd CubicDoggo_06R
    colcon build
    source install/setup.bash
    ros2 launch my_robot_description cubic_doggo.rviz.launch.xacro.py

Notice that the placement of the first servo is different between the front and back legs.

### Launching with commands

Under: ``CubicDoggo_06R/src/my_robot_description/urdf/cubic_doggo.ros2_control.xacro``

   * use ``<plugin>mock_components/GenericSystem</plugin>`` if just want rViz
   * use ``<<plugin>cubic_doggo_namespace/HardwareInterfaceU2D2_cubic_doggo</plugin>`` if want to control hardware

Under: ``CubicDoggo_06R/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py``

   * comment out ``rviz_node`` if don't want rViz to show
   * comment out ``joy_driver_node`` and ``joy_controller_node`` if don't want the joystick controller
   * comment out ``peripheral_node`` if don't want warning from low power

Start the robot (skip launching rviz_node, joy_driver_node, or joy_controller_node if needed):

    cd CubicDoggo_06R
    colcon build
    source install/setup.bash
    # comment in rvis node in CubicDoggo_06R/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    ros2 launch my_robot_bringup cubic_doggo.with_lifecycle.launch.py
    # on another terminal
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "rest"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "stand"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "sit"}"
    ros2 topic pub -1 /leg_set_named example_interfaces/msg/String "{data: "bow"}"
    ros2 topic pub -1 /leg_set_joint example_interfaces/msg/Float64MultiArray "{data: [0, 3.14, 3.14, 3.54]}"
    ros2 topic pub -1 /leg_set_pose my_robot_interface/msg/CubicDoggoLegPoseTarget "{leg_index: 0, x: -0.092, y: 0.053, z: 0.135}" 
    ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: true}"
    ros2 service call /leg_walk_toggle std_srvs/srv/SetBool "{data: false}"

Check out all the gait options under ``CubicDoggo_06R/src/my_robot_commander/src/cubic_doggo_lifecycle.cpp``:

    # double maxVelScale = 1.0, maxAccScale = 1.0;
    # int    waypoint_N     = 100;       // number of waypoints for each cycle,      default 100
    # double waypoint_dt    = 0.01;      // second for each waypoint,                default 0.01
    # double IK_bufferTime  = 0.10;      // time at end of cycle buffer for IK calc, default 0.10
    # double swing_fraction = 0.50;      // creep < 0.25 < stable trot < 0.5 < trot
    # double lift = 0.02, x_stride_max = 0.02, y_stride_max = 0.025, x_shift = 0.0, y_shift = 0.0;
    # double x_stride = 0.0, y_stride = 0.0;

### Testing the joystick controller:

    ls /dev/input/js*
    # output: /dev/input/js0
    # otherwise do: sudo jstest /dev/input/js0
    ros2 run joy joy_enumerate_devices
    # if no device found
    sudo usermod -aG input $USER
    sudo reboot

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 run joy joy_node
    # on another terminal
    ros2 run my_robot_controller cubic_doggo_joy_control
    # on yet another terminal
    ros2 node info /cubic_doggo_joy_control    
    ros2 topic info /joy --verbose
    ros2 topic echo /joy
    
### Launching at the start of RaspPi

    chmod +x /home/cubicdoggo/Documents/CubicDoggo_06R/start_robot.sh
    sudo cp /home/cubicdoggo/Documents/CubicDoggo_06R/robot_startup.service /etc/systemd/system/robot_startup.service
    sudo chmod 644 /etc/systemd/system/robot_startup.service
    sudo systemctl enable robot_startup.service                                # now will start at reboot
    sudo systemctl daemon-reload                                               # reload whenever there is a change
    # sudo systemctl restart robot_startup.service                             # restart, even if is rrunning
    # sudo systemctl stop    robot_startup.service                             # stop right now, kill all relevant nodes
    # sudo systemctl disable robot_startup.service                             # disable at reboot
    # journalctl -u robot_startup.service -n 100 > start_robot_output.txt      # to check the output
    # journalctl -u robot_startup.service -f

### IMU balancing

Standing up and then changing to a slope. Unstable IMU with lots of oscillations:

<img src="https://github.com/SphericalCowww/CubicDoggo_06R/blob/main/plotJuggler1_standThenSlope.png" width="1000">

Walking gait, holding position:

<img src="https://github.com/SphericalCowww/CubicDoggo_06R/blob/main/plotJuggler2_walkingGait.png" width="1000">


## References:
- ROS1 Packages for CHAMP Quadruped Controller (<a href="https://github.com/chvmp/champ">GitHub</a>) => node based IMU control with classical walk gait
- Spot Micro (<a href="https://github.com/mike4192/spotMicro">GitHub</a>, <a href="https://spotmicroai.readthedocs.io/en/latest/">link</a>) => node based IMU control with classical walk gait
- Pupper V3 (<a href="https://pupper-v3-documentation.readthedocs.io/en/latest/">link</a>) => ros2_control based IMU control with RL walk gait


