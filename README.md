# CubicDoggo: Upgrade to Incorporate IMU/LiDAR

Derived from <a href="https://github.com/SphericalCowww/CubicDoggo">GitHub</a>. Copy <a href="https://github.com/SphericalCowww/CubicDoggo/tree/main/src/my_robot_description/mesh/CADv1">CADv1</a> under ``CubicDoggo_06R/tree/main/src/my_robot_description/mesh/``. Also, follow this chapter to set up the servos: ``https://github.com/SphericalCowww/CubicDoggo/tree/main#running-a-single-servo-on-ros2``

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

    sudo apt update && sudo apt install -y i2c-tools
    sudo apt update && sudo apt install gpiod -y
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
    # update /home/cubicdoggo/Documents/CubicDoggo/src/my_robot_bringup/launch/cubic_doggo.with_lifecycle.launch.py
    echo timer | sudo tee /sys/class/leds/ACT/trigger                          # trigger LED first
    sudo chmod -R 777 /sys/class/leds/ACT/                                     # enable LED to indicate battery condition
    sudo chmod -R 777 /sys/class/hwmon/hwmon*/                                 # enable reading of power alarm/voltage
    # test the node 
    echo 0 > /tmp/fake_alarm                                                   # create a fake alarm by
    ros2 run my_robot_peripheral rasp_pi_peripheral_node --ros-args -p alarm_path:=/tmp/fake_alarm
    echo 1 > /tmp/fake_alarm                                                   # create a fake alarm by

## Tracking the variables

Run the robot first:

    cd CubicDoggo
    colcon build
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot.with_lifecycle.launch.py

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
    # start => locate the following
    ## /imu/euler
    ## /joint_states
    # OK
    # on the left panel, expand /imu and /joint_states and drag the variable to the center for live plotting

## References:
- ROS1 Packages for CHAMP Quadruped Controller (<a href="https://github.com/chvmp/champ">GitHub</a>) => node based IMU control with classical walk gait
- Spot Micro (<a href="https://github.com/mike4192/spotMicro">GitHub</a>, <a href="https://spotmicroai.readthedocs.io/en/latest/">link</a>) => node based IMU control with classical walk gait
- Pupper V3 (<a href="https://pupper-v3-documentation.readthedocs.io/en/latest/">link</a>) => ros2_control based IMU control with RL walk gait


