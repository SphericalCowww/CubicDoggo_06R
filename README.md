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

To track the values (remember to connect the RaspPi to a monitor):

    sudo apt install ros-jazzy-plotjuggler-ros
    ros2 run plotjuggler plotjuggler
    # press start
    # find /imu/euler => OK
    # on the left panel => imu => euler => drags x, y, z into the plot

## References:
- ROS1 Packages for CHAMP Quadruped Controller (<a href="https://github.com/chvmp/champ">GitHub</a>) => node based IMU control with classical walk gait
- Spot Micro (<a href="https://github.com/mike4192/spotMicro">GitHub</a>, <a href="https://spotmicroai.readthedocs.io/en/latest/">link</a>) => node based IMU control with classical walk gait
- Pupper V3 (<a href="https://pupper-v3-documentation.readthedocs.io/en/latest/">link</a>) => ros2_control based IMU control with RL walk gait


