# CubicDoggo: Upgrade to Incorporate IMU/LiDAR

Derived from <a href="https://github.com/SphericalCowww/CubicDoggo">GitHub</a>.


## Ingredients

### Hardware requirements

| device | models | count | specification |
| - | - | - | - |
| IMU | Adafruit <a href="https://cdn-learn.adafruit.com/downloads/pdf/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black.pdf">BNO055 [ADA2472]</a> | 1 |  |

## Testing IMU/LiDAR

### Testing IMU

https://blog.berrybase.de/wp-content/uploads/2023/12/Abbildung_11-1536x882.png.webp

  * Vin to 1
  * GND to 9 
  * SDA to 3
  * SCL to 5
  * RST to 11

    sudo apt update && sudo apt install -y i2c-tools
    sudo i2cdetect -y 1                                                 # if 28 lights up, it's alive
    
    sudo apt install -y python3-venv python3-pip python3-lgpio
    cd python_test_scripts
    python3 -m venv env
    source env/bin/activate
    pip install adafruit-circuitpython-bno055 rpi-lgpio
    python bno_test.py
    

## References:
- ROS Packages for CHAMP Quadruped Controller (<a href="https://github.com/chvmp/champ">GitHub</a>)
