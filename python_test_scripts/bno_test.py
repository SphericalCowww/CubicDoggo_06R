import time
import board
import busio
import digitalio
import adafruit_bno055

def main():
    i2c = busio.I2C(board.SCL, board.SDA)

    bno_reset = digitalio.DigitalInOut(board.D17)
    bno_reset.direction = digitalio.Direction.OUTPUT
    bno_reset.value = False
    time.sleep(0.1)
    bno_reset.value = True 
    time.sleep(0.7)

    sensor = adafruit_bno055.BNO055_I2C(i2c)
    print("Connection established! Reading IMU data... (Ctrl+C to exit)")
    while True:
        heading, roll, pitch = sensor.euler
        print(f"Heading: {heading:6.2f}° | Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}°", end="\r")
        time.sleep(0.2)

if __name__ == "__main__": main()
