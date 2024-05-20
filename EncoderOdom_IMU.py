import serial
import time
import math

# Define serial ports for Arduino and IMU
arduino_port = '/dev/ttyUSB0'  # Change to your Arduino serial port
imu_port = '/dev/ttyUSB1'      # Change to your IMU serial port


# Fusion parameters
alpha = 0.98  # Complementary filter coefficient

# Initialize pose
x = 0.0
y = 0.0
theta = 0.0

# Initialize serial connections
arduino_serial = serial.Serial(arduino_port, 9600)
imu_serial = serial.Serial(imu_port, 9600)

def read_arduino():
    """Read odometry data from Arduino."""
    line = arduino_serial.readline().decode('utf-8').strip()
    data = list(map(float, line.split(',')))
    return data  # [x, y, theta]

def read_imu():
    """Read IMU data."""
    line = imu_serial.readline().decode('utf-8').strip()
    imu_data = float(line)  # Assuming we get gyroZ as a single float
    return imu_data

def fuse_data(odom_data, gyroZ, delta_time):
    global x, y, theta

    # Extract odometry data
    odom_x, odom_y, odom_theta = odom_data

    # Complementary filter for angle
    theta_odom = odom_theta
    theta_imu = theta + gyroZ * delta_time

    theta = alpha * theta_odom + (1 - alpha) * theta_imu

    # Update position using odometry data
    x = odom_x
    y = odom_y

    return x, y, theta

def main():
    global x, y, theta

    last_time = time.time()

    while True:
        current_time = time.time()
        delta_time = current_time - last_time
        last_time = current_time

        # Read data from Arduino and IMU
        odom_data = read_arduino()
        gyroZ = read_imu()

        # Fuse the data
        x, y, theta = fuse_data(odom_data, gyroZ, delta_time)

        # Print the updated pose
        print(f"X: {x:.2f}, Y: {y:.2f}, Theta: {theta:.2f}")

        time.sleep(0.1)  # Adjust as needed

if __name__ == "__main__":
    main()
