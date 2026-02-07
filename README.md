# micro-ROS Mecanum Robot Firmware

This repository contains the firmware for a Mecanum wheeled robot powered by an **ESP32**, designed for **ROS 2 Humble** using **micro-ROS**.

## 🚀 Features
- **ROS 2 Humble Compatible**: Native micro-ROS integration.
- **Mecanum Kinematics**: Omnidirectional movement support.
- **High-Performance IMU**: Integrated **Bosch BNO055** 9-axis IMU (Absolute Orientation).
- **Odometry**: Computes wheel odometry and publishes `odom/unfiltered`.
- **PID Control**: Closed-loop motor speed control.
- **50 Hz Control Loop**: Optimized for smooth navigation.

## 🛠 Hardware Configuration
- **Microcontroller**: ESP32 (Dev Module)
- **IMU**: BNO055 (I2C Address: `0x28` or `0x29`)
- **Motor Drivers**: BTS7960 / L298N (Configurable)
- **Encoders**: Quadrature Encoders

## 📦 Topics

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/Twist` | Subscriber | Velocity commands for robot movement. |
| `/odom/unfiltered` | `nav_msgs/Odometry` | Publisher | Raw wheel odometry data. |
| `/imu/data` | `sensor_msgs/Imu` | Publisher | BNO055 IMU data (Orientation, Angular Velocity, Linear Accel). |

## ⚙️ Installation

### 1. Requirements
- [VS Code](https://code.visualstudio.com/)
- [PlatformIO Extension](https://platformio.org/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)

### 2. Build & Upload
1. Clone the repository:
   ```bash
   git clone https://github.com/mirackurtak7/microros_mecanum_robot_firmware.git
   cd microros_mecanum_robot_firmware
   ```
2. Open the folder in VS Code.
3. Connect your ESP32 via USB.
4. Upload the firmware:
   ```bash
   pio run --target upload
   ```

## 📡 Usage (Running micro-ROS Agent)

To communicate with the ESP32, you need to run the **micro-ROS Agent**. You can use Docker or a native installation.

### Option 1: Docker (Recommended)
This is the easiest method. Ensure Docker is installed.

```bash
# Run the agent for Serial connection (Change /dev/ttyUSB0 to your port)
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
```

### Option 2: Snap (Linux)
```bash
sudo snap install micro-ros-agent
micro-ros-agent serial --dev /dev/ttyUSB0 -b 115200
```

### Option 3: Verification
Once the agent is running and connected:
1. Reset the ESP32 (press the EN button).
2. Check active nodes:
   ```bash
   ros2 node list
   # Output: /linorobot_esp32_node
   ```
3. Monitor topics:
   ```bash
   ros2 topic echo /imu/data
   ros2 topic echo /odom/unfiltered
   ```

## 🔧 Configuration
Modify `include/config.h` to adapt to your hardware:
- **Pin Definitions**: Define Encoder and Motor pins.
- **Robot Dimensions**: Set `WHEEL_DIAMETER`, `LR_WHEELS_DISTANCE`, etc.
- **Motor RPM**: Set `MOTOR_MAX_RPM`.

## 🤝 Contributing
1. Fork the repository.
2. Create your feature branch (`git checkout -b feature/AmazingFeature`).
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`).
4. Push to the branch (`git push origin feature/AmazingFeature`).
5. Open a Pull Request.
