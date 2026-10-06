# LiDAR SLAM Autonomous Robot

A custom 4-wheel differential drive robot with LiDAR-based SLAM and 
autonomous navigation using ROS 2 Humble on a Raspberry Pi 4.

## Hardware

- Raspberry Pi 4 (Ubuntu 22.04, arm64)
- Arduino Uno (motor control + encoder odometry)
- 2× L298N dual H-bridge motor drivers
- 4× JGB37-520 6V 90 RPM motors with quadrature encoders
- LD14P 2D LiDAR
- USB camera
- 11.7 V 5200 mAh Li-ion battery
- 2× XL4015E1 buck converters

## Software Stack

- ROS 2 Humble
- SLAM Toolbox (mapping)
- Nav2 (autonomous navigation)
- rosbridge + web_video_server (mobile dashboard)
- roslibjs (browser joystick)
- Custom serial bridge node (Pi ↔ Arduino)

## Architecture

Phone (Web Dashboard)  ─── WiFi ───►  Raspberry Pi (ROS 2)
                                        │
                                        ├── rosbridge_server   (:9090)
                                        ├── web_video_server   (:8080)
                                        ├── http.server        (:8081)
                                        ├── serial_bridge_node
                                        ├── ldlidar_sl_ros2
                                        ├── scan_republisher
                                        └── slam_toolbox / nav2

                                             │ USB serial
                                             ▼
                                        Arduino Uno
                                             │
                                    ┌────────┴────────┐
                                    │                 │
                                 L298N #1          L298N #2
                                    │                 │
                              Left motors         Right motors
                              + encoders          + encoders

## Repository Structure
# slamlidarRos2
