# MicroKiap Training XR Controller

This is the repository for MicroKiap's XR controller code.
The MicroKiap XR training simulation utilises a custom controller and Intel Realsense D405 depth camera for controller input.
To construct and operate the controller, the following code is provided:

 - `generate_marker.py` generates an Aruco marker used to track the movement of the controller
 - `pose_tracking.py` runs the main camera loop to track the position of the controller
 - `Joystick_Control_Test.ino` is used to flash the controller's ESP32
 - The remaining files contain necessary supporting code to ensure the controller and camera function

This controller was designed to run in tandem with the XR simulation [here]().

## Prerequisites

 - [Arduino IDE](https://www.arduino.cc/en/software/)
 - [Python 3](https://www.python.org/downloads/) (Tested on 3.9+)
 - Materials for 1 MicroKiap:tm: XR Controller
 - An Intel Realsense D405 Depth Camera

## Setup instructions

 1. Construct a controller using the provided schematics
 1. Clone this repository
 1. Flash the ESP32 with `Joystick_Control_Test.ino` using Arduino IDE
    1. Within Arduino IDE, install Espressif System's `esp32` from the board manager
    1. Still in Arduino IDE, install lemmingDev's `ESP32-BLE-Gamepad` from the library manager
    1. Open `Joystick_Control_Test.ino`
    1. Connect to the controller's ESP32 board and select the matching board and port
    1. Flash the code onto the ESP32
 1. In a command terminal, navigate to the directory containing this repository
 1. Install all Python dependencies using `pip install -r requirements.txt`
 1. Run `python3 generate_marker.py`
 1. Connect the Intel Realsense D405 camera to your system
 1. Run `python3 pose_tracking.py`

## Configuration

The camera settings can be configured from `config.json`.
Most of the settings should be self-explanatory and left at their defaults.

The packet transfer rate of the camera is found in `pose_tracking.py` and can be configured, also as required.
