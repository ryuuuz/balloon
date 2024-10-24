# bloom

A project based on Arduino.

## Overview

This project is designed to explore and test various functionalities using different microcontrollers and communication technologies, such as LoRa, video streaming, and development boards like ESP32 and Seeeduino Xiao.

## Project Structure

### lora_p2p

This directory contains the code for testing LoRa communication in peer-to-peer mode.

- **lora_p2p.ino**: The main Arduino script for testing LoRa connectivity between devices.

### video_streaming

This directory focuses on testing camera functionality and video streaming using supported hardware.

- **app_httpd.cpp**: The main HTTP server application for video streaming.
- **camera_index.h**: Index page for camera streaming.
- **camera_pins.h**: Configuration file for camera pin connections.
- **Video_Streaming.ino**: The main Arduino script for setting up video streaming.

### seeduino

Contains the code specific to Seeeduino Xiao development board.

- **seeduino.ino**: The main Arduino script for Seeeduino Xiao board functionality testing.

### esp32s3

Contains the code specific to ESP32-S3 development board.

- **image_information**: The Arduino project for testing ESP32-S3 image information.
- **image_segmentation**: The Arduino project for testing ESP32-S3 image segmentation.

## Future Plans

This project is under active development, and additional functionalities or modules may be added in the future.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
