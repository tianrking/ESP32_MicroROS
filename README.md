
# micro-ROS component for ESP-IDF

This component has been tested in ESP-IDF v4.1, v4.2, v4.3 and v4.4 with ESP32, ESP32-S2, ESP32-S3 and ESP32-C3.

## Dependencies

This component needs `colcon` and other Python 3 packages inside the IDF virtual environment in order to build micro-ROS packages:

```bash
git clone  https://github.com/espressif/esp-idf ~/esp-idf
cd ~/esp-idf
git submodule update --init --recursive
export IDF_PATH=~/esp-idf
git clone https://github.com/tianrking/ESP32_MicroROS.git ~/ESP32_MicroROS
cd ~/ESP32_MicroROS
. $IDF_PATH/export.sh
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
```

## Middlewares available

This package support the usage of micro-ROS on top of two different middlewares:
- [eProsima Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/): the default micro-ROS middleware.
- [embeddedRTPS](https://github.com/embedded-software-laboratory/embeddedRTPS): an experimental implementation of a RTPS middleware compatible with ROS 2.

In order to select it, use `idf.py menuconfig` and go to `micro-ROS Settings > micro-ROS middleware`
## Usage

You can clone this repo directly in the `components` folder of your project.

If you encounter issues during the build process, ensure that you are running in a clean shell environment _without_ the ROS 2 setup script sourced.

## Example

In order to test a int32_publisher example:

```bash
. $IDF_PATH/export.sh
cd examples/int32_publisher
# Set target board [esp32|esp32s2|esp32s3|esp32c3]
idf.py set-target esp32
idf.py menuconfig
# Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings
idf.py build
idf.py flash
idf.py monitor
```

To clean and rebuild all the micro-ROS library:

```bash
idf.py clean-microros
```

Is possible to use a micro-ROS Agent just with this docker command:

```bash
# UDPv4 micro-ROS Agent
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## Using serial transport

By default, micro-ROS component uses UDP transport, but is possible to enable UART transport or any other custom transport setting the `colcon.meta` like:

```json
...
"rmw_microxrcedds": {
    "cmake-args": [
        ...
        "-DRMW_UXRCE_TRANSPORT=custom",
        ...
    ]
},
...
```

An example on how to implement this external transports is available in `examples/int32_publisher_custom_transport`.

Available ports are `0`, `1` and `2` corresponding `UART_NUM_0`, `UART_NUM_1` and `UART_NUM_2`.

Is possible to use a micro-ROS Agent just with this docker command:

```bash
# Serial micro-ROS Agent
docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev [YOUR BOARD PORT] -v6
```

