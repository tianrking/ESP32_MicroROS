## Dependencies

```bash
git clone -b release/v5.2 https://github.com/espressif/esp-idf ~/esp-idf_5.2
cd ~/esp-idf_5.2
git submodule update --init --recursive
export IDF_PATH=~/esp-idf_v5.2

git clone https://github.com/micro-ROS/micro_ros_espidf_component ~/micro_ros_espidf_component -b humble
export IDF_ROS_PATH=~/micro_ros_espidf_component
```

## Example

```
git clone https://github.com/tianrking/ESP32_MicroROS ~/ESP32_MicroROS
cd ~/ESP32_MicroROS
```

Setup ENV

```bash
source ~/.espressif/python_env/*/bin/activate
pip3 install catkin_pkg lark-parser empy colcon-common-extensions
. $IDF_PATH/export.sh
```

To Build 

```bash
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


TEST OK:
esp-idf5.2 with microros humbleon esp32
